#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  nh_ = nh;
 
  tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
  wait_for_params(nh_);

  expVol_ = 0;

  ROS_INFO("%s: Waiting for first base to world transform ...", nh_->getNamespace().c_str());
  while( !tfBuffer_.canTransform(worldFrameId_, baseFrameId_, ros::Time(0)) );
    baseToWorld_ = tfBuffer_.lookupTransform(worldFrameId_, baseFrameId_, ros::Time(0));

  for(int i=0; i<nCams_; i++)
  {
    phCamsOpt_.push_back( ph_cam(camInfoK_.row(i).data(), camRes_.row(i).data(), discInt_.row(i).data()) );
    phCamsBase_.push_back(phCamsOpt_[i]);
    phCamsBase_[i].transform(camToBase_[i]);
    phCamsBase_[i].print_polytope();
  }

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);

  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);
  lookaheadPub_ = nh->advertise<geometry_msgs::PointStamped>("lookahead_out", 10);
  compTimePub_ = nh->advertise<std_msgs::Float64>("compute_time_out", 10);

  rrtTree_ = new rrt(rrtNNodes_, scanBnds_[0], scanBnds_[1], rrtRadNear_, rrtDelDist_, radRob_, rrtFailItr_, octDist_);

  pathPoses_.resize(0);
  path_.resize(0,0);

  ROS_INFO("%s: Waiting for the input map ...", nh->getNamespace().c_str());
  while( (isInitialized_ & 0x01) != 0x01 )
	  ros::spinOnce();
}

// ***************************************************************************
void scan_plan::wait_for_params(ros::NodeHandle* nh)
{

  //while(!nh->getParam("distance_interval", distInt_));

  while(!nh->getParam("n_rrt_nodes", rrtNNodes_));
 
  std::vector<double> minBnds, maxBnds;
  while(!nh->getParam("min_bounds", minBnds));
  while(!nh->getParam("max_bounds", maxBnds));
  scanBnds_[0][0] = minBnds[0]; scanBnds_[0][1] = minBnds[1]; scanBnds_[0][2] = minBnds[2];
  scanBnds_[1][0] = maxBnds[0]; scanBnds_[1][1] = maxBnds[1]; scanBnds_[1][2] = maxBnds[2];

  while(!nh->getParam("near_radius_rrt", rrtRadNear_));
  while(!nh->getParam("delta_distance_rrt", rrtDelDist_));
  while(!nh->getParam("robot_radius", radRob_));
  while(!nh->getParam("n_fail_iterations_rrt", rrtFailItr_));

  std::vector<double> camRes;
  while(!nh->getParam("camera_resolution", camRes));
  nCams_ = camRes.size() / 3;

  camRes_.resize(nCams_,3);
  for(int i=0; i<nCams_; i++)
    for(int j=0; j<3; j++)
      camRes_(i,j) = camRes[i*3+j];

  std::vector<double> camInfoK;
  while(!nh->getParam("camera_info_k", camInfoK));
  camInfoK_.resize(nCams_,9);
  for(int i=0; i<nCams_; i++)
    for(int j=0; j<9; j++)
      camInfoK_(i,j) = camInfoK[i*9+j];

  std::vector<double> discInt;
  while(!nh->getParam("discretization_interval", discInt));
  discInt_.resize(nCams_,3);
  for(int i=0; i<nCams_; i++)
    for(int j=0; j<3; j++)
      discInt_(i,j) = discInt[i*3+j];
  
  while(!nh->getParam("base_frame_id", baseFrameId_));
  while(!nh->getParam("world_frame_id", worldFrameId_));

  std::vector<std::string> camFrameId;
  while(!nh->getParam("cam_frame_id", camFrameId));

  double timeIntPhCam, timeIntReplan;
  while(!nh->getParam("time_interval_pose_graph", timeIntPhCam));
  while(!nh->getParam("time_interval_replan", timeIntReplan));

  timerPhCam_ = nh->createTimer(ros::Duration(timeIntPhCam), &scan_plan::timer_ph_cam_cb, this);
  timerReplan_ = nh->createTimer(ros::Duration(timeIntReplan), &scan_plan::timer_replan_cb, this);
  timerReplanPeriod_ = timeIntReplan;

  while(!nh->getParam("c_gain", cGain_));

  while(!nh->getParam("lookahead_path_len", lookaheadDist_));
  while(!nh->getParam("n_hist_poses_exploration_dir", nHistPoses_));

  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());

  for(int i=0; i<nCams_; i++)
  {
    while( !tfBuffer_.canTransform(baseFrameId_, camFrameId[i], ros::Time(0)) );
      camToBase_.push_back( tfBuffer_.lookupTransform(baseFrameId_, camFrameId[i], ros::Time(0)) );
  }
  
  while(!nh->getParam("min_explored_volume_rate", minExpVolRate_));

  ROS_INFO("%s: Cam to base transforms received", nh->getNamespace().c_str());
}

// ***************************************************************************
bool scan_plan::update_base_to_world()
{
  try
  {
    baseToWorld_ = tfBuffer_.lookupTransform(worldFrameId_, baseFrameId_, ros::Time(0));
    return true;
  }
  catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
    return false;
	}
}

// ***************************************************************************
void scan_plan::timer_ph_cam_cb(const ros::TimerEvent&)
{
  if( (isInitialized_ & 0x01) != 0x01 )
    return;

  update_base_to_world();

  if( (isInitialized_ & 0x02) == 0x02 )
  {
    Eigen::Vector3d currPos(baseToWorld_.transform.translation.x,
                            baseToWorld_.transform.translation.y,
                            baseToWorld_.transform.translation.z);
    Eigen::Vector3d lastPos(poseHist_.poses.back().position.x,
                            poseHist_.poses.back().position.y,
                            poseHist_.poses.back().position.z);

    if( (currPos-lastPos).norm() < 1.0) // 1.0 is distance between two poses in history, TODO: make it param
      return;
  }

  if( (isInitialized_ & 0x02) != 0x02 )
    isInitialized_ = isInitialized_ | 0x02;

  place_ph_cams();
  update_base_to_world();
  geometry_msgs::Pose currPose;
  currPose.position.x = baseToWorld_.transform.translation.x;
  currPose.position.y = baseToWorld_.transform.translation.y;
  currPose.position.z = baseToWorld_.transform.translation.z;
  currPose.orientation = baseToWorld_.transform.rotation;
  poseHist_.poses.push_back(currPose);
}

// ***************************************************************************
void scan_plan::timer_replan_cb(const ros::TimerEvent&)
{
  if( (isInitialized_ & 0x03) != 0x03 )
    return;

  ros::Time timeS = ros::Time::now();

  double exploredVol = octTree_->getNumLeafNodes() * pow(octTree_->getResolution(),3);
  double exploredVolRate = (exploredVol - expVol_) / timerReplanPeriod_;
  expVol_ = exploredVol;
  
  if(exploredVolRate >= minExpVolRate_)
    plan_pose_graph_mode();
  else
    plan_frontier_mode();

  update_base_to_world();

  ros::Duration timeE = ros::Time::now() - timeS;

  publish_path();

  std_msgs::Float64 computeTimeMsg;
  computeTimeMsg.data = timeE.toSec();
  compTimePub_.publish(computeTimeMsg);

  
  std::cout << "Replan Compute Time = " << computeTimeMsg.data << " sec" << std::endl;
}

// ***************************************************************************
void scan_plan::plan_frontier_mode()
{
  std::vector<std::vector<octomap::point3d>> frontierGroups = get_frontier_groups(octTree_);

  if (frontierGroups.size() < 1)
    return;

  int largestGroupIndx = 0;
  int largestGroupSize = frontierGroups[0].size();

  for (int i=1; i<frontierGroups.size(); i++)
    if (frontierGroups[i].size() > largestGroupSize)
    {
      largestGroupSize = frontierGroups[i].size();
      largestGroupIndx = i;
    }

  double distToFrontier = pow(baseToWorld_.transform.translation.x - frontierGroups[largestGroupIndx][0].x(), 2) +
                          pow(baseToWorld_.transform.translation.y - frontierGroups[largestGroupIndx][0].y(), 2) +
                          pow(baseToWorld_.transform.translation.z - frontierGroups[largestGroupIndx][0].z(), 2);
  distToFrontier = sqrt(distToFrontier);

  double rrtBnds[2][3];
  rrtBnds[0][0] = baseToWorld_.transform.translation.x - distToFrontier;
  rrtBnds[0][1] = baseToWorld_.transform.translation.y - distToFrontier;
  rrtBnds[0][2] = baseToWorld_.transform.translation.z - distToFrontier;

  rrtBnds[1][0] = baseToWorld_.transform.translation.x + distToFrontier;
  rrtBnds[1][1] = baseToWorld_.transform.translation.y + distToFrontier;
  rrtBnds[1][2] = baseToWorld_.transform.translation.z + distToFrontier;

  rrtTree_->set_bounds(rrtBnds[0], rrtBnds[1]);
  rrtTree_->update_oct_dist(octDist_);

  int goalLeaf = rrtTree_->build( Eigen::Vector3d(baseToWorld_.transform.translation.x, baseToWorld_.transform.translation.y, baseToWorld_.transform.translation.z),
                                  Eigen::Vector3d(frontierGroups[largestGroupIndx][0].x(), frontierGroups[largestGroupIndx][0].y(), frontierGroups[largestGroupIndx][0].z()) );

  path_ = rrtTree_->get_path(goalLeaf);
}

// ***************************************************************************
void scan_plan::plan_pose_graph_mode()
{
  double rrtBnds[2][3];
  get_rrt_bounds(rrtBnds);
  rrtTree_->set_bounds(rrtBnds[0], rrtBnds[1]);
  rrtTree_->update_oct_dist(octDist_);

  rrtTree_->build(Eigen::Vector3d(baseToWorld_.transform.translation.x, baseToWorld_.transform.translation.y, baseToWorld_.transform.translation.z));

  std::vector<int> idLeaves = rrtTree_->get_leaves();
  disp(idLeaves.size(), "Number of paths found");

  // get all paths ending at leaves and choose one with min cost
  double currRobYaw = quat_to_yaw(baseToWorld_.transform.rotation);
  double currExpHeight;
  double currExpYaw = exploration_direction(currExpHeight);
   
  std::cout << std::endl;
  disp(currExpYaw*180/3.14159, "EXPLORATION HEADING");
  disp(currExpHeight, "EXPLORATION HEIGHT");
  std::cout << std::endl;

  double minCst = 10000;
  for(int i=0; i<idLeaves.size(); i++)
  {
    Eigen::MatrixXd path = rrtTree_->get_path(idLeaves[i]);

    std::vector<geometry_msgs::TransformStamped> pathPoses;
    std::vector<ph_cam> phCamsPath = path_ph_cams(path, pathPoses);

    double pathCst = cGain_[0] * exp(-1*fov_dist(phCamsPath))
                   + cGain_[1] * heading_diff(currExpYaw, pathPoses)
                   + cGain_[2] * height_diff(currExpHeight, path)
                   + cGain_[3] * heading_diff(currRobYaw, pathPoses);

    if( minCst > pathCst && path_length(path) > (radRob_+0.1) )
    {
      minCst = pathCst;
      pathPoses_ = pathPoses;
      path_ = path;
    }
  }
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  delete octTree_;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octTree_ = dynamic_cast<octomap::OcTree*>(tree);

  init_dist_map();

  if( (isInitialized_ & 0x01) != 0x01 )
    isInitialized_ = isInitialized_ | 0x01;
}

// ***************************************************************************
void scan_plan::get_rrt_bounds(double (&rrtBnds)[2][3])
{
  //update_base_to_world();
  // min bounds
  double x_min = baseToWorld_.transform.translation.x + scanBnds_[0][0];
  rrtBnds[0][0] = std::max(x_min, x_min);
  rrtBnds[0][1] = baseToWorld_.transform.translation.y + scanBnds_[0][1];
  rrtBnds[0][2] = baseToWorld_.transform.translation.z + scanBnds_[0][2];

  // max bounds
  rrtBnds[1][0] = baseToWorld_.transform.translation.x + scanBnds_[1][0];
  rrtBnds[1][1] = baseToWorld_.transform.translation.y + scanBnds_[1][1];
  rrtBnds[1][2] = baseToWorld_.transform.translation.z + scanBnds_[1][2];
}

// ***************************************************************************
void scan_plan::place_ph_cams()
{
  if(!update_base_to_world()) 
    return;

  baseToWorld_ = tfBuffer_.lookupTransform(worldFrameId_, baseFrameId_, ros::Time(0));

  for(int i=0; i<nCams_; i++)
  {    
    // adding to phCamsWorld_
    phCamsWorld_.push_back(phCamsBase_[i]);
    phCamsWorld_.back().transform(baseToWorld_);
    // checking polytope for collision
    std::vector<geometry_msgs::Point> poly = phCamsWorld_.back().get_polytope();
    std::vector<bool> uCollVec(poly.size());
    int falseCount = 0; 
    for(int i=0; i<poly.size(); i++)
    {
      //std::cout << poly[i].x << ", " << poly[i].y << ", " << poly[i].z << std::endl;
      uCollVec[i] = rrt::u_coll_octomap(Eigen::Vector3d(poly[i].x,poly[i].y,poly[i].z), 0.1, octDist_);
      if(!uCollVec[i])
        falseCount++;
    }

    if(falseCount == poly.size())
      return;

    // reinitializing polytope with collVec
    phCamsWorld_.back().set_polytope(phCamsOpt_[i].get_polytope());
    phCamsWorld_.back().shrink(uCollVec);
    phCamsWorld_.back().transform(camToBase_[i]);
    phCamsWorld_.back().transform(baseToWorld_); 
  } 
}

// ***************************************************************************
void scan_plan::init_dist_map()
{ 
  //dynamicEDT3D examples

  // TODO:change to parameters

  double rrtBnds[2][3];
  get_rrt_bounds(rrtBnds);

  octomap::point3d min(rrtBnds[0][0], rrtBnds[0][1], rrtBnds[0][2]);
  octomap::point3d max(rrtBnds[1][0], rrtBnds[1][1], rrtBnds[1][2]);

  //double x,y,z;

  //octTree_->getMetricMin(x,y,z);
  //octomap::point3d min(x, y, z);

  //octTree_->getMetricMax(x,y,z);
  //octomap::point3d max(x, y, z);

  bool unknownAsOccupied = false;

  float maxDist = 5.0;

  delete octDist_;
  octDist_ = new DynamicEDTOctomap(maxDist, octTree_, min, max, unknownAsOccupied);

  octDist_->update(true);  //This computes the distance map
  std::cout << "Initialized dist map" << std::endl;
}

// ***************************************************************************
void scan_plan::test_script()
{
// 1. Grow tree, check the nodes, paths, distance of each node from the map
  rrtTree_->update_oct_dist(octDist_);
  rrtTree_->build(Eigen::Vector3d(0.1,0,1.1));

  std::cout << "Distance to (0.1,0,1.1) is " << octDist_->getDistance (octomap::point3d(0,0,0)) << std::endl;
  std::cout << "Distance to (2,0,1) is " << octDist_->getDistance (octomap::point3d(2,0,1)) << std::endl;

  //rrtTree_->u_coll_octomap(Eigen::Vector3d(0,0,0));
  
}

// ***************************************************************************
std::vector<ph_cam> scan_plan::path_ph_cams(Eigen::MatrixXd& path, std::vector<geometry_msgs::TransformStamped>& baseToWorldOut)
{
  baseToWorldOut.resize(path.rows());
  std::vector<ph_cam> phCamsPath(path.rows()*nCams_, phCamsBase_[0]);

  Eigen::Vector3d pos1;
  Eigen::Vector3d pos2;

  for(int i=0; i<path.rows()-1; i++)
  {
    pos1 = Eigen::Vector3d(path(i,0), path(i,1), path(i,2));
    pos2 = Eigen::Vector3d(path(i+1,0), path(i+1,1), path(i+1,2));
    baseToWorldOut[i] = transform_msg(pos1, pos2);

    for(int j=0; j<nCams_; j++)
    {
      phCamsPath[nCams_*i + j] = phCamsBase_[j];
      phCamsPath[nCams_*i + j].transform(baseToWorldOut[i]);
    }
  }

  baseToWorldOut[path.rows()-1] = transform_msg(pos1, pos2, false);
  for(int j=0; j<nCams_; j++)
  {
    phCamsPath[nCams_*(path.rows()-1) + j] = phCamsBase_[j];
    phCamsPath[nCams_*(path.rows()-1) + j].transform(baseToWorldOut[path.rows()-1]);
  }

  return phCamsPath;
}
// ***************************************************************************
double scan_plan::nearest(ph_cam phCamIn, std::vector<ph_cam>& phCamLst)
{
  if(phCamLst.size() == 0)
    return 0.0;

  double minDist = phCamLst[0].distance(phCamIn);

  for(int i=1; i<phCamLst.size(); i++)
  {
    double dist = phCamLst[i].distance(phCamIn);
    if( dist < minDist )
      minDist = dist;
  }

  return minDist;
}

// ***************************************************************************
double scan_plan::heading_diff(double currYaw, std::vector<geometry_msgs::TransformStamped>& pathPoses)
{
  const double discountFactor = 0.7;

  double yawMse = 0;
  for(int i=0; i<pathPoses.size(); i++)
  {
    double yawDiff = quat_to_yaw(pathPoses[i].transform.rotation) - currYaw;

    if(yawDiff > 3.14159)
      yawDiff = 2*3.14159 - yawDiff;
    if(yawDiff < -3.14159)
      yawDiff = 2*3.14159 + yawDiff;
    
    yawMse = yawMse + abs(yawDiff)*pow(discountFactor,i);
  }

  return yawMse; //pathPoses.size();
}

// ***************************************************************************
double scan_plan::height_diff(double currHeight, Eigen::MatrixXd& path)
{
  double absHeightErr = 0;
  for(int i=0; i<path.rows(); i++)
  {
    absHeightErr += abs( path(i,2) - currHeight );
  }

  return absHeightErr/path.rows();
}

// ***************************************************************************
double scan_plan::fov_dist(std::vector<ph_cam>& phCamsPath)
{
  double dist = 0;
  for(int i=0; i<phCamsPath.size(); i++)
   dist = dist + nearest(phCamsPath[i], phCamsWorld_);

  dist = dist / phCamsPath.size();
}

// ***************************************************************************
void scan_plan::path_cost(Eigen::MatrixXd& path)
{
  double pathLen = 0;
}

// ***************************************************************************
double scan_plan::quat_to_yaw(geometry_msgs::Quaternion quat)
{
  tf2::Quaternion tfRot;
  fromMsg(quat, tfRot);

  double roll, pitch, yaw;
  tf2::Matrix3x3(tfRot).getRPY(roll, pitch, yaw);

  return yaw;
}

// ***************************************************************************
geometry_msgs::Quaternion scan_plan::yaw_to_quat(double yaw)
{
  tf2::Quaternion rot;
  rot.setRPY( 0, 0, yaw );
  return tf2::toMsg(rot);
}

// ***************************************************************************
geometry_msgs::TransformStamped scan_plan::transform_msg(Eigen::Vector3d pos1, Eigen::Vector3d pos2, bool loc) 
{
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = worldFrameId_;
  transform.child_frame_id = baseFrameId_;

  if(loc)
  {
    transform.transform.translation.x = pos1(0);
    transform.transform.translation.y = pos1(1);
    transform.transform.translation.z = pos1(2);
  }
  else
  {
    transform.transform.translation.x = pos2(0);
    transform.transform.translation.y = pos2(1);
    transform.transform.translation.z = pos2(2);
  }

  Eigen::Vector3d pos = pos2 - pos1;

  transform.transform.rotation = yaw_to_quat(atan2(pos(1), pos(0)));

  return transform;
}

// ***************************************************************************
void scan_plan::publish_path()
{
  if(pathPoses_.size() < 2)
    return;

  // publish path
  nav_msgs::Path path;

  path.header.frame_id = worldFrameId_;
  path.header.stamp = ros::Time::now();
  path.poses.resize(pathPoses_.size());

  for(int i=0; i<pathPoses_.size(); i++)
  {
    path.poses[i].header.frame_id = path.header.frame_id;
    path.poses[i].header.stamp = path.header.stamp;

    path.poses[i].pose.position.x = pathPoses_[i].transform.translation.x;
    path.poses[i].pose.position.y = pathPoses_[i].transform.translation.y;
    path.poses[i].pose.position.z = pathPoses_[i].transform.translation.z;

    path.poses[i].pose.orientation = pathPoses_[i].transform.rotation;
  }
  pathPub_.publish(path);
}

// ***************************************************************************
void scan_plan::publish_lookahead()
{
  if(path_.size() < 2)
    return;

  //std::cout << "Interpolating" << std::endl;
  Eigen::MatrixXd path = interpolate(path_, 5);

  //std::cout << "Interpolated" << std::endl;
  update_base_to_world();
  Eigen::Vector3d robPos (baseToWorld_.transform.translation.x, 
                          baseToWorld_.transform.translation.y, 
                          baseToWorld_.transform.translation.z);

  //std::cout << path << std::endl;
  //std::cout << "Finding 1 m point" << std::endl;
  //std::cout << ( (path.rowwise() - robPos.transpose()).rowwise().squaredNorm() ).array()  << std::endl;
  
  disp(path.rows(), "Path Size");
  Eigen::VectorXd distVec = (path.rowwise() - robPos.transpose()).rowwise().squaredNorm(); // distance of each point from robot

  disp(distVec.rows(), "Dist Vector Size");
  //std::cout <<distVec << std::endl;

  int minDistIndx;
  distVec.minCoeff(&minDistIndx); // indx of closest point
  disp(minDistIndx, "Min Dist Indx");
  int indx; // indx of point 1m from min dist point
  ( distVec.bottomRows(path.rows()-minDistIndx).array() - pow(lookaheadDist_,2) ).cwiseAbs().minCoeff(&indx);

  indx = indx + minDistIndx;

  disp(indx, "1m Indx");

  std::cout << "Publishing" << indx << std::endl;
  Eigen::Vector3d pos(path(indx,0), path(indx,1), path(indx,2));
  // publish lookahead point
  geometry_msgs::PointStamped lookahead;
  lookahead.header.frame_id = worldFrameId_;
  lookahead.header.stamp = ros::Time::now();

  lookahead.point.x = pos(0);
  lookahead.point.y = pos(1);
  lookahead.point.z = pos(2);

  //lookaheadPub_.publish(lookahead);
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::interpolate(Eigen::MatrixXd& path, int res) // res: number of points per segment
{
  if(path.rows() < 2)
    return path;

  Eigen::MatrixXd pathOut(res*(path.rows()-1)+1, path.cols());

  double delTheta = 1/double(res);

  int count = 0;
  for(int i=0; i<(path.rows()-1); i++)
  {
    for(double theta=0; theta<1; theta+=delTheta)
    {
      pathOut.row(count) = (1-theta)*path.row(i) + theta*path.row(i+1);

      count++;
    }
  }

  pathOut.bottomRows(1) = path.bottomRows(1);
  return pathOut;
}

// ***************************************************************************
double scan_plan::exploration_direction(double& avgHeight)
{
  const int poseHistSize = poseHist_.poses.size();
  const int nLastPoses = nHistPoses_;

  if(poseHistSize < 2 || nLastPoses < 2)
    return 0.0;

  Eigen::Vector3d estDirVec(0,0,0);
  avgHeight = 0;

  int startIndx = std::max(0, poseHistSize - nLastPoses);

  for(int i=startIndx; i<(poseHistSize-1); i++)
  {
    estDirVec += Eigen::Vector3d (poseHist_.poses[i+1].position.x - poseHist_.poses[i].position.x,
                                  poseHist_.poses[i+1].position.y - poseHist_.poses[i].position.y,
                                  poseHist_.poses[i+1].position.z - poseHist_.poses[i].position.z) . normalized();

    avgHeight += poseHist_.poses[i].position.z;
  }
  avgHeight += poseHist_.poses[poseHistSize-1].position.z;
  avgHeight = avgHeight / double(poseHistSize);

  return std::atan2(estDirVec(1), estDirVec(0));
}

// ***************************************************************************
double scan_plan::path_length(Eigen::MatrixXd& path)
{
  if(path.rows() == 0)
    return 0;

  return double(path.rows()-1)*rrtDelDist_;
}

// ***************************************************************************
std::vector<std::vector<octomap::point3d>> scan_plan::get_frontier_groups(const octomap::OcTree *octree) 
{
  std::vector<std::vector<octomap::point3d>> frontier_groups;
  std::vector<octomap::point3d> frontier_points;
  octomap::OcTreeNode *n_cur_frontier;
  bool frontier_true;         // whether or not a frontier point
  bool belong_old;            //whether or not belong to old group
  double distance;
  double R1 = 0.4;            //group size
  double x_cur, y_cur, z_cur;

  double octo_reso = octTree_->getResolution();

  for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
  {
      frontier_true = false;
      unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

    if(!octree->isNodeOccupied(*n))
      {
       x_cur = n.getX();
       y_cur = n.getY();
       z_cur = n.getZ();

       if(z_cur < 0.4)    continue;
       if(z_cur > 0.4 + octo_reso)    continue;
       //if there are unknown around the cube, the cube is frontier
       for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
           for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
          {
              n_cur_frontier = octree->search(octomap::point3d(x_cur_buf, y_cur_buf, z_cur));
              if(!n_cur_frontier)
              {
                  frontier_true = true;
                  continue;            
              }

          }
          if(frontier_true)// && num_free >5 )
          {
              // divide frontier points into groups
              if(frontier_groups.size() < 1)
              {
                  frontier_points.resize(1);
                  frontier_points[0] = octomap::point3d(x_cur,y_cur,z_cur);
                  frontier_groups.push_back(frontier_points);
                  frontier_points.clear();
              }
              else
              {
                  bool belong_old = false;            

                  for(std::vector<std::vector<octomap::point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                          distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)) ;
                          if(distance < R1){
                             frontier_groups[u].push_back(octomap::point3d(x_cur, y_cur, z_cur));
                             belong_old = true;
                             break;
                          }
                  }
                  if(!belong_old){
                             frontier_points.resize(1);
                             frontier_points[0] = octomap::point3d(x_cur, y_cur, z_cur);
                             frontier_groups.push_back(frontier_points);
                             frontier_points.clear();
                  }                              
              }
          } 
      }
        
  }
  return frontier_groups;
}

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete octTree_;
  delete octDist_;
  delete rrtTree_;
}
