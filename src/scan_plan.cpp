#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  nh_ = nh;
 
  tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
  wait_for_params(nh_);

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

  //std::vector<bool> uColl(13, 0.0);
  //uColl[10] = true;
  //uColl[0] = true;
  //phCam.shrink(uColl);
  //phCam.print_polytope();

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);
  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);

  rrtTree_ = new rrt(rrtNNodes_, scanBnds_[0], scanBnds_[1], rrtRadNear_, rrtDelDist_, radRob_, rrtFailItr_, octDist_);
  lastPlanTime_ = ros::Time::now();
  lastPhCamTime_ = ros::Time::now();

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

  std::cout << camFrameId[0] << ", " << baseFrameId_ << std::endl;

  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());

  for(int i=0; i<nCams_; i++)
  {
    while( !tfBuffer_.canTransform(baseFrameId_, camFrameId[i], ros::Time(0)) );
      camToBase_.push_back( tfBuffer_.lookupTransform(baseFrameId_, camFrameId[i], ros::Time(0)) );
  }
  
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
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{

  ros::Time timeS = ros::Time::now();

  delete octTree_;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octTree_ = dynamic_cast<octomap::OcTree*>(tree);

  //octTree_ = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octmpMsg);
  //octomap_msgs::readTree<octomap::OcTree> (octTree_, octmpMsg);

  std::cout<<"read in tree, "<<octTree_->getNumLeafNodes()<<" leaves "<<std::endl;

  if( (isInitialized_ & 0x01) != 0x01 )
  {
    init_dist_map();
    place_ph_cams();
    isInitialized_ = isInitialized_ | 0x01;
  }

  init_dist_map();
  if( (ros::Time::now() - lastPhCamTime_).toSec() > timeIntPhCam_ )
  {
    place_ph_cams();

    phCamsWorld_.back().print_polytope();
    int indx = phCamsWorld_.size()-2;
    std::cout << std::endl;
    std::cout << "GJK DISTANCE: " << phCamsWorld_.back().distance(phCamsWorld_[indx]) << std::endl; 
    std::cout << std::endl;

    lastPhCamTime_ = ros::Time::now();
  }

  if( (ros::Time::now() - lastPlanTime_).toSec() > 3.0 )
  {
    test_script();
    rrtTree_->plot_tree();
    std::vector<int> idLeaves = rrtTree_->get_leaves();
    //disp(idLeaves.size(), "Number of Leaves");

    Eigen::MatrixXd minCstpath;
    double minFovOv = 1000;
    for(int i=0; i<idLeaves.size(); i++)
    {
      Eigen::MatrixXd path = rrtTree_->get_path(idLeaves[i]);

      std::vector<geometry_msgs::TransformStamped> phCamsPoses;
      std::vector<ph_cam> phCamsPath = path_ph_cams(path, phCamsPoses);

      //std::cout << path.rows() << "==" << phCamsPath.size();
     // for(int i=0; i<phCamsPath.size(); i++)
      //  phCamsPath[i].print_polytope();
 
      double fovOv = fov_overlap_cost(phCamsPath);
      double yawCst = heading_cost(phCamsPoses);
      //disp(path, "Path");
      //disp(fovOv, "FOV Overlap Cost: ");

      if(minFovOv > fovOv)
      {
        minFovOv = yawCst;
        minCstpath = path;
      }
    }
    lastPlanTime_ = ros::Time::now();

    rrtTree_->plot_path(minCstpath);
    getchar();
  }

  ros::Duration timeE = ros::Time::now() - timeS;
  std::cout << "Time Elapsed = " << timeE.toSec() << " sec" << std::endl;
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
      std::cout << poly[i].x << ", " << poly[i].y << ", " << poly[i].z << std::endl;
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

  double x,y,z;

  octTree_->getMetricMin(x,y,z);
  octomap::point3d min(x, y, z);

  octTree_->getMetricMax(x,y,z);
  octomap::point3d max(x, y, z);

  bool unknownAsOccupied = false;

  float maxDist = 1.0;

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
  baseToWorldOut.resize(path.rows()*nCams_);
  std::vector<ph_cam> phCamsPath(path.rows()*nCams_, phCamsBase_[0]);

  Eigen::Vector3d pos1;
  Eigen::Vector3d pos2;

  for(int i=0; i<path.rows()-1; i++)
    for(int j=0; j<nCams_; j++)
    {
      pos1 = Eigen::Vector3d(path(i,0), path(i,1), path(i,2));
      pos2 = Eigen::Vector3d(path(i+1,0), path(i+1,1), path(i+1,2));
      baseToWorldOut.push_back(transform_msg(pos1, pos2));

      phCamsPath[nCams_*i + j] = phCamsBase_[j];
      phCamsPath[nCams_*i + j].transform(baseToWorldOut.back());
    }

  for(int j=0; j<nCams_; j++)
  {
    baseToWorldOut.push_back(transform_msg(pos1, pos2, false));
    phCamsPath[nCams_*(path.rows()-1) + j] = phCamsBase_[j];
    phCamsPath[nCams_*(path.rows()-1) + j].transform(baseToWorldOut.back());
  }

  return phCamsPath;
}
// ***************************************************************************
double scan_plan::nearest(ph_cam phCamIn, std::vector<ph_cam>& phCamLst)
{
  if(phCamLst.size() == 0)
    return -1.0;

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
double scan_plan::heading_cost(std::vector<geometry_msgs::TransformStamped>& pathPoses)
{
  double currYaw = quat_to_yaw(baseToWorld_.transform.rotation);

  double yawMse = 0;
  for(int i=0; i<pathPoses.size(); i++)
  {
    double yawDiff = quat_to_yaw(pathPoses[i].transform.rotation) - currYaw;

    if(yawDiff > 3.14159)
      yawDiff = 2*3.14159 - yawDiff;
    if(yawDiff < 3.14159)
      yawDiff = 2*3.14159 + yawDiff;
    
    yawMse = yawMse + pow(yawDiff,2);
  }

  return yawMse/pathPoses.size();
}

// ***************************************************************************
double scan_plan::fov_overlap_cost(std::vector<ph_cam>& phCamsPath)
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

 //std::cout << "Pos1: " << pos1.transpose() << std::endl;
 //std::cout << "Pos2: " << pos2.transpose() << std::endl;
 //std::cout << "Yaw: " << atan2(pos(1), pos(0))*180/3.14159 << std::endl;
  transform.transform.rotation = yaw_to_quat(atan2(pos(1), pos(0)));

  return transform;
}

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete octTree_;
  delete octDist_;
  delete rrtTree_;
}
