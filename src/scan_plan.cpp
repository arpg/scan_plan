#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  test_script();

  isInitialized_ = 0x00;
  nh_ = nh;
 
  tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
  setup_pose();
  setup_sensors();
  setup_rrt();
  setup_graph();
  setup_timers();
  setup_scan_plan();

  //TODO: graph u_coll lambda is too large since the nearRad is 10.0*rrtDelDist_

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);

  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);
  lookaheadPub_ = nh->advertise<geometry_msgs::PointStamped>("lookahead_out", 10);
  compTimePub_ = nh->advertise<std_msgs::Float64>("compute_time_out", 10);
  vizPub_ = nh->advertise<visualization_msgs::MarkerArray>("viz_out", 10);
  frontiersPub_ = nh->advertise<geometry_msgs::PoseArray>("frontiers_out", 10);

  pathPoses_.resize(0);
  path_.resize(0,0);

  ROS_INFO("%s: Waiting for the input map ...", nh->getNamespace().c_str());
  while( (isInitialized_ & 0x01) != 0x01 )
	  ros::spinOnce();
}

// ***************************************************************************
void scan_plan::setup_pose()
{
  // prereqs tfBuffer should be setup
  ROS_INFO("%s: Waiting for base and world frame id params ...", nh_->getNamespace().c_str());
  while(!nh->getParam("base_frame_id", baseFrameId_));
  while(!nh->getParam("world_frame_id", worldFrameId_));

  ROS_INFO("%s: Waiting for base to world transform ...", nh_->getNamespace().c_str());
  while( !tfBuffer_.canTransform(worldFrameId_, baseFrameId_, ros::Time(0)) );
  update_base_to_world();

  ROS_INFO("%s: Base to world transform alive", nh_->getNamespace().c_str());
}
// ***************************************************************************
void scan_plan::setup_sensors()
{
  // prereqs setup_pose
  ROS_INFO("%s: Waiting for sensor params ...", nh_->getNamespace().c_str());
  std::vector<double> fovs, res, ranges;
  std::vector<std::string> sensorFrameIds;
  
  while(!nh->getParam("sensor_fovs_horz_vert_degrees", fovs));
  while(!nh->getParam("sensor_res_horz_vert_pts", res));
  while(!nh->getParam("sensor_ranges", ranges));
  while(!nh->getParam("sensor_frame_ids", sensorFrameIds));

  ROS_INFO("%s: Waiting for sensors to base transforms ...", nh_->getNamespace().c_str());
  std::vector<geometry_msgs::TransformStamped> sensorsToBase;
  for(int i=0; i<sensorFrameIds.size(); i++)
  {
    while( !tfBuffer_.canTransform(baseFrameId_, sensorFrameIds[i], ros::Time(0)) );
    sensorsToBase.push_back( tfBuffer_.lookupTransform(baseFrameId_, sensorFrameIds[i], ros::Time(0)) );  
  }

  ROS_INFO("%s: Setting up mapping sensors ...", nh_->getNamespace().c_str());
  for(int i=0; i<sensorFrameIds.size(); i++)
    mSensors_.push_back( mapping_sensor(fovs[i*2], fovs[(i*2)+1], res[i*2], res[(i*2)+1], ranges[i], sensorsToBase[i]) );
  
}
// ***************************************************************************
void scan_plan::setup_rrt()
{
  ROS_INFO("%s: Waiting for tree params ...", nh_->getNamespace().c_str());

  std::vector<double> minBnds, maxBnds;
  while(!nh->getParam("min_bounds_local", minBnds)); // [x_min, y_min, z_min]
  while(!nh->getParam("max_bounds_local", maxBnds)); // [x_max, y_max, z_max]

  double rrtRadNear, rrtDelDist, rrtFailItr, rrtNNodes;

  while(!nh->getParam("n_rrt_nodes", rrtNNodes));
  while(!nh->getParam("near_radius_rrt", rrtRadNear));
  while(!nh->getParam("delta_distance_rrt", rrtDelDst));
  while(!nh->getParam("n_fail_iterations_rrt", rrtFailItr));
  while(!nh->getParam("robot_radius", radRob_));

  ROS_INFO("%s: Setting up local tree ...", nh_->getNamespace().c_str());
  rrtTree_ = new rrt(rrtNNodes, minBnds, maxBnds, rrtRadNear, rrtDelDist, radRob_, rrtFailItr);  
}

// ***************************************************************************
void scan_plan::setup_graph()
{
  // prereqs setup_pose, setup_sensors, setup_rrt

  ROS_INFO("%s: Waiting for graph params ...", nh_->getNamespace().c_str());
  double graphRadNear, minVolGain;
  std::vector<double> homePos;
 
  while(!nh->getParam("near_radius_graph", graphRadNear));
  while(!nh->getParam("min_vol_gain_cubic_m", minVolGain)); // used for local/global switching and removing frontiers
  while(!nh->getParam("home_position", homePos)); 
  
  ROS_INFO("%s: Setting up graph ...", nh_->getNamespace().c_str());
  //Eigen::Vector3d currPos(baseToWorld_.transform.translation.x, 
  //                        baseToWorld_.transform.translation.y, 
  //                        baseToWorld_.transform.translation.z);

  graph_ = new graph(Eigen::Vector3d(homePos[0],homePos[1],homePos[2]), graphRadNear, radRob_, mSensors_, minVolGain, worldFrameId_);
}

// ***************************************************************************
void scan_plan::setup_timers()
{
  ROS_INFO("%s: Waiting for timer params ...", nh_->getNamespace().c_str());
  double timeIntPhCam, timeIntReplan;
  while(!nh->getParam("time_interval_pose_graph", timeIntPhCam));
  while(!nh->getParam("time_interval_replan", timeIntReplan));

  ROS_INFO("%s: Creating timers ...", nh_->getNamespace().c_str());
  timerPhCam_ = nh->createTimer(ros::Duration(timeIntPhCam), &scan_plan::timer_ph_cam_cb, this);
  timerReplan_ = nh->createTimer(ros::Duration(timeIntReplan), &scan_plan::timer_replan_cb, this);
}

// ***************************************************************************
void scan_plan::setup_octomap()
{
  ROS_INFO("%s: Waiting for octomap params ...", nh_->getNamespace().c_str());
  while(!nh->getParam("esdf_max_dist", maxDistEsdf_));
  while(!nh->getParam("esdf_unknown_as_occupied", esdfUnknownAsOccupied_));

  ROS_INFO("%s: Setting up octomap manager ...", nh_->getNamespace().c_str());
  
}

// ***************************************************************************
void scan_plan::setup_scan_plan()
{
  ROS_INFO("%s: Waiting for scan_plan params ...", nh_->getNamespace().c_str());

  while(!nh->getParam("c_gain", cGain_));
  while(!nh->getParam("lookahead_path_len", lookaheadDist_));
  while(!nh->getParam("n_hist_poses_exploration_dir", nHistPoses_));
  while(!nh->getParam("path_res_for_lookahead_npts_per_seg", pathResLookaheadPt_));
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
Eigen::Vector3d scan_plan::transform_to_eigen_pos(const geometry_msgs::TransformStamped& transformIn)
{
  return ( Eigen::Vector3d ( transformIn.transform.translation.x, 
                             transformIn.transform.translation.y, 
                             transformIn.transform.translation.z ) ); 
}

// ***************************************************************************
void scan_plan::timer_replan_cb(const ros::TimerEvent&)
{
  if( (isInitialized_ & 0x03) != 0x03 )
    return;

  ros::Time timeS = ros::Time::now();
  
  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );

  //double rrtBnds[2][3];
  //get_rrt_bounds(rrtBnds);
  rrtTree_->set_bounds( geofence_saturation(localBndsMin_+robPos), geofence_saturation(localBndsMax_+robPos) );
  rrtTree_->update_octomap_man(octomapMan_);
  //rrtTree_->update_oct_dist(octDist_);
  graph_->update_octomap_man(octomapMan_);

  rrtTree_->build(robPos);

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

  int idPathLeaf = -1;

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

    if( minCst > pathCst && min_path_len(path) ) // minimum path length condition
    {
      minCst = pathCst;
      pathPoses_ = pathPoses;
      path_ = path;
      idPathLeaf = idLeaves[i];
    }
  }

  std::cout << "Adding paths to the graph" << std::endl;
  // assuming if a path is found that can be followed i.e, lowest cost and having atleast minimum path length
  if( idPathLeaf > -1 ) 
  {
    bool success = add_paths_to_graph(rrtTree_, idLeaves, idPathLeaf, graph_);
    if(!success)
      ROS_WARN("%s: Graph connection unsuccessful", nh_->getNamespace().c_str());
  }
  //TODO: Generate graph diconnect warning if no path is successfully added to the graph
  // Only add node to the graph if there is no existing node closer than radRob in graph lib, return success in that case since the graph is likely to be connected fine
  // update older node if within robot radius with updated terrain and collision information, the latter may mean taking nodes out of the graph

  //TODO: Consider including a check, update path if end-of-path is reached, and increase the replan time

  graph_->update_frontiers_vol_gain();
   
  ros::Duration timeE = ros::Time::now() - timeS;

  std::cout << "Publishing lookahead path" << std::endl;
  publish_path();

  std::cout << "Publishing frontiers" << std::endl; 
  graph_->publish_frontiers(frontiersPub_); 

  std::cout << "Publishing visualizations" << std::endl;
  graph_->publish_viz(vizPub_);
  //rrtTree_->publish_viz(vizPub_,worldFrameId_,idLeaves);

  std_msgs::Float64 computeTimeMsg;
  computeTimeMsg.data = timeE.toSec();
  compTimePub_.publish(computeTimeMsg);
  
  std::cout << "Replan Compute Time = " << computeTimeMsg.data << " sec" << std::endl;
}

// ***************************************************************************
bool scan_plan::add_paths_to_graph(rrt* tree, std::vector<int>& idLeaves, int idLookaheadLeaf, graph* gph) 
{
  //TODO: Add paths with best pose history distances instead of randomly choosing

  const int nGraphPaths = 5; // assuming 10 paths in total are to be added to the graph
  bool success = false;

  Eigen::MatrixXd path;

  std::cout << "Adding non-lookahead paths" << std::endl;
  // all other paths

  int nCurrGraphPaths = 0;
  while((nCurrGraphPaths < (nGraphPaths-1)) && (nCurrGraphPaths < idLeaves.size()))
  {
    if(idLeaves[nCurrGraphPaths] == idLookaheadLeaf)
    {
      nCurrGraphPaths++;
      continue;
    }

    path = tree->get_path(idLeaves[nCurrGraphPaths]);
    if( !min_path_len(path) )
    {
      nCurrGraphPaths++;
      continue;
    }

    std::cout << "HERE" << std::endl;
    if( add_path_to_graph(path, gph, true) )
      success = true;
    std::cout << "HERE2" << std::endl;
    nCurrGraphPaths++;
  }

  std::cout << "Adding lookahead path" << std::endl;
  // lookahead path
  // assuming already checked for minimum path length
  path = tree->get_path(idLookaheadLeaf);
  if( add_path_to_graph(path, gph, false) )
    success = true;

  return success;
}

// ***************************************************************************
bool scan_plan::add_path_to_graph(Eigen::MatrixXd& path, graph*, bool containFrontier)
{
  bool success= false;

  gvert vert;
  for(int i=0; i<path.rows(); i++)
  {
    vert.pos = path.row(i);
    vert.commSig = 0;
    
    if( containFrontier && i==(path.rows()-1) )
      vert.isFrontier = true;
    else
      vert.isFrontier = false;

    vert.terrain = gvert::UNKNOWN;

    std::cout << "Adding vertex" << std::endl;
    success = graph_->add_vertex(vert);
  }

  return success;
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  //delete octTree_;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octTree_ = dynamic_cast<octomap::OcTree*>(tree);

  octomapMan_->update_octree(octTree_);
  octomapMan_->update_esdf();

  init_dist_map();

  if( (isInitialized_ & 0x01) != 0x01 )
    isInitialized_ = isInitialized_ | 0x01;
}

// ***************************************************************************
Eigen::Vector3d scan_plan::geofence_saturation(const Eigen::Vector3d& posIn)
{
  Eigen::Vector3d posOut = posIn;

  if(posOut(0) < geoFenceMin(0))
    pos(0) = geoFenceMin(0);
  if(posOut(0) > geoFenceMax(0))
    pos(0) = geoFenceMax(0);

  if(posOut(1) < geoFenceMin(1))
    pos(1) = geoFenceMin(1);
  if(posOut(1) > geoFenceMax(1))
    pos(1) = geoFenceMax(1);

  if(posOut(2) < geoFenceMin(2))
    pos(2) = geoFenceMin(2);
  if(posOut(2) > geoFenceMax(2))
    pos(2) = geoFenceMax(2);

  return posOut;
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

  bool unknownAsOccupied = esdfUnknownAsOccupied_; // true

  float maxDist = maxDistEsdf_; // 5.0

  delete octDist_;
  octDist_ = new DynamicEDTOctomap(maxDist, octTree_, min, max, unknownAsOccupied);

  octDist_->update(true);  //This computes the distance map
  std::cout << "Initialized dist map" << std::endl;
}

// ***************************************************************************
void scan_plan::test_script()
{
 // graph gph(Eigen::Vector3d(0.6,0.5,12), 0.5);

  return;

// 1. Grow tree, check the nodes, paths, distance of each node from the map
  rrtTree_->update_oct_dist(octDist_);
  rrtTree_->build(Eigen::Vector3d(0.1,0,1.1));

  std::cout << "Distance to (0.1,0,1.1) is " << octDist_->getDistance (octomap::point3d(0,0,0)) << std::endl;
  std::cout << "Distance to (2,0,1) is " << octDist_->getDistance (octomap::point3d(2,0,1)) << std::endl;

  //rrtTree_->u_coll_octomap(Eigen::Vector3d(0,0,0));
  
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
bool scan_plan::min_path_len(Eigen::MatrixXd& path)
{
  double pathLength = path_length(path);
  return (pathLength > (radRob_+0.1)) && (pathLength > (rrtDelDist_+0.1));
}

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete octTree_;
  delete octDist_;
  delete rrtTree_;
  delete graph_;
  delete mSensors_;
}
