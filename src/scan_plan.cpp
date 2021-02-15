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
  setup_octomap();
  setup_path();
  setup_rrt();
  setup_graph();
  setup_timers();
  setup_scan_plan();

  //TODO: graph u_coll lambda is too large since the nearRad is 10.0*rrtDelDist_

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);
  poseHistSub_ = nh->subscribe("pose_hist_in", 1, &scan_plan::pose_hist_cb, this);
  goalSub_ = nh->subscribe("goal_in", 1, &scan_plan::goal_cb, this);

  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);
  compTimePub_ = nh->advertise<std_msgs::Float64>("compute_time_out", 10);
  vizPub_ = nh->advertise<visualization_msgs::MarkerArray>("viz_out", 10);
  frontiersPub_ = nh->advertise<geometry_msgs::PoseArray>("frontiers_out", 10);

  ROS_INFO("%s: Waiting for the input map ...", nh->getNamespace().c_str());
  //while( (isInitialized_ & 0x01) != 0x01 )
	//  ros::spinOnce();
  ROS_INFO("%s: Exiting constructor ...", nh->getNamespace().c_str());
}

// ***************************************************************************
void scan_plan::setup_pose()
{
  // prereqs tfBuffer should be setup
  ROS_INFO("%s: Waiting for base and world frame id params ...", nh_->getNamespace().c_str());
  while(!nh_->getParam("base_frame_id", baseFrameId_));
  while(!nh_->getParam("world_frame_id", worldFrameId_));

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
  
  while(!nh_->getParam("sensor_fovs_horz_vert_degrees", fovs)); // input in degress, converted to radians
  while(!nh_->getParam("sensor_res_horz_vert_pts", res));
  while(!nh_->getParam("sensor_ranges", ranges));
  while(!nh_->getParam("sensor_frame_ids", sensorFrameIds));

  ROS_INFO("%s: Waiting for sensors to base transforms ...", nh_->getNamespace().c_str());
  std::vector<geometry_msgs::TransformStamped> sensorsToBase;
  for(int i=0; i<sensorFrameIds.size(); i++)
  {
    while( !tfBuffer_.canTransform(baseFrameId_, sensorFrameIds[i], ros::Time(0)) );
    sensorsToBase.push_back( tfBuffer_.lookupTransform(baseFrameId_, sensorFrameIds[i], ros::Time(0)) );  
  }

  ROS_INFO("%s: Setting up mapping sensors ...", nh_->getNamespace().c_str());
  for(int i=0; i<sensorFrameIds.size(); i++)
    mapSensors_.push_back( mapping_sensor(fovs[i*2]*(pi_/180), fovs[(i*2)+1]*(pi_/180), res[i*2], res[(i*2)+1], ranges[i], sensorsToBase[i]) );
  
}
// ***************************************************************************
void scan_plan::setup_rrt()
{
  // prereqs setup_octomap

  ROS_INFO("%s: Waiting for tree params ...", nh_->getNamespace().c_str());

  std::vector<double> minBnds, maxBnds;
  while(!nh_->getParam("min_bounds_local", minBnds)); // [x_min, y_min, z_min]
  while(!nh_->getParam("max_bounds_local", maxBnds)); // [x_max, y_max, z_max]

  localBndsMin_(0) = minBnds[0];
  localBndsMin_(1) = minBnds[1];
  localBndsMin_(2) = minBnds[2];

  localBndsMax_(0) = maxBnds[0];
  localBndsMax_(1) = maxBnds[1];
  localBndsMax_(2) = maxBnds[2];

  double rrtRadNear, rrtDelDist, rrtFailItr, rrtNNodes;

  while(!nh_->getParam("n_rrt_nodes", rrtNNodes));
  while(!nh_->getParam("near_radius_rrt", rrtRadNear));
  while(!nh_->getParam("delta_distance_rrt", rrtDelDist));
  while(!nh_->getParam("n_fail_iterations_rrt", rrtFailItr));

  ROS_INFO("%s: Setting up local tree ...", nh_->getNamespace().c_str());
  rrtTree_ = new rrt(rrtNNodes, minBnds, maxBnds, rrtRadNear, rrtDelDist, radRob_, rrtFailItr, octMan_);  
}

// ***************************************************************************
void scan_plan::setup_graph()
{
  // prereqs setup_pose, setup_octomap

  ROS_INFO("%s: Waiting for graph params ...", nh_->getNamespace().c_str());
  double graphRadNear, minVolGain, graphRadNearest;
  std::vector<double> homePos;
 
  while(!nh_->getParam("near_radius_graph", graphRadNear));
  while(!nh_->getParam("nearest_radius_graph", graphRadNearest));
  while(!nh_->getParam("min_vol_gain_cubic_m", minVolGain)); // used for local/global switching and removing frontiers
  while(!nh_->getParam("home_position", homePos)); // must be collision-free
  
  ROS_INFO("%s: Setting up graph ...", nh_->getNamespace().c_str());
  homePos_(0) = homePos[0];
  homePos_(1) = homePos[1];
  homePos_(2) = homePos[2];

  graph_ = new graph(homePos_, graphRadNear, graphRadNearest, radRob_, minVolGain, worldFrameId_, octMan_);
}

// ***************************************************************************
void scan_plan::setup_timers()
{
  ROS_INFO("%s: Waiting for timer params ...", nh_->getNamespace().c_str());

  while(!nh_->getParam("time_interval_replan", timeIntReplan_));

  ROS_INFO("%s: Creating timers ...", nh_->getNamespace().c_str());
  timerReplan_ = nh_->createTimer(ros::Duration(timeIntReplan_), &scan_plan::timer_replan_cb, this);
}

// ***************************************************************************
void scan_plan::setup_octomap()
{
  // prereqs setup_sensors
  ROS_INFO("%s: Waiting for octomap params ...", nh_->getNamespace().c_str());

  std::string vehicleType;
  double maxGroundRoughness, maxGroundStep, maxDistEsdf, groundPlaneSearchDist;
  bool esdfUnknownAsOccupied;

  while(!nh_->getParam("esdf_max_dist", maxDistEsdf));
  while(!nh_->getParam("esdf_unknown_as_occupied", esdfUnknownAsOccupied));
  while(!nh_->getParam("vehicle_type", vehicleType)); // "air", "ground"
  while(!nh_->getParam("robot_radius", radRob_));
  while(!nh_->getParam("max_ground_roughness", maxGroundRoughness)); // [0, 180], angle from postive z
  while(!nh_->getParam("max_ground_step", maxGroundStep)); // [epsilon,inf]
  while(!nh_->getParam("ground_plane_search_distance", groundPlaneSearchDist));

  ROS_INFO("%s: Setting up octomap manager ...", nh_->getNamespace().c_str());
  octMan_ = new octomap_man(maxDistEsdf, esdfUnknownAsOccupied, vehicleType, radRob_, maxGroundRoughness*(pi_/180), maxGroundStep, groundPlaneSearchDist, mapSensors_);
}

// ***************************************************************************
void scan_plan::setup_path()
{
  // prereqs setup_octomap
  ROS_INFO("%s: Waiting for path manager params ...", nh_->getNamespace().c_str());

  double minPathLen;
  while(!nh_->getParam("admissible_min_path_length", minPathLen));

  ROS_INFO("%s: Setting up path manager ...", nh_->getNamespace().c_str());
  pathMan_ = new path_man(minPathLen, octMan_);
}

// ***************************************************************************
void scan_plan::setup_scan_plan()
{
  // pre-reqs setup_graph
  ROS_INFO("%s: Waiting for scan_plan params ...", nh_->getNamespace().c_str());

  std::vector<double> geoFenceMin, geoFenceMax;
  while(!nh_->getParam("c_gain", cGain_));
  while(!nh_->getParam("min_bnds_geofence", geoFenceMin));
  while(!nh_->getParam("max_bnds_geofence", geoFenceMax)); // in world frame
  while(!nh_->getParam("n_hist_pts_for_exploration_dir", nHistPosesExpDir_));

  geoFenceMin_(0) = geoFenceMin[0];
  geoFenceMin_(1) = geoFenceMin[1];
  geoFenceMin_(2) = geoFenceMin[2];

  geoFenceMax_(0) = geoFenceMax[0];
  geoFenceMax_(1) = geoFenceMax[1];
  geoFenceMax_(2) = geoFenceMax[2];
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

  std::vector<int> idLeaves;
  int idPathLeaf;
  Eigen::MatrixXd minCstPath = generate_local_exp_path(idLeaves, idPathLeaf);

  if( minCstPath.rows() < 2 ) // if no admissible path is found
    timerReplan_.setPeriod(ros::Duration(1.0), false); // compute path at a faster rate
  else
  {
    std::cout << "Adding paths to the graph" << std::endl;
    timerReplan_.setPeriod(ros::Duration(timeIntReplan_), false); // don't reset timer
    bool success = add_paths_to_graph(rrtTree_, idLeaves, idPathLeaf, graph_);
    if(!success)
      ROS_WARN("%s: Graph connection unsuccessful", nh_->getNamespace().c_str());
  }

  //TODO: Generate graph diconnect warning if no path is successfully added to the graph
  // Only add node to the graph if there is no existing node closer than radRob in graph lib, return success in that case since the graph is likely to be connected fine
  // update older node if within robot radius with updated terrain and collision information, the latter may mean taking nodes out of the graph
  // consider immidiate replan if a path is not found by-passing timer

  //TODO: Consider including a check, update path if end-of-path is reached, and increase the replan time

  ///graph_->update_frontiers_vol_gain();
   
  ros::Duration timeE = ros::Time::now() - timeS;

  std::cout << "Publishing lookahead path" << std::endl;
  path_man::publish_path(minCstPath_, worldFrameId_, pathPub_);

  std::cout << "Publishing frontiers" << std::endl; 
  graph_->publish_frontiers(frontiersPub_); 

  std::cout << "Publishing visualizations" << std::endl;
  graph_->publish_viz(vizPub_);
  rrtTree_->publish_viz(vizPub_,worldFrameId_,idLeaves);

  std_msgs::Float64 computeTimeMsg;
  computeTimeMsg.data = timeE.toSec();
  compTimePub_.publish(computeTimeMsg);
  
  std::cout << "Replan Compute Time = " << computeTimeMsg.data << " sec" << std::endl;
}
// ***************************************************************************
scan_plan::MODE scan_plan::next_mode()
{
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_from_graph(const Eigen::Vector3d& toPos, VertexDescriptor& fromVertex)
{
  // returns a path and the graph vertex start point
  return ( plan_to_graph(toPos, fromVertex) ).rowwise().reverse(); // plan from the goal point to graph, then flip the path 
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_to_graph(const Eigen::Vector3d& fromPos, VertexDescriptor& toVertex)
{
  // returns a path and the graph vertex end point
 
  std::vector<VertexDescriptor> vertices = graph_->find_vertices_inside_box(localBndsMin_+fromPos, localBndsMax_+fromPos);
  if(vertices.size() < 1)
  {
    ROS_WARN("%s: Not enough nearby graph vertices to connect", nh_->getNamespace().c_str());
    return Eigen::MatrixXd(0,0);
  }

  std::cout << "Connecting to graph via a straight line" << std::endl;
  for(int i=0; i<vertices.size(); i++)
  {
    toVertex = vertices[i];
    Eigen::Vector3d vertexPos = graph_->get_pos(toVertex);

    if( octMan_->u_coll( fromPos, vertexPos) )
      continue;

    Eigen::MatrixXd path(2,3);
    path.row(0) = fromPos.transpose();
    path.row(1) = vertexPos.transpose();
    return path;
  }

  std::cout << "Connecting to graph via an rrt path" << std::endl;
  for(int i=0; i<vertices.size(); i++)
  {
    toVertex = vertices[i];
    Eigen::Vector3d vertexPos = graph_->get_pos(toVertex);

    int leafId = rrtTree_->build(fromPos, vertexPos); // using goal-bias rrt to find a path 

    if( leafId < 1 ) // if root node is returned / if a path is not found
      continue;

    Eigen::MatrixXd path = rrtTree_->get_path(leafId);
    return path;
  }

  std::cout << "Giving up" << std::endl;
  return Eigen::MatrixXd(0,0);
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::generate_local_exp_path(std::vector<int>& idLeaves, int& idPathLeaf) 
{
  // returns rrt lookahead path, ids of all leaves and of the lookahead path leaf
  // get_path function in rrt can be used to get paths for a leaf id

  Eigen::MatrixXd minCstPath(0,0);

  update_base_to_world();

  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );

  rrtTree_->set_bounds( geofence_saturation(localBndsMin_+robPos), geofence_saturation(localBndsMax_+robPos) );
  rrtTree_->build(robPos);

  idLeaves = rrtTree_->get_leaves();
  disp(idLeaves.size(), "Number of paths found");

  // get all paths ending at leaves and choose one with min cost
  double currRobYaw = quat_to_yaw(baseToWorld_.transform.rotation);
  std::pair<double, double> currExpYawHeight = path_man::mean_heading_height(posHist_.topRows(posHistSize_), nHistPosesExpDir_); ////////

  idPathLeaf = -1;

  double minCst = 100000;
  
  for(int i=0; i<idLeaves.size(); i++)
  {
    Eigen::MatrixXd path = rrtTree_->get_path(idLeaves[i]);

    std::pair<double, double> pathYawHeightErr = path_man::mean_heading_height_err(std::get<0>(currExpYawHeight), std::get<1>(currExpYawHeight), path);
    double pathCst = cGain_[0] * exp( -1 * path_man::path_to_path_dist(path,posHist_.topRows(posHistSize_)) ) // distance between candidate path and pose history path
                   + cGain_[1] * std::get<0>(pathYawHeightErr) // distance between the pose history and candidate path headings
                   + cGain_[2] * std::get<1>(pathYawHeightErr); // distance between the pose history and candidate path heights

    if( minCst > pathCst && pathMan_->path_len_check(path) ) // minimum path length condition
    {
      minCst = pathCst;
      minCstPath = path;
      idPathLeaf = idLeaves[i];
    }
  }

  return minCstPath;
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
    if( !pathMan_->path_len_check(path) )
    {
      nCurrGraphPaths++;
      continue;
    }

    if( gph->add_path(path, true) )
      success = true;
    nCurrGraphPaths++;
  }

  std::cout << "Adding lookahead path" << std::endl;
  // lookahead path
  // assuming already checked for minimum path length
  path = tree->get_path(idLookaheadLeaf);
  if( gph->add_path(path, false) )
    success = true;

  return success;
}

// ***************************************************************************
void scan_plan::goal_cb(const geometry_msgs::PointStamped& goalMsg)
{
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  //delete octTree_;

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octomap::OcTree* octTree = dynamic_cast<octomap::OcTree*>(tree);

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_octree(octTree);

  octMan_->update_esdf(localBndsMin_+robPos, localBndsMax_+robPos);
  octMan_->update_robot_pos(robPos);

  std::cout << "Validating path" << std::endl;
  changeDetected_ = !( pathMan_->validate_path(minCstPath_) );

  std::cout << "Publishing validated path" << std::endl;
  pathMan_->publish_path(minCstPath_, worldFrameId_, pathPub_);

  if( (isInitialized_ & 0x01) != 0x01 )
  {
    changeDetected_ = false;
    isInitialized_ = isInitialized_ | 0x01;
  }
}

// ***************************************************************************
Eigen::Vector3d scan_plan::geofence_saturation(const Eigen::Vector3d& posIn)
{
  Eigen::Vector3d posOut = posIn;

  if(posOut(0) < geoFenceMin_(0))
    posOut(0) = geoFenceMin_(0);
  if(posOut(0) > geoFenceMax_(0))
    posOut(0) = geoFenceMax_(0);

  if(posOut(1) < geoFenceMin_(1))
    posOut(1) = geoFenceMin_(1);
  if(posOut(1) > geoFenceMax_(1))
    posOut(1) = geoFenceMax_(1);

  if(posOut(2) < geoFenceMin_(2))
    posOut(2) = geoFenceMin_(2);
  if(posOut(2) > geoFenceMax_(2))
    posOut(2) = geoFenceMax_(2);

  return posOut;
}

// ***************************************************************************
void scan_plan::test_script()
{
 // graph gph(Eigen::Vector3d(0.6,0.5,12), 0.5);

  //return;

// 1. Grow tree, check the nodes, paths, distance of each node from the map
  //rrtTree_->update_oct_dist(octDist_);
  //rrtTree_->build(Eigen::Vector3d(0.1,0,1.1));

  //std::cout << "Distance to (0.1,0,1.1) is " << octDist_->getDistance (octomap::point3d(0,0,0)) << std::endl;
  //std::cout << "Distance to (2,0,1) is " << octDist_->getDistance (octomap::point3d(2,0,1)) << std::endl;

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
void scan_plan::pose_hist_cb(const nav_msgs::Path& poseHistMsg)
{
  if(poseHistMsg.poses.size() > posHist_.size())
  {
    ROS_INFO("Resizing pose history array");
    posHist_.conservativeResize(poseHistMsg.poses.size()+100, 3);
  }

  for(int i=0; i<poseHistMsg.poses.size(); i++)
  {
    posHist_(i,0) = poseHistMsg.poses[i].pose.position.x;
    posHist_(i,1) = poseHistMsg.poses[i].pose.position.y;
    posHist_(i,2) = poseHistMsg.poses[i].pose.position.z;
  }

  posHistSize_ = poseHistMsg.poses.size();
  if( (isInitialized_ & 0x02) != 0x02 )
    isInitialized_ = isInitialized_ | 0x02;
}

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete rrtTree_;
  delete graph_;
  delete octMan_;
  delete tfListenerPtr_;
}
