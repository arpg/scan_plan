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
  taskSub_ = nh->subscribe("task_in", 1, &scan_plan::task_cb, this);
  posHistNeighborsSub_ = nh->subscribe("pos_hist_neighbors_in", 1, &scan_plan::pos_hist_neighbors_cb, this);
  posHistNeighborSub_ = nh->subscribe("pos_hist_neighbor_in", 1, &scan_plan::pos_hist_neighbor_cb, this); // for one neighbor

  canPlanPub_ = nh->advertise<std_msgs::Bool>("can_plan_out", 10);
  planStatusPub_ = nh->advertise<std_msgs::String>("status_out", 10);
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
  while( !update_base_to_world() );

  ROS_INFO("%s: Base to world transform alive", nh_->getNamespace().c_str());
}
// ***************************************************************************
void scan_plan::setup_sensors()
{
  // prereqs setup_pose
  ROS_INFO("%s: Waiting for sensor params ...", nh_->getNamespace().c_str());
  std::vector<double> fovs, res, ranges;
  std::vector<std::string> sensorFrameIds;
  
  while(!nh_->getParam("sensor_frame_ids", sensorFrameIds) || sensorFrameIds.size() < 1);
  while(!nh_->getParam("sensor_ranges", ranges) || ranges.size() != sensorFrameIds.size());
  while(!nh_->getParam("sensor_fovs_horz_vert_degrees", fovs) || fovs.size() != 2*sensorFrameIds.size() ); // input in degress, converted to radians
  while(!nh_->getParam("sensor_res_horz_vert_pts", res) || res.size() != 2*sensorFrameIds.size() );

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
  double rrtSuccRad;
  while(!nh_->getParam("min_bounds_local", minBnds) || minBnds.size() != 3); // [x_min, y_min, z_min]
  while(!nh_->getParam("max_bounds_local", maxBnds) || maxBnds.size() != 3); // [x_max, y_max, z_max]

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
  while(!nh_->getParam("succ_rad_rrt", rrtSuccRad));

  ROS_INFO("%s: Setting up local tree ...", nh_->getNamespace().c_str());
  rrtTree_ = new rrt(rrtNNodes, minBnds, maxBnds, rrtRadNear, rrtDelDist, rrtSuccRad, rrtFailItr, octMan_);  
}

// ***************************************************************************
void scan_plan::setup_graph()
{
  // prereqs setup_pose, setup_octomap

  ROS_INFO("%s: Waiting for graph params ...", nh_->getNamespace().c_str());
  double graphRadNear, minVolGain, minDistNodes, minManDistFrontier, manRadAvoidFrontier, minSeparationRobots;
  int maxEdgesPerVertex, maxNAvoidFrontiers;
  std::vector<double> homePos, entranceMin, entranceMax, cGain;
 
  while(!nh_->getParam("near_radius_graph", graphRadNear));
  while(!nh_->getParam("min_distance_between_nodes", minDistNodes));
  while(!nh_->getParam("max_edges_per_vertex", maxEdgesPerVertex));
  while(!nh_->getParam("min_vol_gain_frontier", minVolGain)); // used for local/global switching and removing frontiers, (m^3)
  while(!nh_->getParam("home_position", homePos) || homePos.size() != 3); // must be collision-free
  while(!nh_->getParam("min_man_dist_frontiers", minManDistFrontier));
  while(!nh_->getParam("entrance_min_bnds", entranceMin) || entranceMin.size() != 3);
  while(!nh_->getParam("entrance_max_bnds", entranceMax) || entranceMax.size() != 3);
  while(!nh_->getParam("frontier_cost_gains", cGain) || cGain.size() != 2);
  while(!nh_->getParam("avoid_frontier_man_radius", manRadAvoidFrontier));
  while(!nh_->getParam("max_no_of_avoid_frontiers", maxNAvoidFrontiers)); // reset avoid frontier array to zero after this number, has to be atleast one for deconflict_replan
  while(!nh_->getParam("min_separation_robots", minSeparationRobots));
 
  ROS_INFO("%s: Setting up graph ...", nh_->getNamespace().c_str());
  homePos_(0) = homePos[0];
  homePos_(1) = homePos[1];
  homePos_(2) = homePos[2];

  graph_ = new graph(homePos_, graphRadNear, minDistNodes, maxEdgesPerVertex, minVolGain, worldFrameId_, octMan_, minManDistFrontier, entranceMin, entranceMax, cGain, manRadAvoidFrontier, maxNAvoidFrontiers, minSeparationRobots);
}

// ***************************************************************************
void scan_plan::setup_timers()
{
  ROS_INFO("%s: Waiting for timer params ...", nh_->getNamespace().c_str());

  while(!nh_->getParam("time_interval_replan", timeIntReplan_));
  while(!nh_->getParam("min_path_dist_time_based_replan", minPathDistTimerBasedReplan_));

  ROS_INFO("%s: Creating timers ...", nh_->getNamespace().c_str());
  timerReplan_ = nh_->createTimer(ros::Duration(timeIntReplan_), &scan_plan::timer_replan_cb, this);
}

// ***************************************************************************
void scan_plan::setup_octomap()
{
  // prereqs setup_sensors
  ROS_INFO("%s: Waiting for octomap params ...", nh_->getNamespace().c_str());

  std::string vehicleType;
  double maxGroundStep, maxGroundRoughnessThresh, avgGroundRoughnessThresh, maxDistEsdf, groundPlaneSearchDist, baseFrameHeightAboveGround, robWidth, robLength, successfulProjectionsPercent, successfulStairProjectionsPercent;
  bool esdfUnknownAsOccupied;
  bool useRoughness = true; // can be made parameter by uneccesary at this point
  bool useStairs = false;

  while(!nh_->getParam("esdf_max_dist", maxDistEsdf));
  while(!nh_->getParam("esdf_unknown_as_occupied", esdfUnknownAsOccupied));
  while(!nh_->getParam("vehicle_type", vehicleType)); // "air", "ground"
  while(!nh_->getParam("robot_width", robWidth));
  while(!nh_->getParam("robot_length", robLength));
  while(!nh_->getParam("ground_plane_search_distance", groundPlaneSearchDist));
  while(!nh_->getParam("base_frame_height_above_ground", baseFrameHeightAboveGround));
  while(!nh_->getParam("successful_projection_percent", successfulProjectionsPercent));
  while(!nh_->getParam("successful_stair_projection_percent", successfulStairProjectionsPercent));
  while(!nh_->getParam("max_ground_step", maxGroundStep)); // [epsilon,inf]
  while(!nh_->getParam("max_roughness_thresh", maxGroundRoughnessThresh));
  while(!nh_->getParam("avg_roughness_thresh", avgGroundRoughnessThresh));
  while(!nh_->getParam("is_stair_capable", useStairs));

  ROS_INFO("%s: Setting up octomap manager ...", nh_->getNamespace().c_str());

  octMan_ = new octomap_man(maxDistEsdf, esdfUnknownAsOccupied, useRoughness, vehicleType, robWidth, robLength, groundPlaneSearchDist, mapSensors_, baseFrameHeightAboveGround, successfulProjectionsPercent, successfulStairProjectionsPercent, maxGroundStep, maxGroundRoughnessThresh, avgGroundRoughnessThresh, useStairs);
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

  while(!nh_->getParam("path_cost_gains", cGain_) || cGain_.size() != 5);
  while(!nh_->getParam("min_bnds_geofence", geoFenceMin) || geoFenceMin.size() != 3);
  while(!nh_->getParam("max_bnds_geofence", geoFenceMax) || geoFenceMax.size() != 3); // in world frame
  while(!nh_->getParam("n_hist_pts_for_exploration_dir", nHistPosesExpDir_));
  while(!nh_->getParam("min_vol_gain_local_plan", minVolGainLocalPlan_));
  while(!nh_->getParam("no_of_tries_local_plan", nTriesLocalPlan_));
  while(!nh_->getParam("no_of_tries_global_plan", nTriesGlobalPlan_));
  while(!nh_->getParam("no_of_tries_replan", nTriesReplan_));
  while(!nh_->getParam("no_of_tries_path_validation", nTriesPathValidation_));
  while(!nh_->getParam("succ_rad_end_of_path", endOfPathSuccRad_));

  geoFenceMin_(0) = geoFenceMin[0];
  geoFenceMin_(1) = geoFenceMin[1];
  geoFenceMin_(2) = geoFenceMin[2];

  geoFenceMax_(0) = geoFenceMax[0];
  geoFenceMax_(1) = geoFenceMax[1];
  geoFenceMax_(2) = geoFenceMax[2];

  std::vector<double> minDynBnds, maxDynBnds;
  while(!nh_->getParam("min_bounds_local_dyn", minDynBnds) || minDynBnds.size() != 3); // [x_min, y_min, z_min]
  while(!nh_->getParam("max_bounds_local_dyn", maxDynBnds) || maxDynBnds.size() != 3); // [x_max, y_max, z_max]

  localBndsDynMin_(0) = minDynBnds[0];
  localBndsDynMin_(1) = minDynBnds[1];
  localBndsDynMin_(2) = minDynBnds[2];

  localBndsDynMax_(0) = maxDynBnds[0];
  localBndsDynMax_(1) = maxDynBnds[1];
  localBndsDynMax_(2) = maxDynBnds[2];
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
void scan_plan::task_cb_str(const std::string& taskIn)
{
  std_msgs::String taskMsg;
  taskMsg.data = taskIn;

  task_cb(taskMsg);
}

// ***************************************************************************
void scan_plan::task_cb(const std_msgs::String& taskMsg)
{
  // INDUCED TASK LOGIC FUNCTION

  if( (isInitialized_ & 0x03) != 0x03 )
    return;

  if( taskMsg.data == "end_of_path" || taskMsg.data == "eop" )
  {
    publish_plan_status("Ending path, replanning in the next iteration");
    status_.inducedEndOfPath = true;
    return;
  }

  if( taskMsg.data == "careful" )
  {
    publish_plan_status("Careful Path Validation Requested");
    octMan_->set_careful_validation(true);

    return;
  }

  if( taskMsg.data == "plan_local" )
  {
    publish_plan_status("Local Plan Requested");
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for computation

    Eigen::MatrixXd minCstPath = plan_locally(); // does not give a path if vol gain is low, outputs path that maintain neighbor separation if one exists

    if( minCstPath.rows() > 0 )
    {
      status_.mode = plan_status::MODE::LOCALEXP;
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
    {
      minCstPath = plan_locally("alpha", nTriesLocalPlan_); // gives a path regardless of vol gain or neighbor separation
      if(minCstPath.rows() > 0)
      {
        status_.mode = plan_status::MODE::LOCALEXP;
        minCstPath_ = minCstPath;
        publish_can_plan(true);
      }
      else
        publish_can_plan(false);
    }

    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise publish older path and don't change mode
    return;
  }

  if( taskMsg.data == "plan_global" )
  {
    publish_plan_status("Global Plan Requested");
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for computation

    if(graph_->is_empty_frontiers())
    {
      publish_can_plan(false);
      return;
    }

    Eigen::MatrixXd minCstPath = plan_globally();

    if(minCstPath.rows() > 1)
    {
      status_.mode = plan_status::MODE::GLOBALEXP;
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
      publish_can_plan(false);
    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise publish older path and don't change mode

    return;
  }

  if( boost::iequals(taskMsg.data , "deconflict") )
  {
    publish_plan_status("Deconflict request received");
    ROS_WARN("%s: Deconflict replan request received", nh_->getNamespace().c_str());
    if( minCstPath_.rows() < 1 ) // nothing to deconflict for an empty path
    {
      publish_can_plan(false);
      return;
    }

    if( (status_.mode != plan_status::MODE::LOCALEXP) && (status_.mode != plan_status::MODE::GLOBALEXP) )
    {
      publish_can_plan(false);
      return;
    }

    ROS_WARN("%s: Deconflict replan triggered", nh_->getNamespace().c_str());
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for safety

    Eigen::Vector3d avoidPos = minCstPath_.bottomRows(1).transpose();
    graph_->add_avoid_frontier(avoidPos); // temporarily avoid the end position, clear avoidPos array at exit

    Eigen::MatrixXd minCstPath = plan_exploration_path();
    if( minCstPath.rows() > 0 )
    {
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
      publish_can_plan(false);

    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_);
    graph_->remove_avoid_frontier(avoidPos);
  }

  if( boost::iequals(taskMsg.data, "unstuck") ) // gonna replan every time it's received
  {
    publish_plan_status("Unstuck request received");
    ROS_WARN("%s: Unstuck request received", nh_->getNamespace().c_str());
    if( minCstPath_.rows() > 0 )
      graph_->add_avoid_frontier(minCstPath_.bottomRows(1).transpose()); // temporarily avoid the end position, TODO: clear avoidPos array when the end of stuck path is reached / on timer

    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for safety
    Eigen::MatrixXd minCstPath = plan_locally("random", nTriesLocalPlan_, false); // randomly chooses a path regardless of any cost, dont add any path to graph

    if(minCstPath.rows() > 0)
    {
      status_.mode = plan_status::MODE::UNSTUCK; // only turn to unstuck mode if you have a path
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
    {
      minCstPath_ = Eigen::MatrixXd(0,0);
      publish_can_plan(false);
    }

    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise keep path empty to trigger eop of the current task

    return;
  }

  //TODO: right now it only replans to the goal point if the mode changes but what if it's already in goal point mode and the goal point changes 
  if( ( boost::iequals(taskMsg.data, "guiCommand") || boost::iequals(taskMsg.data, "guiCmd") ) && status_.mode != plan_status::MODE::GOALPT )
  {  
    publish_plan_status("Gui request for goal point received");
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for computation
    Eigen::MatrixXd minCstPath = plan_to_point(status_.goalPt);

    if(minCstPath.rows() > 0)
    {
      status_.mode = plan_status::MODE::GOALPT;
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
      publish_can_plan(false);
    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise publish older path and don't change mode

    return;
  }

  // if the robot is following unstuck path, only gui goal point can override it
  //if( status_.mode == plan_status::MODE::UNSTUCK )
  //  return;

  if( ( boost::iequals(taskMsg.data, "report") || boost::iequals(taskMsg.data, "home") ) && status_.mode != plan_status::MODE::REPORT )
  {
    publish_plan_status("Go home request received");
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for computation

    Eigen::MatrixXd minCstPath = plan_home(); // try planning home
    if(minCstPath.rows() > 0) // path found
    {
      status_.mode = plan_status::MODE::REPORT;
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
      publish_can_plan(false);
    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise publish older path and don't change mode

    return;
  }

  if( boost::iequals(taskMsg.data, "explore") && (status_.mode != plan_status::MODE::LOCALEXP && status_.mode != plan_status::MODE::GLOBALEXP) )
  {
    publish_plan_status("Explore request received");

    Eigen::MatrixXd minCstPath = plan_exploration_path();

    if(minCstPath.rows() > 0)  // if local path is found publish it, otherwise leave in local exp mode so the timer retries and figures it out
    {
      minCstPath_ = minCstPath;
    }
    else
    {
      minCstPath_ = Eigen::MatrixXd(0,0); // because timer retries at the end-of-task triggered by the end-of-path
    }

    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise empty path 

    return;
  }
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  //delete octTree_;

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  OcTreeT* octTree = (OcTreeT*)tree;

  //publish_plan_mode();

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );

  octMan_->update_octree( octTree );

  octMan_->update_esdf(localBndsMin_+robPos, localBndsMax_+robPos);
  octMan_->update_robot_pos(robPos);

  status_.mapUpdated = true;

  if( (isInitialized_ & 0x01) != 0x01 )
    isInitialized_ = isInitialized_ | 0x01;
}

// ***************************************************************************
bool scan_plan::end_of_path()
{
  if( status_.inducedEndOfPath ) // if end-of-path is induced
  {
    status_.inducedEndOfPath = false;
    return true;
  }

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  if(minCstPath_.rows() < 1) // if no valid path
    return true;
  if( (robPos.transpose()-minCstPath_.bottomRows(1)).norm() < endOfPathSuccRad_ ) // if the robot is near the end point of the path
    return true;

  std::cout << "Robot-Path Distance: " << path_man::point_to_path_dist(robPos, minCstPath_) << std::endl;

  if( path_man::point_to_path_dist(robPos, minCstPath_) > minPathDistTimerBasedReplan_ )
    return true; // if robot is away from planned path and time interval has elapsed

  return false;
}

// ***************************************************************************
void scan_plan::timer_replan_cb(const ros::TimerEvent&) // running at a fast rate
{
  // END-OF-TASK LOGIC FUNCTION

  // stops timer to clear the queque, don't forget to start on every return

  if( (isInitialized_ & 0x03) != 0x03 )
    return;

  bool triedReplan = true;

  ros::Time timeS = ros::Time::now();

  Eigen::MatrixXd minCstPathPrev = minCstPath_;

  if(status_.mode == plan_status::MODE::UNSTUCK && end_of_path() )
  {
    stop_if_cliff_after_eop();

    graph_->clear_avoid_frontiers();
    if(minCstPath_.rows() > 0) graph_->add_path(minCstPath_, false);  // add unstuck path to graph after robot starts moving to avoid dense graph around the stationary robot   

    Eigen::MatrixXd minCstPath = plan_exploration_path(); // vehicle already stopped for computation due to EOP
    if(minCstPath.rows() > 0) // exploration path found
    {
      ROS_WARN("%s: Exploration path found", nh_->getNamespace().c_str());
      minCstPath_ = minCstPath;
    }
  }
  else if(status_.mode == plan_status::MODE::REPORT && end_of_path())
  {
    ROS_WARN("%s: Report mode end-of-path, planning exploration path", nh_->getNamespace().c_str());
    Eigen::MatrixXd minCstPath = plan_exploration_path(); // vehicle already stopped for computation due to EOP
    if(minCstPath.rows() > 0) // exploration path found
    {
      ROS_WARN("%s: Exploration path found", nh_->getNamespace().c_str());
      minCstPath_ = minCstPath;
    }
  }
  else if(status_.mode == plan_status::MODE::GOALPT && end_of_path())
  {
    ROS_WARN("%s: Goal point mode end-of-path, planning exploration path", nh_->getNamespace().c_str());
    Eigen::MatrixXd minCstPath = plan_exploration_path(); // vehicle already stopped for computation due to EOP
    if(minCstPath.rows() > 0) // exploration path found
    {
      ROS_WARN("%s: Exploration path found", nh_->getNamespace().c_str());
      minCstPath_ = minCstPath;
    }
  }
  else if(status_.mode == plan_status::MODE::GLOBALEXP && end_of_path())
  {
    stop_if_cliff_after_eop();

    update_base_to_world();
    Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
    octMan_->update_robot_pos(robPos);

    if( minCstPath_.rows() > 0 && (robPos.transpose()-minCstPath_.bottomRows(1)).norm() < endOfPathSuccRad_ ) // if reached frontier force remove it
      graph_->remove_frontier(minCstPath_.bottomRows(1).transpose());

    ROS_WARN("%s: Global exp mode end-of-path, planning exploration path", nh_->getNamespace().c_str());
    Eigen::MatrixXd minCstPath = plan_exploration_path(); // vehicle already stopped for computation due to EOP
    if(minCstPath.rows() > 0) // exploration path found
    {
      ROS_WARN("%s: Exploration path found", nh_->getNamespace().c_str());
      minCstPath_ = minCstPath;
    }
  }
  else if( (status_.mode == plan_status::MODE::LOCALEXP) && end_of_path())
  {
    stop_if_cliff_after_eop();

    ROS_WARN("%s: Local exp mode end-of-path, planning exploration path", nh_->getNamespace().c_str());
    Eigen::MatrixXd minCstPath = plan_exploration_path(); // vehicle already stopped for computation due to EOP
    if(minCstPath.rows() > 0) // exploration path found
    {
      ROS_WARN("%s: Exploration path found", nh_->getNamespace().c_str());
      minCstPath_ = minCstPath;
    }
  }
  else if( (status_.mode == plan_status::MODE::MOVEANDREPLAN) ) // buggy perception/map
  {
    ROS_WARN("%s: Could not sample enough around the robot, following pose history and replanning", nh_->getNamespace().c_str());

    Eigen::MatrixXd minCstPath = plan_locally("alpha", nTriesLocalPlan_); // gives a path regardless of vol gain
    if(minCstPath.rows() > 0)
    {
      minCstPath_ = minCstPath;
      status_.mode = plan_status::MODE::LOCALEXP;
    }
  }
  else
    triedReplan = false;

  bool didReplan = !path_man::are_equal(minCstPathPrev, minCstPath_); 

  // already catering for failed replans if in MOVEANDREPLAN, no need to increment, no need to validate the path
  if( triedReplan && !didReplan && status_.mode != plan_status::MODE::MOVEANDREPLAN )
    status_.nFailedReplans++;
  else if( didReplan || status_.mode == plan_status::MODE::MOVEANDREPLAN )
  {
    status_.nFailedReplans = 0;
    status_.nPathInvalidations = 0;
  }

  std::cout << "Number of failed replans: " << status_.nFailedReplans << std::endl;

  if( status_.nFailedReplans >= nTriesReplan_ ) // assuming nTriesReplan_ > 0
  {
    minCstPath_ = posHist_.topRows(posHistSize_).colwise().reverse();
    status_.mode = plan_status::MODE::MOVEANDREPLAN;

    didReplan = true;
    status_.nFailedReplans = 0;
    status_.nPathInvalidations = 0;
  }

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  std::cout << "Number of path rows before validation: " << minCstPath_.rows() << std::endl;
  bool isPathValid = true;

  if( status_.mode != plan_status::MODE::MOVEANDREPLAN || octMan_->get_careful_validation() )
   isPathValid = pathMan_->validate_path_without_mod(minCstPath_, robPos+localBndsDynMin_, robPos+localBndsDynMax_);

  std::cout << "Number of path rows after validation: " << minCstPath_.rows() << std::endl;

  if( isPathValid )
  {
    status_.nPathInvalidations = 0; // start counting again if a valid path is encountered
    octMan_->set_careful_validation(false);
  }
  else
    status_.nPathInvalidations++;

  std::cout << "Number of path invalidations: " << status_.nPathInvalidations << std::endl;

  // if the vehicle is not following the pose graph, then validate the path
  if( status_.nPathInvalidations >= nTriesPathValidation_ ) 
  {
    minCstPath_ = Eigen::MatrixXd(0,3); // TODO: the clipped path can be clipped behind the vehicle causing the vehicle to go back which needs to be fixed to take this statement out
    path_man::publish_empty_path(worldFrameId_, pathPub_);
    std::cout << "Updating graph occupancy (all)" << std::endl;

    graph_-> update_occupancy(localBndsDynMin_+robPos, localBndsDynMax_+robPos, false); // if path is hitting/dynamic obstacle appeared update occupany of all edges, so that points can be sampled in the neighboorhood of appearing obstacle cz the robot is there, this prevents graph disconnections

    status_.nPathInvalidations = 0;
    octMan_->set_careful_validation(false);

    if( status_.mode == plan_status::MODE::GOALPT )
    {
      status_.mode = plan_status::MODE::LOCALEXP; // set a different mode to retrigger GOALPT, also fall back to it if task_cb doesn't update the path 
      task_cb_str("GOALPT");
    }
    if( status_.mode == plan_status::MODE::REPORT )
    {
      status_.mode = plan_status::MODE::LOCALEXP; // set a different mode to retrigger REPORT, also fall back to it if task_cb doesn't update the path 
      task_cb_str("REPORT");
    }
  }
  else
  {
    std::cout << "Updating graph occupancy (occupied only)" << std::endl;
    graph_-> update_occupancy(geoFenceMin_, geoFenceMax_, true); // if path is obstacle-free update occupany of only occupied edges
   }

  if(didReplan)
  {
    // only publish graph when replan happens to keep bags from going big
    std::cout << "Publishing frontiers" << std::endl; 
    graph_->publish_frontiers(frontiersPub_); 

    std::cout << "Publishing  graph viz" << std::endl;
    graph_->publish_viz(vizPub_);
  }

  //TODO: Generate graph diconnect warning if no path is successfully added to the graph
  // Only add node to the graph if there is no existing node closer than radRob in graph lib, return success in that case since the graph is likely to be connected fine
  // update older node if within robot radius with updated terrain and collision information, the latter may mean taking nodes out of the graph
  // consider immidiate replan if a path is not found by-passing timer

  //TODO: Consider including a check, update path if end-of-path is reached, and increase the replan time

  ///graph_->update_frontiers_vol_gain();
   
  ros::Duration timeE = ros::Time::now() - timeS;

  std::cout << "Publishing rrt viz" << std::endl;
  rrtTree_->publish_viz(vizPub_,worldFrameId_);

  std::cout << "Publishing plan status" << std::endl;
  publish_plan_status();

  std::cout << "Publishing lookahead path" << std::endl;
  path_man::publish_path(minCstPath_, worldFrameId_, pathPub_);

  std_msgs::Float64 computeTimeMsg;
  computeTimeMsg.data = timeE.toSec();
  compTimePub_.publish(computeTimeMsg);
  
  std::cout << "Replan Compute Time = " << computeTimeMsg.data << " sec" << std::endl;

  // clear the queque, reset the time
  timerReplan_.stop();
  timerReplan_.start();
}

// ***************************************************************************
void scan_plan::stop_if_cliff_after_eop()
{
  if(minCstPath_.rows() < 2 || octMan_->vehicle_type() == "air") // need atleast two path points to extrapolate
    return;

  Eigen::Vector3d pos1 = minCstPath_.row(minCstPath_.rows()-2).transpose();
  Eigen::Vector3d pos2 = minCstPath_.row(minCstPath_.rows()-1).transpose();
  double heading = std::atan2( (pos2(1)-pos1(1)) , (pos2(0)-pos1(0)) );

  if( octMan_->check_for_cliff(pos2, heading) )
   path_man::publish_empty_path(worldFrameId_, pathPub_);
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_exploration_path()
{
  ROS_WARN("%s: Planning locally", nh_->getNamespace().c_str());
  status_.mode = plan_status::MODE::LOCALEXP;
  Eigen::MatrixXd minCstPathLocal = plan_locally(); // does not give a path if vol gain is low, outputs path that maintain neighbor separation if one exists

  double localPathSeparation = -1.0;
  if( minCstPathLocal.rows() > 0 )
  {
    ROS_WARN("%s: Local path found", nh_->getNamespace().c_str());
    localPathSeparation = path_man::point_to_paths_dist(minCstPathLocal.bottomRows(1).transpose(), posHistNeighbors_);
    if( ( localPathSeparation < 0 || localPathSeparation >= graph_->get_min_separation_robots() ) )
    {
      status_.mode = plan_status::MODE::LOCALEXP;
      return minCstPathLocal;
    }
    ROS_WARN("%s: No local path with enough neighbor separation", nh_->getNamespace().c_str());
  }
  else
    ROS_WARN("%s: No local path with enough vol gain", nh_->getNamespace().c_str());

  // either no local path with enough vol gain exists or it doesn't maintain enough separation with neigbors  

  double globalPathSeparation = -1.0;
  Eigen::MatrixXd minCstPathGlobal(0,0);
  if(!graph_->is_empty_frontiers()) 
  {
    ROS_WARN("%s: Frontier list is not empty, planning globally", nh_->getNamespace().c_str());
    minCstPathGlobal = plan_globally(); // vehicle already stopped for computation due to EOP

    if( minCstPathGlobal.rows() > 0 )
    {
      ROS_WARN("%s: Global path found", nh_->getNamespace().c_str());
      globalPathSeparation = path_man::point_to_paths_dist(minCstPathGlobal.bottomRows(1).transpose(), posHistNeighbors_);
      if( ( globalPathSeparation < 0 || globalPathSeparation >= graph_->get_min_separation_robots() ) )
      {
        status_.mode = plan_status::MODE::GLOBALEXP;
        return minCstPathGlobal;
      }
      ROS_WARN("%s: No Global path with enough neighbor separation", nh_->getNamespace().c_str());
    }
    else
      ROS_WARN("%s: Global path cannot be planned", nh_->getNamespace().c_str());
  }


   // No path with enough vol gain and enough neighbor separation exist but minCstPathLocal and minCstPathGlobal may have enough vol gain paths

   if( minCstPathLocal.rows() > 0 && minCstPathGlobal.rows() > 0 )
   {
     if( globalPathSeparation > localPathSeparation )
     {
       ROS_WARN("%s: Global path has better separation", nh_->getNamespace().c_str());
       status_.mode = plan_status::MODE::GLOBALEXP;
       return minCstPathGlobal;
     }
     else
     {
       ROS_WARN("%s: Local path has better separation", nh_->getNamespace().c_str());
       status_.mode = plan_status::MODE::LOCALEXP;
       return minCstPathLocal;
     }
   }

   if( minCstPathLocal.rows() > 0 )
   {
     ROS_WARN("%s: Sending local path", nh_->getNamespace().c_str());
     status_.mode = plan_status::MODE::LOCALEXP;
     return minCstPathLocal;
   }

   if( minCstPathGlobal.rows() > 0 )
   {
     ROS_WARN("%s: Sending global path", nh_->getNamespace().c_str());
     status_.mode = plan_status::MODE::GLOBALEXP;
     return minCstPathGlobal;
   }

   ROS_WARN("%s: Planning locally with alpha cost", nh_->getNamespace().c_str());
   minCstPathLocal = plan_locally("alpha", nTriesLocalPlan_); // gives a path regardless of vol gain or neighbor separation
   if(minCstPathLocal.rows() > 0)
   {
     status_.mode = plan_status::MODE::LOCALEXP;
     return minCstPathLocal;
   }


   status_.mode = plan_status::MODE::LOCALEXP;
   return Eigen::MatrixXd(0,0); // giving up, nothing found
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_home()
{
  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  gvert gphVert;
  gphVert.pos = robPos;
  gphVert.commSig = 0;
  gphVert.isFrontier = false;
  gphVert.terrain = gvert::Terrain::UNKNOWN;

  ROS_WARN("%s: Planning Home", nh_->getNamespace().c_str());

  std::cout << "Planning to graph" << std::endl;
  double latestPosDist;
  VertexDescriptor srcVertD = graph_->latest_pos_vert(robPos, latestPosDist);

  if( latestPosDist < 0.0 || latestPosDist > minPathDistTimerBasedReplan_ )  // TODO: 5.0 -> path start tolerance from the rob pos, like endOfPathSuccRad_ 
  {
    std::cout << "Couldn't find the latest pos_hist pos nearby ( latestPosDist = " <<  latestPosDist << "), looking for the closest vertex" << std::endl;
    srcVertD = graph_->closest_vertex(robPos, latestPosDist);

    if( latestPosDist < 0.0 || latestPosDist > minPathDistTimerBasedReplan_ ) 
    {
      std::cout << "Couldn't find the closest vertex nearby, adding a new vertex" << std::endl;
      if( !graph_->add_vertex(gphVert, srcVertD, true) ) // bool ignoreMinDistNodes = true
        return Eigen::MatrixXd(0,0);
    }
  }

  std::cout << "Planning vertex to home vertex" << std::endl;
  Eigen::MatrixXd minCstPath = graph_->plan_home(srcVertD);
  if(minCstPath.rows() < 2)
    return Eigen::MatrixXd(0,0); // posHist_.topRows(posHistSize_).colwise().reverse();

  return minCstPath;
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_to_point(const Eigen::Vector3d& goalPos)
{
  // Input goalPos is NOT assumed to be on the ground

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  gvert gphVert;
  gphVert.pos = robPos;
  gphVert.commSig = 0;
  gphVert.isFrontier = false;
  gphVert.terrain = gvert::Terrain::UNKNOWN;

  std::cout << "Planning to graph" << std::endl;
  double latestPosDist;
  VertexDescriptor srcVertD = graph_->latest_pos_vert(robPos, latestPosDist);

  if( latestPosDist < 0.0 || latestPosDist > minPathDistTimerBasedReplan_ )  // TODO: 5.0 -> path start tolerance from the rob pos, like endOfPathSuccRad_ 
  {
    std::cout << "Couldn't find the latest pos_hist pos nearby ( latestPosDist = " <<  latestPosDist << "), looking for the closest vertex" << std::endl;
    srcVertD = graph_->closest_vertex(robPos, latestPosDist);

    if( latestPosDist < 0.0 || latestPosDist > minPathDistTimerBasedReplan_ ) 
    {
      std::cout << "Couldn't find the closest vertex nearby, adding a new vertex" << std::endl;
      if( !graph_->add_vertex(gphVert, srcVertD, true) ) // bool ignoreMinDistNodes = true
        return Eigen::MatrixXd(0,0);
    }
  }

  std::cout << "Planning from graph" << std::endl;

  double closestGoalVertDist;
  VertexDescriptor clstVertD = graph_->closest_vertex(goalPos, closestGoalVertDist);
  if( closestGoalVertDist < 0.0 )
  {
    std::cout << "Could not find graph" << std::endl;
    return Eigen::MatrixXd(0,0);
  }

  octMan_->update_esdf(localBndsMin_+goalPos, localBndsMax_+goalPos); // getting the esdf around the goal rather than default robot position

  bool goalConnected = false;
  VertexDescriptor goalVertD;

  Eigen::Vector3d goalPtProj;
  double roughness; bool isOnStairs;
  if( octMan_->vehicle_type() != "air" )
  {
    if( octMan_->cast_ray_down(goalPos, goalPtProj, roughness, isOnStairs) == 1 ) // valid projection
    {
      goalPtProj(2) += octMan_->get_base_frame_height_above_ground();
      gphVert.pos = goalPtProj;
      goalConnected = graph_->add_vertex(gphVert, goalVertD, true);
    }
    else
    {
      goalPtProj = goalPos;
      ROS_WARN("%s: Could not project the goal point to the ground", nh_->getNamespace().c_str());
    }
  }
  else
  {
    goalPtProj = goalPos;
    gphVert.pos = goalPtProj;
    goalConnected = graph_->add_vertex(gphVert, goalVertD, true);
  }

  octMan_->update_esdf(localBndsMin_+robPos, localBndsMax_+robPos);  // getting the esdf back around default robot position

  std::cout << "Planning vertex to vertex" << std::endl;
  Eigen::MatrixXd minCstPath(0,0);
  if( goalConnected )
    minCstPath = graph_->plan_shortest_path(srcVertD, goalVertD);

  if(minCstPath.rows() < 2)
    minCstPath = graph_->plan_shortest_path(srcVertD, clstVertD);

  if(minCstPath.rows() < 2)
    return Eigen::MatrixXd(0,0);

  // This straight line path can be longer than near_rad_graph causing a disconnection, but once goal and robPos are added to the graph why not try a straighter path
  std::cout << "Checking if goal and robot positions can be connected via a straight line" << std::endl;
  if( !octMan_->u_coll(robPos, goalPtProj) ) // try to connect via straight line
  {
    minCstPath = Eigen::MatrixXd(2,3);

    minCstPath.row(0) = robPos;
    minCstPath.row(1) = goalPtProj;
  }

  return minCstPath;
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_from_graph(const Eigen::Vector3d& toPos, VertexDescriptor& fromVertex)
{
  // returns a path and the graph vertex start point
  return ( plan_to_graph(toPos, fromVertex) ).colwise().reverse(); // plan from the goal point to graph, then flip the path 
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

  std::cout << "From Position: " << fromPos.transpose() << std::endl;
  std::cout << "Checking Nearby Graph Vertices: ";

  std::cout << "Connecting to graph via a straight line" << std::endl;
  for(int i=0; i<vertices.size(); i++)
  {
    toVertex = vertices[i];
    Eigen::Vector3d vertexPos = graph_->get_pos(toVertex);

    std::cout << vertexPos.transpose() << std::endl;

    if( octMan_->u_coll( fromPos, vertexPos) )
      continue;

    Eigen::MatrixXd path(2,3);

    path.row(0) = fromPos;
    path.row(1) = vertexPos;
    return path;
  }

  std::cout << "Connecting to graph via an rrt path" << std::endl;
  for(int i=0; i<vertices.size(); i++)
  {
    toVertex = vertices[i];
    Eigen::Vector3d vertexPos = graph_->get_pos(toVertex);

    int leafId = update_local_tree(fromPos, vertexPos); // using goal-bias rrt to find a path 

    if( leafId < 1 ) // if root node is returned / if a path is not found
      continue;

    Eigen::MatrixXd path = rrtTree_->get_path(leafId);
    return path;
  }

  std::cout << "Giving up" << std::endl;
  return Eigen::MatrixXd(0,0);
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_globally() 
{
  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  graph_->update_frontiers_vol_gain();

  if( graph_->is_empty_frontiers() )
    return Eigen::MatrixXd(0,0);
/*
  std::cout << "Connecting to the frontier via a straight line" << std::endl;
  if( !octMan_->u_coll( robPos, graph_->get_pos(front.vertDesc) ) )
  {
    Eigen::MatrixXd path(2,3);

    path.row(0) = robPos;
    path.row(1) = graph_->get_pos(front.vertDesc);
    return path;
  }
*/

  gvert gphVert;
  gphVert.pos = robPos;
  gphVert.commSig = 0;
  gphVert.isFrontier = false;
  gphVert.terrain = gvert::Terrain::UNKNOWN;

  std::cout << "Planning to graph" << std::endl;
  double latestPosDist;
  VertexDescriptor srcVertD = graph_->latest_pos_vert(robPos, latestPosDist);

  if( latestPosDist < 0.0 || latestPosDist > minPathDistTimerBasedReplan_ )  // TODO: 5.0 -> path start tolerance from the rob pos, like endOfPathSuccRad_ 
  {
    std::cout << "Couldn't find the latest pos_hist pos nearby ( latestPosDist = " <<  latestPosDist << "), looking for the closest vertex" << std::endl;
    srcVertD = graph_->closest_vertex(robPos, latestPosDist);

    if( latestPosDist < 0.0 || latestPosDist > minPathDistTimerBasedReplan_ ) 
    {
      std::cout << "Couldn't find the closest vertex nearby, adding a new vertex" << std::endl;
      if( !graph_->add_vertex(gphVert, srcVertD, true) ) // bool ignoreMinDistNodes = true
        return Eigen::MatrixXd(0,0);
    }
  }

  std::cout << "Path to graph non-empty, planning over graph" << std::endl;
  Eigen::MatrixXd minCstPath = graph_->plan_to_frontier(srcVertD, nTriesGlobalPlan_, posHistNeighbors_);

  if(minCstPath.rows() < 2)
    return Eigen::MatrixXd(0,0);
  std::cout << "Planning over graph successful" << std::endl;

  return minCstPath;
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_locally()
{
  Eigen::MatrixXd minCstPath;
  
  std::cout << "Planning with alpha cost, no of tries: " << nTriesLocalPlan_ << std::endl;
  minCstPath = plan_locally("alpha", nTriesLocalPlan_);

  if( minCstPath.rows() > 0 )
  {
    double separationDist = path_man::point_to_paths_dist(minCstPath.bottomRows(1).transpose(), posHistNeighbors_);
    double volGain = octMan_->volumetric_gain(minCstPath.bottomRows(1).transpose());

    if( (volGain >= minVolGainLocalPlan_) && (separationDist < 0 || separationDist >= graph_->get_min_separation_robots()) ) // if alpha path has enough vol gain and separation, return it
      return minCstPath;
  }

  // if no path is found or the path found does not have enough vol gain, continue to using beta cost
  std::cout << "Not enough vol gain planning with beta cost, no of tries: " << nTriesLocalPlan_ << std::endl;
  minCstPath = plan_locally("beta", nTriesLocalPlan_);

  if( minCstPath.rows() > 0 ) // beta paths all are above min vol gain requirement, so no need to check for vol gain
    return minCstPath;

  // giving up, send empty path
  std::cout << "Not enough vol gain, sending empty path" << std::endl;
  return Eigen::MatrixXd(0,0);
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_locally(const std::string& costType, const int& nTries, bool addToGraph)
{
  // plan_locally does not return a path if no path is found that satisfies path length requirement or if all paths lead to entrance or if vol gain of all paths is low (in beta mode)

  Eigen::MatrixXd minCstPath(0,0);
  for(int i=0; i<nTries; i++)
  {
    minCstPath = plan_locally(costType, addToGraph); // try with costType until a path is found
    if( minCstPath.rows() > 0 )
      return minCstPath;
  }
  return Eigen::MatrixXd(0,0);
} 

// ***************************************************************************
void scan_plan::update_local_tree(const Eigen::Vector3d& robPos)
{
  if( octMan_->vehicle_type() == "ground" ) // if "ground", clip local bounds from below, assuming base frame is the highest point of the robot
    rrtTree_->set_bounds( geofence_saturation( localBndsMin_+robPos-Eigen::Vector3d(0,0,localBndsMin_(2)) ), geofence_saturation(localBndsMax_+robPos) );
  else
    rrtTree_->set_bounds( geofence_saturation(localBndsMin_+robPos), geofence_saturation(localBndsMax_+robPos) );
  rrtTree_->build(robPos);
}

// ***************************************************************************
int scan_plan::update_local_tree(const Eigen::Vector3d& robPos, const Eigen::Vector3d& goalPos)
{
  if( octMan_->vehicle_type() == "ground" ) // if "ground", clip local bounds from below, assuming base frame is the highest point of the robot
    rrtTree_->set_bounds( geofence_saturation( localBndsMin_+robPos-Eigen::Vector3d(0,0,localBndsMin_(2)) ), geofence_saturation(localBndsMax_+robPos) );
  else
    rrtTree_->set_bounds( geofence_saturation(localBndsMin_+robPos), geofence_saturation(localBndsMax_+robPos) );
  return rrtTree_->build(robPos, goalPos);
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_locally(const std::string& costType, bool addToGraph) 
{
  // returns rrt lookahead path, ids of all leaves and of the lookahead path leaf
  // get_path function in rrt can be used to get paths for a leaf id

  Eigen::MatrixXd minCstPath(0,0);
  int idPathLeaf = -1;
  std::vector<int> idLeaves;

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  update_local_tree(robPos);

  idLeaves = rrtTree_->get_leaves();
  disp(idLeaves.size(), "Number of paths found");

  // get all paths ending at leaves and choose one with min cost
  double currRobYaw = quat_to_yaw(baseToWorld_.transform.rotation);
  std::pair<double, double> currExpYawHeight = path_man::mean_heading_height(posHist_.topRows(posHistSize_), nHistPosesExpDir_); ////////
  
  if(costType == "alpha") // maintain exploration heading and height
  {
    double minCst = std::numeric_limits<double>::max();

    for(int i=0; i<idLeaves.size(); i++)
    {
      Eigen::MatrixXd path = rrtTree_->get_path(idLeaves[i]);
      if(!pathMan_->path_len_check(path) || path.rows() < 1) // minimum path length condition
        continue;
      if( graph_->is_avoid_frontier(path.bottomRows(1).transpose()) ) // avoid blacklisted frontiers
        continue;
      if( graph_->is_entrance(path.bottomRows(1).transpose()) ) // avoid entrance
        continue;

      double pathCst = path_cost_alpha(path, std::get<0>(currExpYawHeight), std::get<1>(currExpYawHeight));

      if( (minCst > pathCst) ) // minimum path length condition
      {
        minCst = pathCst;
        minCstPath = path;
        idPathLeaf = idLeaves[i];
      }
    } 
  }
  else if(costType == "beta") // maintain exploration and height but ignore low vol gain frontier
  {
    double minCst = std::numeric_limits<double>::max();
    bool foundWellSeparatedPath = false;

    for(int i=0; i<idLeaves.size(); i++)
    {
      Eigen::MatrixXd path = rrtTree_->get_path(idLeaves[i]);
      if(!pathMan_->path_len_check(path) || path.rows() < 1) // minimum path length condition
        continue;
      if( graph_->is_avoid_frontier(path.bottomRows(1).transpose()) ) // avoid blacklisted frontiers
        continue;
      if( graph_->is_entrance(path.bottomRows(1).transpose()) ) // avoid entrance
        continue;

      double volGain, separationDist;
      double pathCst = path_cost_beta(path, std::get<0>(currExpYawHeight), std::get<1>(currExpYawHeight), volGain, separationDist);

      bool isWellSeparatedPath = ( (separationDist < 0.0) || (separationDist >= graph_->get_min_separation_robots()) ); // 

      if( !foundWellSeparatedPath && isWellSeparatedPath && volGain >= minVolGainLocalPlan_ ) // if this is the first well-separated good vol gain path, mark it minCst
      {
        minCst = pathCst;
        minCstPath = path;
        idPathLeaf = idLeaves[i];

        foundWellSeparatedPath = true;
        continue;
      }

      if( foundWellSeparatedPath && !isWellSeparatedPath ) // if we have a well-separated path, reject the future ones that are not
        continue;
 
      // skip paths with low vol gain
      if( (volGain >= minVolGainLocalPlan_ &&  minCst > pathCst) ) 
      {
        minCst = pathCst;
        minCstPath = path;
        idPathLeaf = idLeaves[i];
      }
    }
  }
  else if(costType == "random") // random path
  {
    for(int i=0; i<idLeaves.size(); i++)
    {
      Eigen::MatrixXd path = rrtTree_->get_path(idLeaves[i]);
      if(!pathMan_->path_len_check(path) || path.rows() < 1) // minimum path length condition
        continue;
      if( graph_->is_avoid_frontier(path.bottomRows(1).transpose()) ) // avoid pos condition
        continue;
      if( graph_->is_entrance(path.bottomRows(1).transpose()) )
        continue;

      minCstPath = path;
      idPathLeaf = idLeaves[i];
    
      break;
    }
  }
  else
    ROS_ERROR("%s: Unknown tree propagation mode", nh_->getNamespace().c_str());

  //std::cout << std::endl;

  if( minCstPath.rows() > 1 && idPathLeaf >= 0 && addToGraph) // second condition is redundant with first one
  {
    std::cout << "Adding paths to the graph" << std::endl;
    bool success = add_paths_to_graph(rrtTree_, idLeaves, idPathLeaf, graph_);
    if(!success)
      ROS_WARN("%s: Graph connection unsuccessful", nh_->getNamespace().c_str());
  }

  return minCstPath;
}

// ***************************************************************************
double scan_plan::path_cost_alpha(const Eigen::MatrixXd& path, const double& currExpYaw, const double& currExpHeight)
{
  // assumes path to have atleast one vertex, ideally more than one

  std::pair<double, double> pathYawHeightErr = path_man::mean_heading_height_err(currExpYaw, currExpHeight, path);
  return cGain_[0] * ( -1 * path_man::path_to_path_dist( posHist_.topRows(posHistSize_),path ) ) // distance between candidate path and pose history path
         + cGain_[1] * std::get<0>(pathYawHeightErr) // distance between the pose history and candidate path headings
         + cGain_[2] * std::get<1>(pathYawHeightErr); // distance between the pose history and candidate path heights
}

// ***************************************************************************
double scan_plan::path_cost_beta(const Eigen::MatrixXd& path, const double& currExpYaw, const double& currExpHeight, double& volGain, double& separationDist)
{
  // assumes path to have atleast one vertex, ideally more than one

  std::pair<double, double> pathYawHeightErr = path_man::mean_heading_height_err(currExpYaw, currExpHeight, path);
  volGain = octMan_->volumetric_gain( path.bottomRows(1).transpose() );

  separationDist = path_man::point_to_paths_dist( path.bottomRows(1).transpose(), posHistNeighbors_ );
  if( separationDist < 0.0 ) 
    cGain_[4] = 0.0; // ignore separation dist if posHistNeighbors_ is empty 

  return   cGain_[4] * (-1 * separationDist ) // separtion distance from neighbors
         + cGain_[3] * ( -1 * volGain ); // volumetric gain at the end of the path
         + cGain_[1] * std::get<0>(pathYawHeightErr) // distance between the pose history and candidate path headings
         + cGain_[2] * std::get<1>(pathYawHeightErr); // distance between the pose history and candidate path heights
}

// ***************************************************************************
bool scan_plan::add_paths_to_graph(rrt* tree, std::vector<int>& idLeaves, int idLookaheadLeaf, graph* gph) 
{
  //TODO: Add paths with best pose history distances instead of randomly choosing

  const int nGraphPaths_ = 5; // assuming 10 paths in total are to be added to the graph
  bool success = false;

  Eigen::MatrixXd path;

  std::cout << "Adding non-lookahead paths" << std::endl;
  // all other paths

  int nGraphPaths = 0;
  for(int i=0; i<idLeaves.size(); i++)
  {
    if(nGraphPaths > nGraphPaths_)
      break;

    if(idLeaves[i] == idLookaheadLeaf)
      continue;

    path = tree->get_path(idLeaves[i]);
    if( !pathMan_->path_len_check(path) )
      continue;

    if( gph->add_path(path, true) )
    {
      nGraphPaths++;
      success = true;
    }
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
void scan_plan::goal_cb(const geometry_msgs::PoseStamped& goalMsg)
{
  Eigen::Vector3d goalPtIn(goalMsg.pose.position.x, goalMsg.pose.position.y, goalMsg.pose.position.z);

  if( (status_.goalPt - goalPtIn).lpNorm<1>() < 1e-3 ) // if the goal point changes update the goal
    return;

  status_.goalPt = goalPtIn;

  if( status_.mode != plan_status::MODE::GOALPT ) // if the goal point changes and current mode is GOALPT, replan to the new goal
    return;

  status_.mode = plan_status::MODE::LOCALEXP; // set a different mode than GOALPT so the task_cb retriggers GOALPT mode and fall back to it in case goal point plan fails
  task_cb_str("GOALPT");

  //status_.mode = plan_status::MODE::GOALPT;
  //timerReplan_.setPeriod(ros::Duration(0.2), false); // don't reset the timer because the goal could be coming in at high frequency
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
  //std::cout << "Pose history in size: " << poseHistMsg.poses.size() << std::endl;
  //std::cout << "Pose history current size: " << posHist_.rows() << std::endl;

  if( (isInitialized_ & 0x02) != 0x02 )
  {
    ROS_INFO("%s: Waiting for pose history frame to world transform ...", nh_->getNamespace().c_str());
    try
    {
      poseHistFrameToWorld_ = tfBuffer_.lookupTransform(worldFrameId_, poseHistMsg.header.frame_id, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
	  {
		  ROS_WARN("%s",ex.what());
      return;
	  }
  }

  if(poseHistMsg.poses.size() > posHist_.rows())
  {
    ROS_INFO("%s: Resizing pose history array", nh_->getNamespace().c_str());
    posHist_.conservativeResize(poseHistMsg.poses.size()+25, 3);
  }

  for(int i=0; i<poseHistMsg.poses.size(); i++)
  {
    geometry_msgs::Pose poseHistMsgPose;
    tf2::doTransform (poseHistMsg.poses[i].pose, poseHistMsgPose, poseHistFrameToWorld_);

    posHist_(i,0) = poseHistMsgPose.position.x;
    posHist_(i,1) = poseHistMsgPose.position.y;
    posHist_(i,2) = poseHistMsgPose.position.z;
  }

  posHistSize_ = poseHistMsg.poses.size();

  if( (isInitialized_ & 0x01) == 0x01 ) // only if octomap is initialized
    graph_->update_pos_hist(posHist_.topRows(posHistSize_));

  if( (isInitialized_ & 0x02) != 0x02 && poseHistMsg.poses.size() > 0 )
    isInitialized_ = isInitialized_ | 0x02;
}

// ***************************************************************************
void scan_plan::pos_hist_neighbors_cb(const scan_plan_msgs::PointArrays& pointArrays)
{
  if( pointArrays.arrays.size() != posHistNeighbors_.size() )
    posHistNeighbors_.resize( pointArrays.arrays.size() );

  for( int i=0; i<posHistNeighbors_.size(); i++ )
  {
    posHistNeighbors_[i].conservativeResize( pointArrays.arrays[i].points.size(), 3 );
    for( int j=0; j<posHistNeighbors_[i].rows(); j++ )
      posHistNeighbors_[i].row(j) = Eigen::Vector3d( pointArrays.arrays[i].points[j].x, pointArrays.arrays[i].points[j].y, pointArrays.arrays[i].points[j].z );
  }
}

// ***************************************************************************
void scan_plan::pos_hist_neighbor_cb(const nav_msgs::Path& pathMsg)
{
  // temporary subscriber for testing with path msg type with one neighbor
  if( pathMsg.poses.size() < 1 )
    return;

  scan_plan_msgs::PointArrays pointArraysMsg;
  scan_plan_msgs::PointArray pointArrayMsg;
  pointArrayMsg.points.resize(pathMsg.poses.size());

  for( int i=0; i<pathMsg.poses.size(); i++ )
    pointArrayMsg.points[i] = pathMsg.poses[i].pose.position;
  
  pointArraysMsg.arrays.push_back(pointArrayMsg);

  pos_hist_neighbors_cb(pointArraysMsg);
}

// ***************************************************************************
void scan_plan::update_explored_volume(const double& expVol)
{
  //status_.volExp = expVol;
  //status_.volExpStamp = ros::Time::now();
}
// ***************************************************************************
void scan_plan::publish_plan_status()
{
  if(status_.mode == plan_status::MODE::LOCALEXP)
   publish_plan_status( "Exploring Locally" );

  else if(status_.mode == plan_status::MODE::GLOBALEXP)
    publish_plan_status( "Exploring Globally" );

  else if(status_.mode == plan_status::MODE::REPORT)
    publish_plan_status( "Going home" );

  else if(status_.mode == plan_status::MODE::UNSTUCK)
    publish_plan_status( "Unstucking vehicle" );

  else if(status_.mode == plan_status::MODE::MOVEANDREPLAN)
    publish_plan_status( "Moving and replanning" );
  
  else if(status_.mode == plan_status::MODE::GOALPT)
    publish_plan_status( "Going to gui goal point" );

  else
    publish_plan_status( "Unknown mode" );
}

// ***************************************************************************
void scan_plan::publish_plan_status(const std::string& statusIn)
{
  std_msgs::String planStatusMsg;

  planStatusMsg.data = statusIn;

  planStatusPub_.publish(planStatusMsg);
}

// ***************************************************************************
void scan_plan::publish_can_plan(bool canPlan)
{
  std_msgs::Bool canPlanMsg;
  canPlanMsg.data = canPlan;
  canPlanPub_.publish(canPlanMsg);
}

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete rrtTree_;
  delete graph_;
  delete octMan_;
  delete tfListenerPtr_;
}
