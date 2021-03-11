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

  canPlanPub_ = nh->advertise<std_msgs::Bool>("can_plan_out", 10);
  planModePub_ = nh->advertise<std_msgs::String>("mode_out", 10);
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
  double rrtSuccRad;
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
  while(!nh_->getParam("succ_rad_rrt", rrtSuccRad));

  ROS_INFO("%s: Setting up local tree ...", nh_->getNamespace().c_str());
  rrtTree_ = new rrt(rrtNNodes, minBnds, maxBnds, rrtRadNear, rrtDelDist, radRob_, rrtSuccRad, rrtFailItr, octMan_);  
}

// ***************************************************************************
void scan_plan::setup_graph()
{
  // prereqs setup_pose, setup_octomap

  ROS_INFO("%s: Waiting for graph params ...", nh_->getNamespace().c_str());
  double graphRadNear, minVolGain, graphRadNearest, minManDistFrontier;
  std::vector<double> homePos, entranceMin, entranceMax;
 
  while(!nh_->getParam("near_radius_graph", graphRadNear));
  while(!nh_->getParam("nearest_radius_graph", graphRadNearest));
  while(!nh_->getParam("min_vol_gain_frontier", minVolGain)); // used for local/global switching and removing frontiers, (m^3)
  while(!nh_->getParam("home_position", homePos)); // must be collision-free
  while(!nh_->getParam("min_man_dist_frontiers", minManDistFrontier));
  while(!nh_->getParam("entrance_min_bnds", entranceMin));
  while(!nh_->getParam("entrance_max_bnds", entranceMax));
 
  
  ROS_INFO("%s: Setting up graph ...", nh_->getNamespace().c_str());
  homePos_(0) = homePos[0];
  homePos_(1) = homePos[1];
  homePos_(2) = homePos[2];

  graph_ = new graph(homePos_, graphRadNear, graphRadNearest, radRob_, minVolGain, worldFrameId_, octMan_, minManDistFrontier, entranceMin, entranceMax);
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
  while(!nh_->getParam("vol_gain_monitor_dur_mode_switch", volGainMonitorDur_));
  while(!nh_->getParam("min_vol_gain_local_plan", minVolGainLocalPlan_));
  while(!nh_->getParam("succ_rad_end_of_path", endOfPathSuccRad_));

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
void scan_plan::task_cb(const std_msgs::String& taskMsg)
{
  if( (taskMsg.data == "report" || taskMsg.data == "Report" || taskMsg.data == "home" || taskMsg.data == "Home") && status_.mode != plan_status::MODE::REPORT )
  {
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for computation

    Eigen::MatrixXd minCstPath = plan_home(); // try planning home
    if(minCstPath > 0) // path found
    {
      status_.mode = plan_status::MODE::REPORT;
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
      publish_can_plan(false);
    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise publish older path and don't change mode
  }

  if( (taskMsg.data == "explore" || taskMsg.data == "Explore") && (status_.mode != plan_status::MODE::LOCALEXP || status_.mode != plan_status::MODE::GLOBALEXP) )
  {
    if(status_.mode = plan_status::MODE::REPORT) // if transitioning from report mode, then flip the path so the robot goes back to where it came from
    {
      status_.mode = plan_status::MODE::GLOBALEXP;
      minCstPath_.colwise().reverseInPlace();
      path_man::publish_path(minCstPath_, worldFrameId_, pathPub_);
    }
    else 
    {
      status_.mode = plan_status::MODE::LOCALEXP;
      Eigen::MatrixXd minCstPath = plan_locally();

      if(minCstPath > 0)  // if local path is found publish it, otherwise leave in local exp mode so the timer retries and figures it out
      {
        minCstPath_ = minCstPath;
        path_man::publish_path(minCstPath_, worldFrameId_, pathPub_);
      }
      else
      {
        minCstPath_ = Eigen::MatrixXd(0,0); // because timer retries at the end-of-path
        path_man::publish_empty_path(worldFrameId_, pathPub_); 
      }
    }

  }

  //TODO: right now it only replans to the goal point if the mode changes but what if it's already in goal point mode and the goal point changes 
  if( (taskMsg.data == "guiCommand" || taskMsg.data == "gui_command" || taskMsg.data == "guiCmd" || taskMsg.data == "gui_cmd") && status_.mode != plan_status::MODE::GOALPT )
  {  
    path_man::publish_empty_path(worldFrameId_, pathPub_); // stop the vehicle for computation
    Eigen::MatrixXd minCstPath = plan_to_point(status_.goalPt);

    if(minCstPath > 0)
    {
      status_.mode = plan_status::MODE::GOALPT;
      minCstPath_ = minCstPath;
      publish_can_plan(true);
    }
    else
      publish_can_plan(false);
    path_man::publish_path(minCstPath_, worldFrameId_, pathPub_); // publish path if found, otherwise publish older path and don't change mode
  }
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{
  //delete octTree_;

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octomap::OcTree* octTree = dynamic_cast<octomap::OcTree*>(tree);

  //publish_plan_mode();

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_octree(octTree);

  octMan_->update_esdf(localBndsMin_+robPos, localBndsMax_+robPos);
  octMan_->update_robot_pos(robPos);

  //std::cout << "Checking for End-of-Path" << std::endl;
  //if( (minCstPath_.rows() > 0) && ( (robPos.transpose()-minCstPath_.bottomRows(1)).norm() < endOfPathSuccRad_ ) )
  //{
  //  std::cout << "End-of-Path: " << (robPos.transpose()-minCstPath_.bottomRows(1)).norm() << std::endl;
  //  status_.mode = plan_status::MODE::LOCALEXP;
   // timerReplan_.setPeriod(ros::Duration(0.2), true);
  //}

  //std::cout << "Validating path" << std::endl;
  //status_.changeDetected_ = !( pathMan_->validate_path(minCstPath_) );
  //pathMan_->publish_path(minCstPath_, worldFrameId_, pathPub_);
  //if(status_.changeDetected_ && status_.mode != plan_status::MODE::LOCALEXP)
  //{
  //  status_.mode = plan_status::MODE::LOCALEXP;
  //  timerReplan_.setPeriod(ros::Duration(0.5), true);
  //}

  //if( status_.mode == plan_status::MODE::LOCALEXP && (ros::Time::now() - status_.volExpStamp).toSec() > volGainMonitorDur_ ) 
  //{
    // if locally exploring, update every volGainMonitorDur_ sec and check for a drop

    //double volExp = octTree->getNumLeafNodes() * pow(octmpMsg.resolution,3);
    //std::cout << "Updating volumetric gain, locally exploring: " << (volExp - status_.volExp) << std::endl;

    //if( minCstPath_.rows() > 0 && octMan_->volumetric_gain(minCstPath_.bottomRows(1).transpose()) < 5.0)
    //{
    //  status_.mode = plan_status::MODE::GLOBALEXP;
    //  timerReplan_.setPeriod(ros::Duration(0.2), true);
    //}

    //update_explored_volume(volExp);
  //}
  //else if(status_.mode != plan_status::MODE::LOCALEXP) // if not locally exploring, update volumetric gain frequently to start monitoring when it does start locally exploring
  //{
   // std::cout << "Updating volumetric gain, not locally exploring" << std::endl;
   // update_explored_volume(octTree->getNumLeafNodes() * pow(octmpMsg.resolution,3));
  //}

  if( (isInitialized_ & 0x01) != 0x01 )
  {
    status_.changeDetected_ = false;
    update_explored_volume(octTree->getNumLeafNodes() * pow(octmpMsg.resolution,3));
    isInitialized_ = isInitialized_ | 0x01;
  }
}

// ***************************************************************************
bool scan_plan::end_of_path()
{
  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  if(minCstPath_.rows() < 1) // if no valid path
    return true;
  if( (robPos.transpose()-minCstPath_.bottomRows(1)).norm() < endOfPathSuccRad_ ) // if the robot is near the end point of the path
    return true;

  return false;
}

// ***************************************************************************
void scan_plan::timer_replan_cb(const ros::TimerEvent&) // running at a fast rate
{
  if( (isInitialized_ & 0x03) != 0x03 )
    return;

  ros::Time timeS = ros::Time::now();
  //update_base_to_world();
  //Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );

  //TODO: Add goal point to graph while planning to a goal point because the mode switches to local so if the goal point is far from graph, connectivity might be a prob
  //TODO: Add vehicle not moving and map change detections logic

  if(status_.mode == plan_status::MODE::REPORT && end_of_path())
  {
    status_.mode == plan_status::MODE::GLOBALEXP;
    Eigen::MatrixXd minCstPath = plan_globally(); // vehicle already stopped for computation due to EOP
    if(minCstPath.rows() > 0) // global path found
      minCstPath_ = minCstPath;
  }
  else if(status_.mode == plan_status::MODE::GOALPT && end_of_path())
  {
    status_.mode == plan_status::MODE::LOCALEXP;
    Eigen::MatrixXd minCstPath = plan_locally();
    if(minCstPath.rows() > 0) // local path found
      minCstPath_ = minCstPath;
  }
  else if(status_.mode == plan_status::MODE::GLOBALEXP && end_of_path())
  {
    status_.mode == plan_status::MODE::LOCALEXP;
    Eigen::MatrixXd minCstPath = plan_locally();
    if(minCstPath.rows() > 0) // local path found
      minCstPath_ = minCstPath;
  }
  else if(status_.mode == plan_status::MODE::LOCALEXP && end_of_path())
  {
    status_.mode == plan_status::MODE::LOCALEXP;
    Eigen::MatrixXd minCstPath = plan_locally();

    if(minCstPath.rows() > 0 && octMan_->volumetric_gain(minCstPath.bottomRows(1).transpose()) >= 5.0) // local path found and vol gain is good enough
      minCstPath_ = minCstPath;

    else if(minCstPath.rows() > 0 && octMan_->volumetric_gain(minCstPath.bottomRows(1).transpose()) < 5.0) // local path found but vol gain is low
    {
      status_.mode == plan_status::MODE::GLOBALEXP;
      Eigen::MatrixXd minCstPath = plan_globally(); // vehicle already stopped for computation due to EOP
      if(minCstPath.rows() > 0) // global path found
        minCstPath_ = minCstPath;
    }    
  }

  //TODO: Generate graph diconnect warning if no path is successfully added to the graph
  // Only add node to the graph if there is no existing node closer than radRob in graph lib, return success in that case since the graph is likely to be connected fine
  // update older node if within robot radius with updated terrain and collision information, the latter may mean taking nodes out of the graph
  // consider immidiate replan if a path is not found by-passing timer

  //TODO: Consider including a check, update path if end-of-path is reached, and increase the replan time

  ///graph_->update_frontiers_vol_gain();
   
  ros::Duration timeE = ros::Time::now() - timeS;

  //rrtTree_->publish_viz(vizPub_,worldFrameId_,idLeaves);

  std::cout << "Publishing plan mode" << std::endl;
  publish_plan_mode();

  std::cout << "Publishing lookahead path" << std::endl;
  path_man::publish_path(minCstPath_, worldFrameId_, pathPub_);

  std::cout << "Publishing frontiers" << std::endl; 
  graph_->publish_frontiers(frontiersPub_); 

  std::cout << "Publishing visualizations" << std::endl;
  graph_->publish_viz(vizPub_);

  std_msgs::Float64 computeTimeMsg;
  computeTimeMsg.data = timeE.toSec();
  compTimePub_.publish(computeTimeMsg);
  
  std::cout << "Replan Compute Time = " << computeTimeMsg.data << " sec" << std::endl;
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_home()
{
  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  std::cout << "Planning to graph" << std::endl;
  VertexDescriptor vertD1;
  Eigen::MatrixXd minCstPath1 = plan_to_graph(robPos, vertD1);
  if(minCstPath1.rows() < 2)
    return posHist_.topRows(posHistSize_).colwise().reverse();

  std::cout << "Planning vertex to home vertex" << std::endl;
  Eigen::MatrixXd minCstPath2 = graph_->plan_home(vertD1);
  if(minCstPath2.rows() < 1)
    return posHist_.topRows(posHistSize_).colwise().reverse();

  std::cout << "Appending paths together" << std::endl;
  Eigen::MatrixXd minCstPath(minCstPath1.rows()+minCstPath2.rows(), 3);
  minCstPath << minCstPath1, minCstPath2;
  return minCstPath;
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_to_point(const Eigen::Vector3d& goalPos)
{
  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  std::cout << "Checking if goal and robot positions are different" << std::endl;
  if(goalPos == robPos) // if goal and robot pos is same
    return Eigen::MatrixXd(0,0);

  std::cout << "Checking if goal and robot positions can be connected via a straight line" << std::endl;
  if(!octMan_->u_coll(robPos, goalPos)) // try to connect via straight line
  {
    Eigen::MatrixXd minCstPath(2,3);

    minCstPath.row(0) = robPos;
    minCstPath.row(1) = goalPos;
    return minCstPath;
  }

  std::cout << "Checking if goal and robot positions are within local bounds of each other" << std::endl;
  if( ((goalPos-robPos-localBndsMin_).array() > 0).all() && ((goalPos-robPos-localBndsMax_).array() < 0).all() ) // if goal is within local bounds of robot, try rrt
  {
    rrtTree_->set_bounds( geofence_saturation(localBndsMin_+robPos), geofence_saturation(localBndsMax_+robPos) );
    int idLeaf = rrtTree_->build(robPos, goalPos);

    if(idLeaf != 0) // if a path is found
      return rrtTree_->get_path(idLeaf);
  }

  std::cout << "Planning to graph" << std::endl;
  VertexDescriptor vertD1;
  Eigen::MatrixXd minCstPath1 = plan_to_graph(robPos, vertD1);
  if(minCstPath1.rows() < 2)
    return Eigen::MatrixXd(0,0);

  std::cout << "Planning from graph" << std::endl;
  octMan_->update_esdf(localBndsMin_+goalPos, localBndsMax_+goalPos); // getting the esdf around the goal rather than default robot position
  VertexDescriptor vertD2;
  Eigen::MatrixXd minCstPath2 = plan_from_graph(goalPos, vertD2);
  if(minCstPath2.rows() < 2)
    return Eigen::MatrixXd(0,0);
  octMan_->update_esdf(localBndsMin_+robPos, localBndsMax_+robPos);  // getting the esdf back around default robot position

  std::cout << "Planning vertex to vertex" << std::endl;
  Eigen::MatrixXd minCstPathG = graph_->plan_shortest_path(vertD1, vertD2);
  if(minCstPathG.rows() < 2)
    return Eigen::MatrixXd(0,0);

  std::cout << "Appending paths together" << std::endl;
  Eigen::MatrixXd minCstPath(minCstPath1.rows()+minCstPathG.rows()+minCstPath2.rows(), 3);
  minCstPath << minCstPath1, minCstPathG, minCstPath2;
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

    rrtTree_->set_bounds( geofence_saturation(localBndsMin_+fromPos), geofence_saturation(localBndsMax_+fromPos) );
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
Eigen::MatrixXd scan_plan::plan_globally() 
{
  graph_->update_frontiers_vol_gain();
  frontier front = graph_->get_best_frontier();

  if(front.volGain <= 0) // if there is no frontier
    return Eigen::MatrixXd(0,0);

  //return plan_path_to_point(graph_->get_pos(front.vertDesc));

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

  std::cout << "Planning to graph" << std::endl;
  VertexDescriptor vertD1;
  Eigen::MatrixXd minCstPath1 = plan_to_graph(robPos, vertD1);
  if(minCstPath1.rows() < 2)
    return Eigen::MatrixXd(0,0);

  std::cout << "Path to graph non-empty" << std::endl;
  if(vertD1 == front.vertDesc)
    return minCstPath1;
  
  std::cout << "Source and target vertices validity? " << graph_->is_valid(vertD1) << ", " << graph_->is_valid(front.vertDesc) << std::endl;
  Eigen::MatrixXd minCstPath2 = graph_->plan_shortest_path(vertD1, front.vertDesc);

  std::cout << "Planning over graph failed" << std::endl;
  if(minCstPath2.rows() < 1)
    return Eigen::MatrixXd(0,0);

  std::cout << "Appending paths together" << std::endl;
  Eigen::MatrixXd minCstPath(minCstPath1.rows()+minCstPath2.rows(), 3);
  minCstPath << minCstPath1, minCstPath2;
  return minCstPath;
}

// ***************************************************************************
Eigen::MatrixXd scan_plan::plan_locally() 
{
  // returns rrt lookahead path, ids of all leaves and of the lookahead path leaf
  // get_path function in rrt can be used to get paths for a leaf id

  Eigen::MatrixXd minCstPath(0,0);
  std::vector<int> idLeaves;
  int idPathLeaf;

  update_base_to_world();
  Eigen::Vector3d robPos( transform_to_eigen_pos(baseToWorld_) );
  octMan_->update_robot_pos(robPos);

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
    double pathCst = cGain_[0] * ( -1 * path_man::path_to_path_dist(path,posHist_.topRows(posHistSize_)) ) // distance between candidate path and pose history path
                   + cGain_[1] * std::get<0>(pathYawHeightErr) // distance between the pose history and candidate path headings
                   + cGain_[2] * std::get<1>(pathYawHeightErr); // distance between the pose history and candidate path heights

    //std::cout << "(" << path_man::path_to_path_dist(path,posHist_.topRows(posHistSize_)) << ", " << std::get<0>(pathYawHeightErr) << ", " << std::get<1>(pathYawHeightErr) << "), ";

    if( minCst > pathCst && pathMan_->path_len_check(path) && !graph_->is_entrance(path.bottomRows(1).transpose()) ) // minimum path length condition
    {
      minCst = pathCst;
      minCstPath = path;
      idPathLeaf = idLeaves[i];
    }
  } 

  //std::cout << std::endl;

  if( minCstPath.rows() > 1 )
  {
    std::cout << "Adding paths to the graph" << std::endl;
    bool success = add_paths_to_graph(rrtTree_, idLeaves, idPathLeaf, graph_);
    if(!success)
      ROS_WARN("%s: Graph connection unsuccessful", nh_->getNamespace().c_str());
  }

  return minCstPath;
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
void scan_plan::goal_cb(const geometry_msgs::PointStamped& goalMsg)
{
  status_.goalPt(0) = goalMsg.point.x;
  status_.goalPt(1) = goalMsg.point.y;
  status_.goalPt(2) = goalMsg.point.z;

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

  if(poseHistMsg.poses.size() > posHist_.rows())
  {
    ROS_INFO("Resizing pose history array");
    posHist_.conservativeResize(poseHistMsg.poses.size()+25, 3);
  }

  for(int i=0; i<poseHistMsg.poses.size(); i++)
  {
    posHist_(i,0) = poseHistMsg.poses[i].pose.position.x;
    posHist_(i,1) = poseHistMsg.poses[i].pose.position.y;
    posHist_(i,2) = poseHistMsg.poses[i].pose.position.z;
  }

  posHistSize_ = poseHistMsg.poses.size();
  if( (isInitialized_ & 0x02) != 0x02 && poseHistMsg.poses.size() > 0 )
    isInitialized_ = isInitialized_ | 0x02;
}
// ***************************************************************************
void scan_plan::update_explored_volume(const double& expVol)
{
  status_.volExp = expVol;
  status_.volExpStamp = ros::Time::now();
}
// ***************************************************************************
void scan_plan::publish_plan_mode()
{
  std_msgs::String planModeMsg;
  
  if(status_.mode == plan_status::MODE::LOCALEXP)
    planModeMsg.data = "Exploring Locally";

  else if(status_.mode == plan_status::MODE::GLOBALEXP)
    planModeMsg.data = "Exploring Globally";

  else if(status_.mode == plan_status::MODE::REPORT)
    planModeMsg.data = "Going home";
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
