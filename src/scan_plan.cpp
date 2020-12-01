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
    geometry_msgs::TransformStamped worldToBase = tfBuffer_.lookupTransform(worldFrameId_, baseFrameId_, ros::Time(0));

  ph_cam phCam(camInfoP_.row(0).data(), camRes_.row(0).data(), maxDepth_[0], camToBase_[0]);

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);
  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);

  rrtTree_ = new rrt(rrtNNodes_, scanBnds_[0], scanBnds_[1], rrtRadNear_, rrtDelDist_, radRob_, rrtFailItr_, octDist_);

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

  std::vector<double> maxDepth;
  while(!nh->getParam("max_depth", maxDepth));
  int nCams = maxDepth.size();

  std::vector<double> camInfoP;
  while(!nh->getParam("camera_info_p", camInfoP));
  camInfoP_.resize(nCams,9);
  for(int i=0; i<nCams; i++)
    for(int j=0; j<9; j++)
      camInfoP_(i,j) = camInfoP[i*9+j];

  std::vector<double> camRes;
  while(!nh->getParam("camera_resolution", camRes));
  camRes_.resize(nCams,2);
  for(int i=0; i<nCams; i++)
    for(int j=0; j<2; j++)
      camRes_(i,j) = camRes[i*2+j];
  
  while(!nh->getParam("base_frame_id", baseFrameId_));
  while(!nh->getParam("world_frame_id", worldFrameId_));

  std::vector<std::string> camFrameId;
  while(!nh->getParam("cam_frame_id", camFrameId));

  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());

  for(int i=0; i<maxDepth.size(); i++)
  {
    while( !tfBuffer_.canTransform(baseFrameId_, camFrameId[i], ros::Time(0)) );
      camToBase_.push_back( tfBuffer_.lookupTransform(baseFrameId_, camFrameId[i], ros::Time(0)) );
  }
  
  ROS_INFO("%s: Cam to base transforms received", nh->getNamespace().c_str());
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
    isInitialized_ = isInitialized_ | 0x01;
  }
  
  init_dist_map();
  
  test_script();

  ros::Duration timeE = ros::Time::now() - timeS;
  std::cout << "Time Elapsed = " << timeE.toSec() << " sec" << std::endl;
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
void scan_plan::path_cost(Eigen::MatrixXd& path)
{
  double pathLen = 0;
}

// ***************************************************************************
// ***************************************************************************
// ***************************************************************************
// ***************************************************************************

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete octTree_;
  delete octDist_;
  delete rrtTree_;
}
