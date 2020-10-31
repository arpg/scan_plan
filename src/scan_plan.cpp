#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  nh_ = nh;
  wait_for_params(nh_);

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);
  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);

  ROS_INFO("%s: Waiting for the input map ...", nh->getNamespace().c_str());
  while( (isInitialized_ & 0x01) != 0x01 )
	  ros::spinOnce();

  octTree_ = new octomap::OcTree(0.05);
  rrtTree_ = new rrt(rrtNNodes_, scanBnds_[0], scanBnds_[1], rrtRadNear_, rrtDelDist_, radRob_, octDist_);
}

// ***************************************************************************
void scan_plan::wait_for_params(ros::NodeHandle* nh)
{

  //while(!nh->getParam("distance_interval", distInt_));

  rrtNNodes_ = 100;
  scanBnds_[0][0] = -10; scanBnds_[0][1] = -10; scanBnds_[0][2] = -10;
  scanBnds_[1][0] = 10; scanBnds_[1][1] = 10; scanBnds_[1][2] = 10;
  rrtDelDist_ = 0.5;
  radRob_ = 0.1;

  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap& octmpMsg)
{

  ros::Time timeS = ros::Time::now();

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
}

// ***************************************************************************
void scan_plan::test_script()
{
// 1. Grow tree, check the nodes, paths, distance of each node from the map
  
  rrtTree_->build(Eigen::Vector3d(0,0,0));

  std::cout << "Distance to (0,0,0) is " << octDist_->getDistance (octomap::point3d(0,0,0)) << std::endl;
  std::cout << "Distance to (2,0,1) is " << octDist_->getDistance (octomap::point3d(2,0,1)) << std::endl;

  //rrtTree_->u_coll_octomap(Eigen::Vector3d(0,0,0));
  
}

// ***************************************************************************
// ***************************************************************************
// ***************************************************************************
// ***************************************************************************
// ***************************************************************************

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete octTree_;
  delete octDist_;
}
