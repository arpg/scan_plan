#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  nh_ = nh;
  wait_for_params(nh_);

  //ocTree_ = new octomap::OcTree(0.05); // what is this resolution

  rrtTree_ = new rrt(rrtNNodes_, scanBnds_[0], scanBnds_[1], rrtRadNear_, rrtDelDist_);

  octSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);
  pathPub_ = nh->advertise<nav_msgs::Path>("path_out", 10);
}

// ***************************************************************************
void scan_plan::wait_for_params(ros::NodeHandle* nh)
{

  //while(!nh->getParam("distance_interval", distInt_));

  rrtNNodes_ = 100;
  scanBnds_[0][0] = -10; scanBnds_[0][1] = -10; scanBnds_[0][2] = -10;
  scanBnds_[1][0] = 10; scanBnds_[1][1] = 10; scanBnds_[1][2] = 10;
  rrtDelDist_ = 0.5;

  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap octmpMsg)
{
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  octTree_ = dynamic_cast<octomap::OcTree*>(tree);

  if( (isInitialized_ & 0x01) == 0x01 )
  {
    init_dist_map();
    isInitialized_ = isInitialized_ | 0x01;
  }

  octDist_->update();
}

// ***************************************************************************
void scan_plan::init_dist_map()
{ 
  //dynamicEDT3D examples

  // TODO:change to parameters
  octomap::point3d min(-1000,-1000,-1000);
  octomap::point3d max(1000,1000,1000);

  bool unknownAsOccupied = false;

  float maxDist = 1.0;

  octDist_ = new DynamicEDTOctomap(maxDist, octTree_, min, max, unknownAsOccupied);

  //This computes the distance map
  //distmap.update(); 
}

// ***************************************************************************
void scan_plan::test_script()
{
// 1. Grow tree, check the nodes, paths, distance of each node from the map
  
  rrtTree_->build(Eigen::Vector3d(0,0,0));

  std::vector<geometry_msgs::Point> checkPos;

  geometry_msgs::Point pt;
  pt.x = 0; pt.y = 0; pt.z = 0;
  checkPos.push_back(pt);
  
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


