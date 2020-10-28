#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  isInitialized_ = 0x00;
  nh_ = nh;
  wait_for_params(nh_);

  //ocTree_ = new octomap::OcTree(0.05); // what is this resolution

  octmpSub_ = nh->subscribe("octomap_in", 1, &scan_plan::octomap_cb, this);

}

// ***************************************************************************
void scan_plan::wait_for_params(ros::NodeHandle* nh)
{
}

// ***************************************************************************
void scan_plan::octomap_cb(const octomap_msgs::Octomap octmpMsg)
{
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octmpMsg);
  ocTree_ = dynamic_cast<octomap::OcTree*>(tree);

  if( (isInitialized_ & 0x01) == 0x01 )
  {
    init_dist_map();
    isInitialized_ = isInitialized_ | 0x01;
  }

  distMap_->update();
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

  distMap_ = new DynamicEDTOctomap(maxDist, ocTree_, min, max, unknownAsOccupied);

  //This computes the distance map
  //distmap.update(); 
}

// ***************************************************************************
scan_plan::~scan_plan()
{
  delete ocTree_;
  delete distMap_;
}


