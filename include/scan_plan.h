#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
#include "ph_cam.h"
#include "rrt.h"
#include "geometry_msgs/PoseArray.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"
//#include "dynamicEDT3D/dynamicEDT3D.h"

// ***************************************************************************
class scan_plan
{

private:

  ros::NodeHandle* nh_;
  geometry_msgs::PoseArray poseHist_;
  std::vector<ph_cam> phCams_;
  //rrt tree_;

  int nNodes_;
  double scanBounds_[2][3]; // [min,max] x [x,y,z]

  ros::Subscriber octmpSub_;

  octomap::OcTree* ocTree_ = NULL;
  DynamicEDTOctomap* distMap_ = NULL;

  uint8_t isInitialized_;

public:
  scan_plan(ros::NodeHandle*);
  ~scan_plan();

  void wait_for_params(ros::NodeHandle*);
  void build_tree(double*, double*, double);
  void init_dist_map();

  void octomap_cb(const octomap_msgs::Octomap);
};

#endif
