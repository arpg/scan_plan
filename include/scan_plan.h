#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
#include "ph_cam.h"
#include "rrt.h"
#include "nav_msgs/Path.h"
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

  double scanBnds_[2][3]; // [min,max] x [x,y,z]

  ros::Subscriber octSub_;
  ros::Publisher pathPub_;

  rrt* rrtTree_;
  octomap::OcTree* octTree_ = NULL;
  DynamicEDTOctomap* octDist_ = NULL;

  uint8_t isInitialized_;

  int rrtNNodes_;
  double rrtRadNear_;
  double rrtDelDist_;

  double radRob_; // max dist to obs to declare collision

  int rrtFailItr_;

public:
  scan_plan(ros::NodeHandle*);
  ~scan_plan();

  void wait_for_params(ros::NodeHandle*);
  void build_tree(double*, double*, double);
  void init_dist_map();

  void octomap_cb(const octomap_msgs::Octomap&);
  void test_script();
};

#endif
