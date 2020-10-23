#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
#include "ph_cam.h"
#include "geometry_msgs/PoseArray.h"

// ***************************************************************************
class scan_plan
{

private:

  ros::NodeHandle* nh_;
  geometry_msgs::PoseArray poseHist_;
  std::vector<ph_cam> phCams_;
  std::vector<rrt_node> nodeLst_;

  int nNodes_;
  double scanBounds_[2][3]; // [min,max] x [x,y,z]


public:
  scan_plan(ros::NodeHandle*);
  void wait_for_params(ros::NodeHandle*);
  void build_tree(double*, double*, double);
};

#endif
