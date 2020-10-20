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


public:
  scan_plan(ros::NodeHandle*);
  void wait_for_params(ros::NodeHandle*);
  //double dist_polytope(geometry_msgs::Pose, geometry_msgs::Pose); 
};

#endif
