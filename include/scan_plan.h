#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"

// ***************************************************************************
class ph_cam_class
{

private: 
  std::vector<double> camInfoP_(9);
  std::vector<double> camRes_(2); //width,height
  double maxDepth_;
  geometry_msgs::TransformStamped camToBase_;
  std::vector<geometry_msgs::Point> polytope_;

public:
  geometry_msgs::Point point2_to_point3(geometry_msgs::Point, bool);
  void compute_polytope();
  std::vector<geometry_msgs::Point> get_polytope();
  ph_cam_class(std::vector<double>, std::vector<double>, double);
};

// ***************************************************************************
class scan_plan_class
{

private:

  ros::NodeHandle* nh_;
  geometry_msgs::PoseArray poseHist_;
  std::vector<ph_cam_class> phCams_;


public:
  void wait_for_params(ros::NodeHandle*);
 
};

#endif
