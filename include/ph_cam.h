#ifndef PHCAM_H
#define PHCAM_H

#include "openGJK.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ***************************************************************************
class ph_cam
{

private: 
  double camInfoP_[9];
  int camRes_[2]; //width,height
  double maxDepth_;
  geometry_msgs::TransformStamped camToBase_;
  std::vector<geometry_msgs::Point> polytope_;

public:
  ph_cam(double*, double*, double, geometry_msgs::TransformStamped);
  geometry_msgs::Point point2_to_point3(geometry_msgs::Point, bool);
  void compute_polytope();
  std::vector<geometry_msgs::Point> get_polytope();
  std::vector<geometry_msgs::Point> transform(geometry_msgs::TransformStamped);
  double distance(std::vector<geometry_msgs::Point>, std::vector<geometry_msgs::Point>);
};

#endif
