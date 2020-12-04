#ifndef PHCAM_H
#define PHCAM_H

extern "C" {
#include "openGJK.h"
}

#include "disp.h"
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
  double info_[10]; // width, height, depth, discInt[3], fx,cx,fy,cy
  std::vector<geometry_msgs::Point> polytope_;
  double time_;

  void set_info(double camInfoK[9], double camRes[3], double discInt[3]);
  double* get_cam_info();
  double* get_cam_res();
  double* get_disc_int();
  geometry_msgs::Point point2_to_point3(geometry_msgs::Point pointIn, bool direction);
  void compute_polytope();
  void set_max_depth(double);

public:
  ph_cam(double[9], double[3], double[3]);
  
  std::vector<geometry_msgs::Point> get_polytope();
  void transform(geometry_msgs::TransformStamped);
  double distance(ph_cam&);
  void shrink(std::vector<bool> uCollVec);
  void print_polytope();
  double time();
  void set_time(double);
  void set_polytope(std::vector<geometry_msgs::Point>);
};

#endif
