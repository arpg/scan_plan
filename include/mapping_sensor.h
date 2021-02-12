#ifndef MAPPINGSENSOR_H
#define MAPPINGSENSOR_H

#include "disp.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "dynamicEDT3D/dynamicEDTOctomap.h"
#include "octomap_msgs/Octomap.h"

#include "ros/ros.h"

#include <eigen3/Eigen/Core>

// ***************************************************************************
class mapping_sensor
{

private: 
  double fovH_; // horizontal degrees e.g., 360 for OS1
  double fovV_; // vertical degrees e.g., 45 for OS1

  double resH_; // no. of horizontal measurements e.g., 1024 for OS1
  double resV_; // no. of vertical measurements e.g., 64 for OS1

  double range_; // max sensor range in meters / mapping range

  Eigen::MatrixXf multiRayEndPts_; // in body frame, populated at initialization
  geometry_msgs::TransformStamped sensorToBase_;

public:
  mapping_sensor(double fovH, double fovV, double resH, double resV, double range, geometry_msgs::TransformStamped sensorToBody);
  
  int n_ray_unseen_voxels(const Eigen::Vector3d& startPt, const Eigen::Vector3d& endPt, octomap::OcTree* octTree);
  double volumetric_gain(octomap::OcTree* octTree, const geometry_msgs::TransformStamped& baseToWorld);c
  double volumetric_gain(octomap::OcTree* octTree, const Eigen::Vector3d& basePos);
  Eigen::Vector3d transform_point(const Eigen::Vector3d& eigPt, const geometry_msgs::TransformStamped& transform);
  void populate_multiray_endpts();
};

#endif
