#ifndef OCTOMAPMAN_H
#define OCTOMAPMAN_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <random>
#include <chrono>
#include <forward_list>
#include "disp.h"

#include "dynamicEDT3D/dynamicEDTOctomap.h"
#include "matplotlib-cpp/matplotlibcpp.h"

#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"

// ***************************************************************************
class octomap_man
{

private: 
  octomap::OcTree* octTree_ = NULL;
  DynamicEDTOctomap* octDist_ = NULL;

  double maxDistEsdf_;
  bool esdfUnknownAsOccupied_; // only for "air" for now
  double radRob_; // radius if "air", half-width if "ground"
  double maxGroundRoughness_; // only for "ground" for now 

  std::string vehicleType_; // "air", "ground"
  Eigen::MatrixXd surfCoordsBase_; // list of coordinates spanning the vehicle's surface area looking from above in base frame (useful to project to ground for terrain) 
  double groundPlaneSearchDist_;

  std::vector<mapping_sensor> mapSensors_; // sensors that are building the map, useful to caluclate volumetric gain
 
public:
  ~octomap_man();
  octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied);
};

#endif
