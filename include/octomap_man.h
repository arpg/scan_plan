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
  bool esdfUnknownAsOccupied_;
  double radRob_; // radius if "air", half-width if "ground"

  std::string vehicleType_; // "air", "ground"
 
public:
  ~octomap_man();
  octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied);
};

#endif
