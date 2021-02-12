#ifndef OCTOMAPMAN_H
#define OCTOMAPMAN_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>

#include "disp.h"
#include "mapping_sensor.h"

#include "dynamicEDT3D/dynamicEDTOctomap.h"

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
  double maxGroundStep_; // only for "ground" for now, determines the maximum step size on the ground that the robot is able to go over

  std::string vehicleType_; // "air", "ground"
  Eigen::MatrixXd surfCoordsBase_; // list of coordinates spanning the vehicle's surface area looking from above in base frame (useful to project to ground for terrain) 
  double groundPlaneSearchDist_;

  std::vector<mapping_sensor> mapSensors_; // sensors that are building the map, useful to caluclate volumetric gain

  bool isInitialized_ = false;
 
public:
  ~octomap_man();
  octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, std::string vehicleType, double radRob, double maxGroundRoughness, double maxGroundStep, const std::vector<mapping_sensor>& mapSensors);

  double volumetric_gain(const Eigen::Vector3d& basePos);
  bool u_coll(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
  bool u_coll_ground(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
  bool u_coll_air(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
  bool u_coll(const Eigen::Vector3d& pos);
  bool u_coll(const Eigen::Vector3d& pos, double& groundRoughness, Eigen::Vector3d& groundPt);
  bool u_coll_air(const Eigen::Vector3d& pos);
  bool u_coll_ground(const Eigen::Vector3d& pos, double& roughness, Eigen::Vector3d& groundPt);
  bool project_point_to_ground(const Eigen::Vector3d& pos, double& roughness, Eigen::Vector3d& groundPt);
  void update_esdf(const Eigen::Vector3d& minBnds, const Eigen::Vector3d& maxBnds);
  void update_octree(octomap::OcTree* octTree);
};

#endif
