#ifndef OCTOMAPMAN_H
#define OCTOMAPMAN_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>

#include "disp.h"
#include "mapping_sensor.h"

// ***************************************************************************
class octomap_man
{

private: 
  OcTreeT* octTree_ = NULL;
  DynamicEDTOctomap* octDist_ = NULL;

  double maxDistEsdf_;
  bool esdfUnknownAsOccupied_;

  double radRob_; // half of the maximum dimension of the robot
  double robWidth_; 
  double robLength_;

  double maxGroundRoughnessThresh_; // only for "ground" for now 
  double avgGroundRoughnessThresh_; // only for "ground" for now 
  double maxGroundStep_; // only for "ground" for now, determines the maximum step size on the ground that the robot is able to go over

  double successfulProjectionsPercent_;

  std::string vehicleType_; // "air", "ground"
  Eigen::MatrixXd surfCoordsBase_; // list of coordinates spanning the vehicle's surface area looking from above in base frame (useful to project to ground for terrain) 
  double groundPlaneSearchDist_;

  std::vector<mapping_sensor> mapSensors_; // sensors that are building the map, useful to caluclate volumetric gain

  Eigen::Vector3d robPos_;

  uint8_t isInitialized_ = 0x00;
  double baseFrameHeightAboveGround_;
 
public:
  ~octomap_man();
  octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, std::string vehicleType, double robWidth, double robLength, double groundPlaneSearchDist, const std::vector<mapping_sensor>& mapSensors, double baseFrameHeightAboveGround, double successfulProjectionsPercent_, double maxGroundStep, double maxGroundRoughnessThresh, double avgGroundRoughnessThresh);

  double volumetric_gain(const Eigen::Vector3d&);

  bool u_coll(const Eigen::Vector3d&, const Eigen::Vector3d&);
  bool u_coll_ground(const Eigen::Vector3d&, const Eigen::Vector3d&);
  bool u_coll_air(const Eigen::Vector3d&, const Eigen::Vector3d&);

  bool u_coll(const Eigen::Vector4d&);
  bool u_coll_with_update(Eigen::Vector4d&);
  bool u_coll_air(const Eigen::Vector4d&);

  bool cast_pose_down(const Eigen::Vector4d&, Eigen::Vector3d& avgGroundPt);
  bool cast_pose_down(const Eigen::Vector4d&, Eigen::Vector3d& avgGroundPt, double& minElevation, double& maxElevation);
  int cast_ray_down(const Eigen::Vector3d& ptIn, Eigen::Vector3d& groundPt, double& groundRoughness);

  void update_esdf(const Eigen::Vector3d& minBnds, const Eigen::Vector3d& maxBnds);
  void update_octree(OcTreeT* octTree);
  void update_robot_pos(const Eigen::Vector3d&);

  Eigen::Vector3d rotz(const Eigen::Vector3d&, const double&);

  void create_robot_surface(const double&);

  std::string vehicle_type();
  void set_esdf_unknown_as_occupied(const bool& esdfUnkownAsOccupied);
  bool get_esdf_unknown_as_occupied();
};

#endif
