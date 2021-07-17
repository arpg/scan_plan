#ifndef PATHMAN_H
#define PATHMAN_H

#include "disp.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "octomap_man.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <math.h>

// ***************************************************************************
class path_man
{
private:
  double minPathLen_;
  octomap_man* octMan_;
  
public:
  path_man(const double&, octomap_man*);
  bool validate_path(Eigen::MatrixXd& path, const Eigen::Vector3d& minBnd, const Eigen::Vector3d& maxBnd);
  bool validate_path_without_mod(const Eigen::MatrixXd& path, const Eigen::Vector3d& minBnd, const Eigen::Vector3d& maxBnd);
  bool path_len_check(const Eigen::MatrixXd& path);


  static double path_len(const Eigen::MatrixXd& path);
  static std::pair<double, double> mean_heading_height(const Eigen::MatrixXd& path, const int& nLastPts);
  static void publish_path(const Eigen::MatrixXd& eigPath, const std::string& frameId, const ros::Publisher& pathPub);
  static void publish_empty_path(const std::string& frameId, const ros::Publisher& pathPub);
  static std::pair<double, double> mean_heading_height_err(double yawIn, double heightIn, const Eigen::MatrixXd& pathIn);
  static double path_to_path_dist(const Eigen::MatrixXd& path1, const Eigen::MatrixXd& path2);
  static double point_to_path_dist(const Eigen::Vector3d& ptIn, const Eigen::MatrixXd& pathIn);
  static double point_to_line_dist(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&);
  static bool in_bounds(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&);
  static bool are_equal(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

  Eigen::MatrixXd interpolate(const Eigen::MatrixXd& path, const double& maxSpacePts);
  bool is_staircase(const Eigen::MatrixXd& path, const double& minStepHeight, const int& minNSteps);
};

#endif
