#ifndef PATHMAN_H
#define PATHMAN_H

#include "disp.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"

#include <eigen3/Eigen/Core>
#include <math.h>

// ***************************************************************************
class path_man
{
public:
  static double path_len(const Eigen::MatrixXd& path);
  static std::pair<double, double> mean_heading_height(const Eigen::MatrixXd& path, const int& nLastPts);
  static void publish_path(const Eigen::MatrixXd& eigPath, std::string frameId, const ros::Publisher& pathPub);
  static std::pair<double, double> mean_heading_height_err(double yawIn, double heightIn, const Eigen::MatrixXd& pathIn);
  static double path_to_path_dist(const Eigen::MatrixXd& path1, const Eigen::MatrixXd& path2);
  static double point_to_path_dist(const Eigen::Vector3d& ptIn, const Eigen::MatrixXd& pathIn);
};

#endif
