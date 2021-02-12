#ifndef PATHMAN_H
#define PATHMAN_H

#include "disp.h"
#include "ros/ros.h"

// ***************************************************************************
class path_man
{
public:
  double path_len(const Eigen::MatrixXd& path);
  std::pair<double, double> mean_heading_height(const Eigen::MatrixXd& path, const int& nLastPts);
  void publish_path(const Eigen::MatrixXd& eigPath, std::string frameId, const ros::Publisher& pathPub);
  std::pair<double, double> mean_heading_height_err(double yawIn, double heightIn, const Eigen::MatrixXd& pathIn);
};

#endif
