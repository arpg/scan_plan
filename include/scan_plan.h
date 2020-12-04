#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
#include "ph_cam.h"
#include "rrt.h"
#include "nav_msgs/Path.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"
#include "tf2_ros/transform_listener.h"
//#include "dynamicEDT3D/dynamicEDT3D.h"

// ***************************************************************************
class scan_plan
{

private:

  ros::NodeHandle* nh_;
  geometry_msgs::PoseArray poseHist_;
  int nCams_;
  std::vector<ph_cam> phCamsWorld_; 
  std::vector<ph_cam> phCamsBase_;
  std::vector<ph_cam> phCamsOpt_;
  double timeIntPhCam_ = 3;

  double scanBnds_[2][3]; // [min,max] x [x,y,z]

  ros::Subscriber octSub_;

  ros::Publisher pathPub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;

  std::string baseFrameId_;
  std::string worldFrameId_;

  std::vector<geometry_msgs::TransformStamped> camToBase_;

  Eigen::MatrixXd camInfoK_;
  Eigen::MatrixXd camRes_;
  Eigen::MatrixXd discInt_;

  rrt* rrtTree_;
  octomap::OcTree* octTree_ = NULL;
  DynamicEDTOctomap* octDist_ = NULL;

  uint8_t isInitialized_;

  int rrtNNodes_;
  double rrtRadNear_;
  double rrtDelDist_;

  double radRob_; // max dist to obs to declare collision

  int rrtFailItr_;

public:
  scan_plan(ros::NodeHandle*);
  ~scan_plan();

  void wait_for_params(ros::NodeHandle*);
  void build_tree(double*, double*, double);
  void init_dist_map();

  void octomap_cb(const octomap_msgs::Octomap&);
  void path_cost(Eigen::MatrixXd&);

  void test_script();
  void place_ph_cams();

  geometry_msgs::TransformStamped transform_msg(Eigen::Vector3d pos1, Eigen::Vector3d pos2, bool loc=true);
  geometry_msgs::Quaternion yaw_to_quat(double);
  double quat_to_yaw(geometry_msgs::Quaternion);
  double nearest_ph_cam(ph_cam, std::vector<ph_cam>&);
  std::vector<geometry_msgs::Point> path_ph_cams(Eigen::MatrixXd&);

};

#endif
