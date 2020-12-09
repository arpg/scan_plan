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
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float64.h"
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

  double scanBnds_[2][3]; // [min,max] x [x,y,z]

  ros::Subscriber octSub_;

  ros::Publisher pathPub_;
  ros::Publisher lookaheadPub_;
  ros::Publisher compTimePub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;

  std::string baseFrameId_;
  std::string worldFrameId_;

  std::vector<geometry_msgs::TransformStamped> camToBase_;
  geometry_msgs::TransformStamped baseToWorld_;

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

  ros::Timer timerPhCam_;
  ros::Timer timerReplan_;

  std::vector<double> cGain_;

  double lookaheadDist_;

  Eigen::MatrixXd path_;
  std::vector<geometry_msgs::TransformStamped> pathPoses_;

  int nHistPoses_;

public:
  scan_plan(ros::NodeHandle*);
  ~scan_plan();

  void wait_for_params(ros::NodeHandle*);
  void build_tree(double*, double*, double);
  void init_dist_map();

  void octomap_cb(const octomap_msgs::Octomap&);
  void timer_replan_cb(const ros::TimerEvent&);
  void timer_ph_cam_cb(const ros::TimerEvent&);

  void path_cost(Eigen::MatrixXd&);

  void test_script();
  void place_ph_cams();

  bool update_base_to_world();
  geometry_msgs::TransformStamped transform_msg(Eigen::Vector3d pos1, Eigen::Vector3d pos2, bool loc=true);
  geometry_msgs::Quaternion yaw_to_quat(double);
  double quat_to_yaw(geometry_msgs::Quaternion);
  double nearest(ph_cam, std::vector<ph_cam>&);
  std::vector<ph_cam> path_ph_cams(Eigen::MatrixXd&, std::vector<geometry_msgs::TransformStamped>&);
  double fov_dist(std::vector<ph_cam>& phCamsPath);
  double heading_diff(double, std::vector<geometry_msgs::TransformStamped>&);

  void publish_lookahead();
  void publish_path();
  Eigen::MatrixXd interpolate(Eigen::MatrixXd&, int);
  void get_rrt_bounds(double (&rrtBnds)[2][3]);

  double exploration_direction(double&);
  double path_length(Eigen::MatrixXd& path);
  double height_diff(double currHeight, Eigen::MatrixXd& path);
};

#endif
