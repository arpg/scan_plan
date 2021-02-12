#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
#include "ph_cam.h"
#include "rrt.h"
#include "graph.h"
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
  Eigen::MatrixXd posHist_(100,3); // initialize for 100 points. If the size increases, allocate memory for another 100 points and so on
  int posHistSize_;

  ros::Subscriber octSub_;
  ros::Subscriber poseHistSub_;

  ros::Publisher pathPub_;
  ros::Publisher compTimePub_;
  ros::Publisher frontiersPub_;
  ros::Publisher vizPub_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;

  std::string baseFrameId_;
  std::string worldFrameId_;

  geometry_msgs::TransformStamped baseToWorld_;

  rrt* rrtTree_;
  octomap_man* octMan_;
  path_man* pathMan_;
  graph* graph_;
  std::vector<mapping_sensor> mapSensors_;

  uint8_t isInitialized_;

  double radRob_; // max dist to obs to declare collision

  ros::Timer timerReplan_;

  std::vector<double> cGain_;

  int nHistPosesExpDir_; // number of last poses to calculate exploration heading
  
  Eigen::Vector3d homePos_;

  Eigen::Vector3d localBndsMin_;  
  Eigen::Vector3d localBndsMax_;

  Eigen::Vector3d geoFenceMin_;
  Eigen::Vector3d geoFenceMax_;

public:
  scan_plan(ros::NodeHandle*);
  ~scan_plan();


  void build_tree(double*, double*, double);
  void init_dist_map();

  void octomap_cb(const octomap_msgs::Octomap&);
  void timer_replan_cb(const ros::TimerEvent&);
  void timer_ph_cam_cb(const ros::TimerEvent&);

  void path_cost(Eigen::MatrixXd&);

  void test_script();
  void place_ph_cams();

  void setup_pose();
  void setup_sensors();
  void setup_rrt();
  void setup_graph();
  void setup_timers();
  void setup_scan_plan();

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

  bool min_path_len(Eigen::MatrixXd& path);
  bool add_paths_to_graph(rrt* tree, std::vector<int>& idLeaves, int idLookaheadLeaf, graph* gph);
  bool add_path_to_graph(Eigen::MatrixXd& path, graph*, bool containFrontier);
};

#endif
