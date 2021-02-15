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
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"

#include "path_man.h"


// ***************************************************************************
class scan_plan
{

private:

  ros::NodeHandle* nh_;
  Eigen::MatrixXd posHist_; // initialize for 100 points. If the size increases, allocate memory for another 100 points and so on
  int posHistSize_;

  ros::Subscriber octSub_;
  ros::Subscriber poseHistSub_;
  ros::Subscriber goalSub_;

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
  graph* graph_;
  std::vector<mapping_sensor> mapSensors_;
  path_man* pathMan_;

  uint8_t isInitialized_;

  double radRob_; // max dist to obs to declare collision

  ros::Timer timerReplan_;
  double timeIntReplan_;

  std::vector<double> cGain_;

  int nHistPosesExpDir_; // number of last poses to calculate exploration heading
  
  Eigen::Vector3d homePos_;

  Eigen::Vector3d localBndsMin_;  
  Eigen::Vector3d localBndsMax_;

  Eigen::Vector3d geoFenceMin_;
  Eigen::Vector3d geoFenceMax_;

  Eigen::MatrixXd minCstPath_;

  double pi_ = acos(-1);

  bool changeDetected_;
  //double monitorTimeModeSwitch_;

public:
  enum MODE {LOCALEXP, GLOBALEXP, RETURN, REPORT, GOAL};

  scan_plan(ros::NodeHandle*);
  ~scan_plan();

  void setup_pose();
  void setup_sensors();
  void setup_rrt();
  void setup_graph();
  void setup_timers();
  void setup_octomap();
  void setup_path();
  void setup_scan_plan();
  bool update_base_to_world();
  Eigen::Vector3d transform_to_eigen_pos(const geometry_msgs::TransformStamped& transformIn);
  void timer_replan_cb(const ros::TimerEvent&);
  void goal_cb(const geometry_msgs::PointStamped&);
  bool add_paths_to_graph(rrt* tree, std::vector<int>& idLeaves, int idLookaheadLeaf, graph* gph) ;
  void octomap_cb(const octomap_msgs::Octomap& octmpMsg);
  Eigen::Vector3d geofence_saturation(const Eigen::Vector3d& posIn);
  void test_script();
  double quat_to_yaw(geometry_msgs::Quaternion quat);
  geometry_msgs::Quaternion yaw_to_quat(double yaw);
  bool min_path_len(Eigen::MatrixXd& path);
  void pose_hist_cb(const nav_msgs::Path& poseHistMsg);

  Eigen::MatrixXd generate_local_exp_path(std::vector<int>& idLeaves, int& idPathLeaf);
  scan_plan::MODE next_mode();
  Eigen::MatrixXd plan_to_graph(const Eigen::Vector3d& fromPos, VertexDescriptor& toVertex);
  Eigen::MatrixXd plan_from_graph(const Eigen::Vector3d& toPos, VertexDescriptor& fromVertex);
  
};

#endif
