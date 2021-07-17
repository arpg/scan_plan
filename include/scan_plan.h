#ifndef SCANPLAN_H
#define SCANPLAN_H

#include "ros/ros.h"
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
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "scan_plan_msgs/PointArrays.h"
#include "boost/algorithm/string/predicate.hpp"

#include "path_man.h"

struct plan_status
{
  enum MODE {LOCALEXP, GOALPT, GLOBALEXP, REPORT, UNSTUCK, MOVEANDREPLAN};

  MODE mode = MODE::LOCALEXP;
  Eigen::Vector3d goalPt = Eigen::Vector3d(0,0,0); // if planning to a goal point
  VertexDescriptor goalVtx; // if planning to a goal vertex

  double volExp = 0; // cubic meters per volGainMonitorDur secs (since lastVolGainCheck)
  ros::Time lastReplanTime = ros::Time::now();

  bool mapUpdated = false;
  int nFailedReplans = 0;
  int nPathInvalidations = 0;
  bool inducedEndOfPath = false;
};

// ***************************************************************************
class scan_plan
{

private:

  ros::NodeHandle* nh_;
  Eigen::MatrixXd posHist_; // initialize for 100 points. If the size increases, allocate memory for another 100 points and so on
  int posHistSize_;

  std::vector<Eigen::MatrixXd> posHistNeighbors_;

  ros::Subscriber octSub_;
  ros::Subscriber poseHistSub_;
  ros::Subscriber goalSub_;
  ros::Subscriber taskSub_;
  ros::Subscriber posHistNeighborsSub_;

  ros::Publisher canPlanPub_;
  ros::Publisher planModePub_; 
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

  ros::Timer timerReplan_;
  double timeIntReplan_;

  std::vector<double> cGain_;

  int nHistPosesExpDir_; // number of last poses to calculate exploration heading
  
  Eigen::Vector3d homePos_;

  Eigen::Vector3d localBndsMin_;  
  Eigen::Vector3d localBndsMax_;

  Eigen::Vector3d localBndsDynMin_;  
  Eigen::Vector3d localBndsDynMax_;

  Eigen::Vector3d geoFenceMin_;
  Eigen::Vector3d geoFenceMax_;

  Eigen::Vector3d entranceMin_;
  Eigen::Vector3d entranceMax_;

  Eigen::MatrixXd minCstPath_;

  const double pi_ = acos(-1);

  plan_status status_;

  double volGainMonitorDur_; // secs
  double minVolGainLocalPlan_;
  int nTriesLocalPlan_;
  int nTriesGlobalPlan_; // number of frontiers to plan
  int nTriesReplan_; // number of unsuccessful replans to declare buggy perception (move and replan)
  int nTriesPathValidation_; // number of consecutive invalidation attempt to delare a dynamic obstacle

  double minPathDistTimerBasedReplan_;
  double timeIntTimerBasedReplan_;

  double endOfPathSuccRad_;

public:
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
  void goal_cb(const geometry_msgs::PoseStamped&);
  bool add_paths_to_graph(rrt* tree, std::vector<int>& idLeaves, int idLookaheadLeaf, graph* gph) ;
  void octomap_cb(const octomap_msgs::Octomap& octmpMsg);
  Eigen::Vector3d geofence_saturation(const Eigen::Vector3d& posIn);
  void test_script();
  double quat_to_yaw(geometry_msgs::Quaternion quat);
  geometry_msgs::Quaternion yaw_to_quat(double yaw);
  bool min_path_len(Eigen::MatrixXd& path);
  void pose_hist_cb(const nav_msgs::Path& poseHistMsg);
  void pos_hist_neighbors_cb(const scan_plan_msgs::PointArrays&);
  void task_cb(const std_msgs::String&);
  void task_cb_str(const std::string&);

  Eigen::MatrixXd plan_locally();
  Eigen::MatrixXd plan_to_graph(const Eigen::Vector3d& fromPos, VertexDescriptor& toVertex);
  Eigen::MatrixXd plan_from_graph(const Eigen::Vector3d& toPos, VertexDescriptor& fromVertex);

  void update_explored_volume(const double& expVol);

  Eigen::MatrixXd plan_to_point(const Eigen::Vector3d& goalPos);
  Eigen::MatrixXd plan_globally();
  Eigen::MatrixXd plan_home();
  bool is_entrance(const Eigen::Vector3d& ptIn);

  void publish_plan_mode();
  void publish_can_plan(bool canPlan);
  bool end_of_path();

  double path_cost_alpha(const Eigen::MatrixXd&, const double&, const double&);
  double path_cost_beta(const Eigen::MatrixXd&, const double&, const double&, double&);
  Eigen::MatrixXd plan_locally(const std::string& costType, const int& nTries, bool = true);
  Eigen::MatrixXd plan_locally(const std::string&, bool = true);  

  int update_local_tree(const Eigen::Vector3d&, const Eigen::Vector3d&);
  void update_local_tree(const Eigen::Vector3d&);
};

#endif
