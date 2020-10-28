#ifndef RRT_H
#define RRT_H

#include <iostream>
//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <vector>

// ***************************************************************************
struct near_node
{
  int id; // node id/index
  double cstNd; // cost of near node
  double cstLnk; // cost of line(near node - new node)
};


class rrt
{

private: 
  Eigen::MatrixXd posNds_; // position of all the nodes (Nx3)
  Eigen::VectorXd cstNds_; // cost of all the nodes (Nx1)
  int nNodes_;

  std::vector<int> idPrnts_; // parent ids of all the nodes

  int actNds_; // active nodes
  double delDist_;

  double minBnds_[3];
  double maxBnds_[3];

  double radNear_;

  std::function<bool(Eigen::Vector3d, Eigen::Vector3d)> u_coll_fcn_ptr_;

  // outputs of 'find_near' function, avoiding repeated allocation/deallocation
  std::vector<near_node> nearNds_;

public:
  rrt(int nNodes, double*, double*, double, double, std::function<bool(Eigen::Vector3d, Eigen::Vector3d)>);
  ~rrt();

  void clear();
  void modify_node(int, Eigen::Vector3d, double, int);
  int add_node(Eigen::Vector3d, double, int);
  void build(Eigen::Vector3d);
  Eigen::Vector3d rand_pos(double*, double*);
  int find_nearest(Eigen::Vector3d);
  void find_near(Eigen::Vector3d, double);
  Eigen::Vector3d steer(Eigen::Vector3d, Eigen::Vector3d, double);
  bool under_collision(Eigen::Vector3d, Eigen::Vector3d);
};

#endif
