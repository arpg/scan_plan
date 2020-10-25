#ifndef RRT_H
#define RRT_H

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

  // outputs of 'find_near' function, avoiding repeated allocation/deallocation
  std::vector<near_node> nearNds_;

public:
  rrt();
  ~rrt();
};

#endif
