#ifndef RRT_H
#define RRT_H

// ***************************************************************************
struct rrt_node
{
  rrt_node* parent;
  geometry_msgs::Point pos;
};


class rrt
{

private: 
  std::vector<rrt_node> nodes_;

  double minBnds_[3];
  double maxBnds_[3];

  double radNear_;

  // outputs of 'find_near' function, avoiding repeated allocation/deallocation
  std::vector<int> idsNear_; // near node ids (indx in the list nodes_)
  std::vector<double> costsN_; // near-root cost
  std::vector<double> costsL_; // near-new cost

  double delta_;
public:
  

  rrt();
  ~rrt();
};

#endif
