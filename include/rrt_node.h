#ifndef RRTNODE_H
#define RRTNODE_H

// ***************************************************************************
class rrt_node
{

private: 
  int id_;
  rrt_node* parent_;
  std::vector<rrt_node*> children_;
  double cost_;
  double pos_[3];

public:
  void set_cost(double);

  void set_pos(double, double, double);

  void set_parent(rrt_node*);

  void set_child(rrt_node*);
  
  void set_child_f(rrt_node*);

  void delete_child(rrt_node*);

  void delete_child(int);

  int get_id();

  ~rrt_node();
};

#endif
