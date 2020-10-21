#include "rrt_node.h"

// ***************************************************************************
void rrt_node::set_cost(double cost) 
{
  cost_ = cost;
}

// ***************************************************************************
void rrt_node::set_pos(double x, double y, double z) 
{
  pos_[0] = x;
  pos_[1] = y;
  pos_[2] = z;
}
  
// ***************************************************************************
void rrt_node::set_parent(rrt_node* parent)
{
  parent_ = parent;
}

// ***************************************************************************
void rrt_node::set_child(rrt_node* child)
{
  bool exists = false;
  for(int i=0; i<children_.size(); i++)
  {
    if(children_[i]->get_id() == child->get_id())
    {
      exists = true;
      break;
    }
  }
    
  if(!exists)
   children_.push_back(child);
}

// ***************************************************************************
void rrt_node::set_child_f(rrt_node* child)
{
  children_.push_back(child);
}

// ***************************************************************************
void rrt_node::delete_child(rrt_node* child)
{
  int indx = -1;
  for(int i=0; i<children_.size(); i++)
  {
    if(children_[i]->get_id() == child->get_id())
      indx = i;
  }

  if(indx >=0 )
    children_.erase(children_.begin()+indx);
}

// ***************************************************************************
void rrt_node::delete_child(int childId)
{
  int indx = -1;
  for(int i=0; i<children_.size(); i++)
  {
    if(children_[i]->get_id() == childId)
      indx = i;
  }

  if(indx >=0 )
    children_.erase(children_.begin()+indx);
}

// ***************************************************************************
int rrt_node::get_id()
{
  return id_;
}

// ***************************************************************************
rrt_node::~rrt_node()
{
  delete parent_;
  delete[] children_;
}
