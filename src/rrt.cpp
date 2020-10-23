#include "rrt.h"

// ***************************************************************************
rrt::rrt(int nNodes, double* minBnds, double* maxBnds, double nearRad)
{
  nodes_.reserve(nNodes);

  std::memcpy(minBnds_, minBnds, sizeof(double)*3);
  std::memcpy(maxBnds_, maxBnds, sizeof(double)*3);
  //map_ = map; (in some form)

  nearRad_ = nearRad;
  idsNear_.reserve(nNodes);
  costsN_.reserve(nNodes);
  costsL_.reserve(nNodes);
}

// ***************************************************************************
void rrt::clear()
{
  nodes_.resize(0);
}

// ***************************************************************************
return rrt::add_node(geoemtry_msgs::Pose pos, rrt_node* parent)
{
  rrt_node nd;
  nd.pos = pos;
  nd.parent = parent;
  nodes_.push_back(nd);

  return (nodes_.size()-1);
}

// ***************************************************************************
void rrt::build(geometry_msgs::Point posRoot)
{
  add_node(posRoot, NULL);

  while (nodes_.size() < nodes_.capacity())
  {
    geometry_mgs::Point posRand = rand_pos(minBnds_, maxBnds_);

    int idNearest = find_nearest(posRand);
    geometry_msgs::Point posNearest = nodes_[idNearest].pos;

    geometry_msgs::Point posNew = steer(posNearest, posRand, delta_);

    if ( under_collision(posNearest, posNew) )
      continue;

    int idNew = add_node(posNew, &nodes_[idNearest]);
    double costNew = node_cost(idNew); 

    //idsNear_.resize(0);
    //colls_.resize(0);
    find_near(posNew, radNear_); // copy idsNear_,pCosts,lCosts_   

    int idMin = idNearest;
    for (i=0; i<idsNear_.size(); i++)
    {
      int idNear = idsNear_[i];
      double costL = costsL_[i]; // line (with new) cost
      double costN = costsN_[i]; // node cost

      geometry_msgs::Point posNear = nodes_[idNear].pos;

      if( costN + costL < costNew )
      {
        nodes_[idNew].parent = &nodes_[idNear];
        idMin = idNear;
        break; // check if any solution is fine or the least has to be found
      }
    }

    costNew = node_cost(idNew);
    for (i=0; i<idsNear_.size(); i++)
    {
      int idNear = idsNear_[i];
      double costL = costsL_[i]; // line (near-new) cost
      double costN = costsN_[i]; // near node cost

      if ( idNear == idMin )
        continue; 

      geometry_msgs::Point posNear = nodes_[idNear].pos;
 
      if( costNew + costL < costN )
      {
        nodes_[idNear].parent = &nodes_[idNew];
        break; // check if any solution is fine or the least has to be found
      }
    }

  } // end while

}

// ***************************************************************************
geometry_mgs::Point rrt::rand_pos(double* min, double* max)
{
  //double val = (double)rand() / RAND_MAX;
  //return min + val * (max - min);
}

// ***************************************************************************
rrt_node rrt::find_nearest(geometry_msgs::Point pos)
{
  
}

// ***************************************************************************
std::vector<int> rrt::find_near(geometry_msgs::Point pos, double rad)
{
  //clear idsNear_,costsN_, costsL_ first
  // check for collisions too, for pos/new - nears lines, only output collision free ones
}

// ***************************************************************************
geometry_msgs::Point rrt::steer(geometry_msgs::Point posFrom, geometry_msgs::Point posTowards)
{
}

// ***************************************************************************
bool rrt::under_collision(geometry_msgs::Point pos1, geometry_msgs::Point pos2)
{
}

// ***************************************************************************
double node_cost(int id)
{
}
