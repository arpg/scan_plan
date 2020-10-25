#include "rrt.h"

// ***************************************************************************
rrt::rrt(int nNodes, double* minBnds, double* maxBnds, double radNear, double delta)
{
  posNds_.resize(nNodes,3);
  cstNds_.resize(nNodes);
  prntNds_.reserve(nNodes);

  std::memcpy(minBnds_, minBnds, sizeof(double)*3);
  std::memcpy(maxBnds_, maxBnds, sizeof(double)*3);
  //map_ = map; (in some form)

  radNear_ = radNear;
  nearNds_.reserve(nNodes);

  delDist_ = delDist;
  nNodes = nNodes_;
}

// ***************************************************************************
void rrt::clear()
{
  actNds_ = 0;
}

// ***************************************************************************
void rrt::modify_node(int idNd, Eigen::Vector3d pos, double cost, int idPrnt)
{
  posNds_.row(idNd) = pos;
 
  cstNds_(idNd) = cost;

  idPrnts_[idNd] = idPrnt;
}

// ***************************************************************************
int rrt::add_node(Eigen::Vector3d pos, double cost, int idPrnt)
{
  posNds_.row(actNds_) = pos;
 
  cstNds_(actNds_) = cost;

  idPrnts_[actNds_] = idPrnt;

  actNds_++;

  return (actNds_-1);
}

// ***************************************************************************
void rrt::build(geometry_msgs::Point posRoot)
{
  clear();
  add_node(posRoot, 0, 0);

  while (actNds_ < nNodes_)
  {
    Eigen::Vector3d posRand = rand_pos(minBnds_, maxBnds_);

    int idNearest = find_nearest(posRand);
    Eigen::Vector3d posNearest = posNds_.row(idNearest);

    Eigen::Vector3d posNew = steer(posNearest, posRand, delDist_);

    if ( under_collision(posNearest, posNew) )
      continue;

    find_near(posNew, radNear_); // copy idsNear_,pCosts,lCosts_

    double cstNew = cstNds_(idNearest) + delDist_;    
    int idNew = add_node(posNew, cstNew, idNearest);    

    int idMin = idNearest;
    for (int i=0; i<nearNds_.size(); i++)
    {
      int idNear = nearNds_[i].id;
      double cstL = nearNds_[i].cstLnk; // line (with new) cost
      double cstN = nearNds_[i].cstNd; // node cost

      Eigen::Vector3d posNear = posNds_.row(idNear);

      double potCstNew = cstN + cstL;
      if( potCstNew < cstNew )
      {
        //cstNew = potCstNew;
        modify_node(idNew, posNew, potCstNew, idNear);

        idMin = idNear;
        break; // check if any solution is fine or the least has to be found
      }
    }

    cstNew = cstNds_(idNew);
    for (int i=0; i<nearNds_.size(); i++)
    {
      int idNear = nearNds_[i].id;
      double cstL = nearNds_[i].cstLnk; // line (with new) cost
      double cstN = nearNds_[i].cstNd; // node cost

      if ( idNear == idMin )
        continue; 

      Eigen::Vector3d posNear = posNds_.row(idNear);
 
      double potCstN = costNew + costL;
      if( potCstN < costN )
      {
        //cstN = potCstN;
        modify_node(idNear, posNear, potCstN, idNew);

        break; // check if any solution is fine or the least has to be found
      }
    }

  } // end while

}

// ***************************************************************************
geometry_mgs::Point rrt::rand_pos(double* min, double* max)
{
  srand (time(NULL));

  geometry_mgs::Point pt;

  pt.x = min[0] + (double)rand() / RAND_MAX * (max[0] - min[0]);
  pt.y = min[1] + (double)rand() / RAND_MAX * (max[1] - min[1]);
  pt.z = min[2] + (double)rand() / RAND_MAX * (max[2] - min[2]);
  return pt;
}

// ***************************************************************************
int rrt::find_nearest(geometry_msgs::Point pos)
{
  Eigen::Vector3d posRand;
  posRand(0) = pos.x;
  posRand(1) = pos.y;
  posRand(2) = pos.z;

  Eigen::Index minIndx;
  ( posNds_.topRows(actNds_).rowwise() - posRand.transpose() ).rowwise().squaredNorm().minCoeff(&minIndex);

  return minIndx;
}

// ***************************************************************************
void rrt::find_near(geometry_msgs::Point pos, double rad)
{
  nearNds_.resize(0);
  
  Eigen::Vector3d posNew;
  posNew(0) = pos.x;
  posNew(1) = pos.y;
  posNew(2) = pos.z;  

  near_node nearNd;
  
  for (int i=0; i<actNds_; i++)
  {
    nearNd.cstLnk = ( posNds_.row(i) - posNew.transpose() ).norm();

    if ( nearNd.cstLnk > rad )
      continue;
    if ( under_collision( posNds_.row(i), posNew ) )
      continue;

    nearNd.cstNd = cstNds_(i);
    nearNd.id = i;

    nearNds_.push_back(nearNd);
  }

  // ^ the orders can be reversed in col checks are less expensive than norms
}

// ***************************************************************************
geometry_msgs::Point rrt::steer(geometry_msgs::Point posFrom, geometry_msgs::Point posTowards)
{
}

// ***************************************************************************
bool rrt::under_collision(geometry_msgs::Point pos1, geometry_msgs::Point pos2)
{
}
