#include "rrt.h"

// ***************************************************************************
rrt::rrt(int nNodes, double minBnds[3], double maxBnds[3], double radNear, double delDist, double radRob, DynamicEDTOctomap* octDist)
{
  octDist_ = octDist;
  init(nNodes, minBnds, maxBnds, radNear, radRob, delDist);
}

// ***************************************************************************
void rrt::init(int nNodes, double minBnds[3], double maxBnds[3], double radNear, double delDist, double radRob)
{
  posNds_.resize(nNodes,3);
  cstNds_.resize(nNodes);
  idPrnts_.reserve(nNodes);

  std::memcpy(minBnds_, minBnds, sizeof(double)*3);
  std::memcpy(maxBnds_, maxBnds, sizeof(double)*3);

  radNear_ = radNear;
  nearNds_.reserve(nNodes);

  delDist_ = delDist;
  nNodes = nNodes_;

  radRob_ = radRob*radRob;
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
void rrt::build(Eigen::Vector3d posRoot)
{
  clear();
  add_node(posRoot, 0, 0);

  while (actNds_ < nNodes_)
  {
    Eigen::Vector3d posRand = rand_pos(minBnds_, maxBnds_);

    int idNearest = find_nearest(posRand);
    Eigen::Vector3d posNearest = posNds_.row(idNearest);

    Eigen::Vector3d posNew = steer(posNearest, posRand, delDist_);

    if ( u_coll(posNearest, posNew) )
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
 
      double potCstN = cstNew + cstL;
      if( potCstN < cstN )
      {
        //cstN = potCstN;
        modify_node(idNear, posNear, potCstN, idNew);

        break; // check if any solution is fine or the least has to be found
      }
    }

  } // end while

}

// ***************************************************************************
Eigen::Vector3d rrt::rand_pos(double* min, double* max)
{
  srand (time(NULL));

  Eigen::Vector3d pt;

  pt(0) = min[0] + (double)rand() / RAND_MAX * (max[0] - min[0]);
  pt(1) = min[1] + (double)rand() / RAND_MAX * (max[1] - min[1]);
  pt(2) = min[2] + (double)rand() / RAND_MAX * (max[2] - min[2]);
  return pt;
}

// ***************************************************************************
int rrt::find_nearest(Eigen::Vector3d posRand)
{
  Eigen::Index minIndx;
  ( posNds_.topRows(actNds_).rowwise() - posRand.transpose() ).rowwise().squaredNorm().minCoeff(&minIndx);

  return minIndx;
}

// ***************************************************************************
void rrt::find_near(Eigen::Vector3d posNew, double rad)
{
  nearNds_.resize(0);

  near_node nearNd;
  
  for (int i=0; i<actNds_; i++)
  {
    nearNd.cstLnk = ( posNds_.row(i) - posNew.transpose() ).norm();

    if ( nearNd.cstLnk > rad )
      continue;
    if ( u_coll( posNds_.row(i), posNew ) )
      continue;

    nearNd.cstNd = cstNds_(i);
    nearNd.id = i;

    nearNds_.push_back(nearNd);
  }

  // ^ the orders can be reversed in col checks are less expensive than norms
}

// ***************************************************************************
Eigen::Vector3d rrt::steer(Eigen::Vector3d posFrom, Eigen::Vector3d posTowards, double delDist)
{
  Eigen::Vector3d vec = posTowards - posFrom;
  Eigen::Vector3d uVec = vec.normalized();

  if(vec == uVec) // if norm is zero, *this is unchanged
  {
    uVec = Eigen::Vector3d::UnitX();
    std::cout << "WARN: q_rand is very close to q_nearest, choosing random direction" << std::endl;
  }

  return posFrom + delDist * uVec;
}

// ***************************************************************************
bool rrt::u_coll(Eigen::Vector3d pos1, Eigen::Vector3d pos2)
{
  double delLambda = 0.2;

  double lambda = 0;
  Eigen::Vector3d pos;
  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll_octomap(pos) )
      return true;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
bool rrt::u_coll_octomap(Eigen::Vector3d pos)
{
  double distObs = octDist_->getDistance ( octomap::point3d( pos(0), pos(1), pos(2) ) );

  if ( distObs == DynamicEDTOctomap::distanceValue_Error )
    return true;

  if ( distObs <= radRob_ )
    return true;

  return false;
}

// ***************************************************************************
// ***************************************************************************
// ***************************************************************************

// ***************************************************************************
// ***************************************************************************
// ***************************************************************************
// ***************************************************************************


