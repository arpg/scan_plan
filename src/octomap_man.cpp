#include "octomap_man.h"

// ***************************************************************************
octomap_man::octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, octomap::OcTree*, std::string vehicleType, double radRob)
{
  maxDistEsdf_ = maxDistEsdf;
  esdfUnknownAsOccupied_ = esdfUnknownAsOccupied;
  vehicleType_ = vehicleType;
  radRob_ = radRob;
}

// ***************************************************************************
bool rrt::u_coll(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  const double delLambda = octTree->getResolution() / (pos2 - pos1).norm();

  double lambda = 0;
  Eigen::Vector3d pos;
  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll(pos) )
      return true;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
bool rrt::u_coll(const Eigen::Vector3d& pos)
{
  if(vehicleType == "air")
    u_coll_air(pos);
  else
    u_coll_ground(pos);
}

// ***************************************************************************
bool rrt::u_coll_air(const Eigen::Vector3d& pos)
{
  double distObs = octDist_->getDistance ( octomap::point3d( pos(0), pos(1), pos(2) ) );

  if ( distObs == DynamicEDTOctomap::distanceValue_Error )
    return true;

  if ( distObs <= radRob_ )
    return true;

  return false;
}

// ***************************************************************************
bool rrt::u_coll_ground(const Eigen::Vector3d& pos)
{

}

// ***************************************************************************
void octomap_man::update_esdf(const Eigen::Vector3d& minBnds, const Eigen::Vector3d& maxBnds)
{
  //double rrtBnds[2][3];
  //get_rrt_bounds(rrtBnds);

  octomap::point3d min(minBnds(0), minBnds(1), minBnds(2));
  octomap::point3d max(maxBnds(0), maxBnds(1), maxBnds(2));

  //double x,y,z;

  //octTree_->getMetricMin(x,y,z);
  //octomap::point3d min(x, y, z);

  //octTree_->getMetricMax(x,y,z);
  //octomap::point3d max(x, y, z);

  bool unknownAsOccupied = esdfUnknownAsOccupied_; // true

  float maxDist = maxDistEsdf_; // 5.0

  delete octDist_;
  octDist_ = new DynamicEDTOctomap(maxDist, octTree_, min, max, unknownAsOccupied);

  octDist_->update(true);  //This computes the distance map
  //std::cout << "Initialized dist map" << std::endl;
}

// ***************************************************************************
void octomap_man::update_octree(octomap::OcTree* octTree)
{
  delete octTree_;
  octTree_ = octTree;
}
