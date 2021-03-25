#include "octomap_man.h"

// ***************************************************************************
octomap_man::octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, std::string vehicleType, double radRob, double maxGroundRoughness, double maxGroundStep, double groundPlaneSearchDist, const std::vector<mapping_sensor>& mapSensors, double baseFrameHeightAboveGround)
{
  maxDistEsdf_ = maxDistEsdf;
  esdfUnknownAsOccupied_ = esdfUnknownAsOccupied;
  vehicleType_ = vehicleType;
  radRob_ = radRob;
  maxGroundRoughness_ = maxGroundRoughness;
  maxGroundStep_ = maxGroundStep;
  groundPlaneSearchDist_ = groundPlaneSearchDist;
  baseFrameHeightAboveGround_ = baseFrameHeightAboveGround;

  mapSensors_ = mapSensors;

  isInitialized_ = false; // wait for the first octree to set this to true
}

// ***************************************************************************
double octomap_man::volumetric_gain(const Eigen::Vector3d& basePos)
{
  //std::cout << "Calculating volumetric gain at : " << basePos.transpose() <<std::endl;
  //std::cout << "Number of sensors : " << mapSensors_.size() <<std::endl;

  double volGain = 0;
  for(int i=0; i<mapSensors_.size(); i++)
    volGain += mapSensors_[i].volumetric_gain(octTree_, basePos);

 // std::cout << "Volumetric gain of all sensors : " << volGain <<std::endl;
  return volGain;
}

// ***************************************************************************
bool octomap_man::u_coll(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  if(vehicleType_ == "air")
    return u_coll_air(pos1, pos2);
  else
    return u_coll_ground(pos1, pos2);
}

// ***************************************************************************
bool octomap_man::u_coll_ground(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  //const double delLambda = octTree_->getResolution() / (pos2 - pos1).norm(); // projection surfaces should overlap so a thin wall below a path is not missed
  const double delLambda = radRob_ / (2*(pos2 - pos1).norm()); // projection surfaces should overlap so a thin wall below a path is not missed

  double lambda = 0;
  Eigen::Vector3d pos;

  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll(pos) && ((pos-robPos_).squaredNorm() > pow(radRob_,2)) )
      return true; // under-collision

    lambda += delLambda;
  }

  return false; // collision-free
}

// ***************************************************************************
bool octomap_man::u_coll_air(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  //const double delLambda = octTree_->getResolution() / (pos2 - pos1).norm();
  const double delLambda = radRob_ / (2*(pos2 - pos1).norm());

  //std::cout << "Delta lambda for u_coll: " << delLambda << std::endl;
  double lambda = 0;
  Eigen::Vector3d pos;
  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll(pos) && ((pos-robPos_).squaredNorm() > pow(radRob_,2)) ) // if the point is outside of robot's radius
      return true;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
bool octomap_man::u_coll_with_update(Eigen::Vector3d& pos)
{
  if(vehicleType_ == "air")
    return u_coll_air(pos); // assuming u_coll_air doesnt change the argument
  else
  {
    Eigen::Vector3d avgGroundPt;
    bool returnStatus = cast_pos_down(pos, avgGroundPt);

    if(returnStatus) // if successful projection
    {
      pos = avgGroundPt;
      pos(2) += baseFrameHeightAboveGround_;
    }

    return !returnStatus; // successful projection means NOT under-collision
  }
}

// ***************************************************************************
bool octomap_man::u_coll(const Eigen::Vector3d& pos)
{
  if(vehicleType_ == "air")
    return u_coll_air(pos);
  else
  {
    Eigen::Vector3d avgGroundPt;
    return !cast_pos_down(pos, avgGroundPt); // successful projection means NOT under-collision
  }
}

// ***************************************************************************
bool octomap_man::u_coll_air(const Eigen::Vector3d& pos)
{
  double distObs = octDist_->getDistance ( octomap::point3d( pos(0), pos(1), pos(2) ) );

  if ( distObs == DynamicEDTOctomap::distanceValue_Error )
    return true;

  if ( distObs <= radRob_ )
    return true;

  return false;
}

// ***************************************************************************
bool octomap_man::cast_pos_down(const Eigen::Vector3d& pos, Eigen::Vector3d& avgGroundPt)
{
  // false: unsuccessful projection, true: successful projection
  // avgGroundPt only valid if false is returned

  if( surfCoordsBase_.rows() < 1 ) // if the robot is represented by no surface shadow, then return unsuccessful projection
  {
    std::cout << "Ground robot half-width is zero :( " << std::endl; 
    return false;
  }

  const double successfulProjectionsPercent = 0.10;  // if projections success is smaller, it's a cliff or severe decline so return false in that case
  int successfulProjections = 0;

  avgGroundPt = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d currGroundPt(0,0,0);

  double maxElevation, minElevation;
  bool isFirst = true;
  for(int i=0; i<surfCoordsBase_.rows(); i++)
  {
    int castDownStatus = cast_ray_down( surfCoordsBase_.row(i) + pos.transpose(), currGroundPt );

    if( castDownStatus == 0 ) // under-collision
      return false;

    if( castDownStatus == -1 ) // invalid projection, could not project to ground
      continue;

    // if sucessfully projected to ground
    successfulProjections++;
    avgGroundPt += currGroundPt;

    if(isFirst)
    {
      minElevation = currGroundPt(2);
      maxElevation = currGroundPt(2);
      isFirst = false;
    }
    else if( currGroundPt(2) > maxElevation )
      maxElevation = currGroundPt(2);
    else if( currGroundPt(2) < minElevation )
      minElevation = currGroundPt(2);
  }

  if( (maxElevation - minElevation) > maxGroundStep_ )
  {
    std::cout << "Large elevation change: " << (maxElevation - minElevation) << std::endl;
    return false;
  }

  if( double(successfulProjections) / double(surfCoordsBase_.rows()) < successfulProjectionsPercent )
  {
    std::cout << "Not enough projections " << double(successfulProjections) << "," << double(surfCoordsBase_.rows()) << std::endl;
    return false;
  }
  else
  {
    avgGroundPt /= double(successfulProjections);
    return true;
  }
}

// ***************************************************************************
int octomap_man::cast_ray_down(const Eigen::Vector3d& ptIn, Eigen::Vector3d& groundPt)
{
  // returns 1: ground found, 0: under collision, -1: ground not found
  // groundPt only valid if 1 is returned

  octomap::OcTreeKey currKey = octTree_->coordToKey (ptIn(0), ptIn(1), ptIn(2)); // initialize key at current robot position
  octomap::OcTreeNode* currNode = octTree_->search(currKey);

  // if unknown is occupied and the point to project is unknown, point is under collision
  if (esdfUnknownAsOccupied_ && currNode == NULL)
  {
    groundPt = Eigen::Vector3d(0,0,0);
    return 0;
  }

  // if the point to project is occupied, point is under collision
  if (currNode != NULL && octTree_->isNodeOccupied(currNode) )  
  {
    groundPt = Eigen::Vector3d(0,0,0);
    return 0;
  }

  // if the first voxel is collision-free , cast ray downwards

  for(int i=0; i < ceil(groundPlaneSearchDist_/octTree_->getResolution()); i++ ) // until the search distance is over
  {
    currKey.k[2]--;
    currNode = octTree_->search(currKey);

    if (currNode == NULL) // if unknown
      continue;
    if (!octTree_->isNodeOccupied(currNode)) // if free
      continue;

    // if an occupied cell is found, it is ground, now return the point and search status
    octomap::point3d groundPtOct = octTree_->keyToCoord(currKey);
    groundPt(0) = groundPtOct.x();
    groundPt(1) = groundPtOct.y();
    groundPt(2) = groundPtOct.z();
    return 1;
  }

  // if search all the way down, no ground found
  groundPt = Eigen::Vector3d(0,0,0);
  return -1;
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

  //std::cout << "Updating esdf ..." << std::endl;
  //std::cout << "Min Bounds: " << min.x() << ", " <<  min.y() << ", " <<  min.z() << std::endl;
  //std::cout << "Max Bounds: " << max.x() << ", " <<  max.y() << ", " <<  max.z() << std::endl;
  //std::cout << "Max Dist: " << maxDist << std::endl;
  //std::cout << "Unknown as occupied: " << unknownAsOccupied << std::endl;

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

  if(isInitialized_)
    return;

  // creating robot shadow in base frame to be projected to the ground
  int robLenVoxs = ceil(2*radRob_ / octTree_->getResolution());

  surfCoordsBase_.resize( pow(robLenVoxs,2), 3 ); // assuming square ground projection of the robot

  for(int i=0; i<robLenVoxs; i++) // rows
    for(int j=0; j<robLenVoxs; j++) // columns
    {
      surfCoordsBase_(i*robLenVoxs+j,0) = -radRob_ + double(j)*octTree_->getResolution();
      surfCoordsBase_(i*robLenVoxs+j,1) = -radRob_ + double(i)*octTree_->getResolution();
      surfCoordsBase_(i*robLenVoxs+j,2) = 0;
    }

  isInitialized_ = true;
}

// ***************************************************************************
void octomap_man::update_robot_pos(const Eigen::Vector3d& robPos)
{
  robPos_ = robPos;
}

// ***************************************************************************
octomap_man::~octomap_man()
{
  delete octTree_;
  delete octDist_;
}
