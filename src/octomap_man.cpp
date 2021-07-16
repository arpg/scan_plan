#include "octomap_man.h"

// ***************************************************************************
octomap_man::octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, std::string vehicleType, double robWidth, double robLength, double maxGroundRoughness, double maxGroundStep, double groundPlaneSearchDist, const std::vector<mapping_sensor>& mapSensors, double baseFrameHeightAboveGround)
{
  maxDistEsdf_ = maxDistEsdf;
  esdfUnknownAsOccupied_ = esdfUnknownAsOccupied;
  vehicleType_ = vehicleType;
  robWidth_ = robWidth;
  robLength_ = robLength;
  radRob_ = std::max(robWidth, robLength) / 2;
  maxGroundRoughness_ = maxGroundRoughness;
  maxGroundStep_ = maxGroundStep;
  groundPlaneSearchDist_ = groundPlaneSearchDist;
  baseFrameHeightAboveGround_ = baseFrameHeightAboveGround;

  mapSensors_ = mapSensors;

  isInitialized_ = 0x00; // wait for the first octree and ufo msgs to set this to 0x03
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
  double yaw = atan2( ( pos2(1) - pos1(1) ) , ( pos2(0) - pos1(0) ) );

  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll( Eigen::Vector4d(pos(0),pos(1),pos(2),yaw) ) && ((pos-robPos_).squaredNorm() > pow(radRob_,2)) )
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

    if ( u_coll( Eigen::Vector4d(pos(0),pos(1),pos(2),0) ) && ((pos-robPos_).squaredNorm() > pow(radRob_,2)) ) // if the point is outside of robot's radius
      return true;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
bool octomap_man::u_coll_with_update(Eigen::Vector4d& pose) // x,y,z,theta
{
  if(vehicleType_ == "air")
    return u_coll_air(pose); // assuming u_coll_air doesnt change the argument
  else
  {
    Eigen::Vector3d avgGroundPt;
    bool returnStatus = cast_pose_down(pose, avgGroundPt);

    if(returnStatus) // if successful projection
    {
      pose(0) = avgGroundPt(0);
      pose(1) = avgGroundPt(1);
      pose(2) = avgGroundPt(2) + baseFrameHeightAboveGround_;
    }

    return !returnStatus; // successful projection means NOT under-collision
  }
}

// ***************************************************************************
bool octomap_man::u_coll(const Eigen::Vector4d& pose) // x,y,z,theta
{
  if(vehicleType_ == "air")
    return u_coll_air(pose);
  else
  {
    Eigen::Vector3d avgGroundPt;
    return !cast_pose_down(pose, avgGroundPt); // successful projection means NOT under-collision
  }
}

// ***************************************************************************
bool octomap_man::u_coll_air(const Eigen::Vector4d& pose) // x,y,z,theta
{
  //Eigen::Vector3d pos( pose(0), pose(1), pose(2) );

  double distObs = octDist_->getDistance ( octomap::point3d( pose(0), pose(1), pose(2) ) );

  if ( distObs == DynamicEDTOctomap::distanceValue_Error )
  {
    if(esdfUnknownAsOccupied_) 
      return true;
    else
      return false;
  }

  if ( distObs <= radRob_ )
    return true;

  return false;
}

// ***************************************************************************
bool octomap_man::cast_pose_down(const Eigen::Vector4d& pose, Eigen::Vector3d& avgGroundPt)
{
  double minElevation, maxElevation;
  return cast_pose_down(pose, avgGroundPt, minElevation, maxElevation);
}

// ***************************************************************************
bool octomap_man::cast_pose_down(const Eigen::Vector4d& pose, Eigen::Vector3d& avgGroundPt, double& minElevation, double& maxElevation)
{
  // false: unsuccessful projection, true: successful projection
  // avgGroundPt, minElevation, maxElevation only valid if true is returned
  // pose: x,y,z,yaw

  if( surfCoordsBase_.rows() < 1 ) // if the robot is represented by no surface shadow, then return unsuccessful projection
  {
    std::cout << "Ground robot half-width is zero :( " << std::endl; 
    return false;
  }

  Eigen::Vector3d pos( pose(0), pose(1), pose(2) );
  double yaw = pose(3);

  const double successfulProjectionsPercent = 0.10;  // if projections success is smaller, it's a cliff or severe decline so return false in that case
  int successfulProjections = 0;

  avgGroundPt = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d currGroundPt(0,0,0);

  bool isFirst = true;
  for(int i=0; i<surfCoordsBase_.rows(); i++)
  {
   // std::cout << "Robot Pose: " << pose.transpose() << ", Surface Point: " << surfCoordsBase_.row(i) << ", " << ( rotz(surfCoordsBase_.row(i), yaw) + pos ).transpose() << std::endl;
    int castDownStatus = cast_ray_down( rotz(surfCoordsBase_.row(i), yaw) + pos, currGroundPt );

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
    //std::cout << "Large elevation change: " << (maxElevation - minElevation) << std::endl;
    return false;
  }

  if( double(successfulProjections) / double(surfCoordsBase_.rows()) < successfulProjectionsPercent )
  {
    //std::cout << "Not enough projections " << double(successfulProjections) << "," << double(surfCoordsBase_.rows()) << std::endl;
    return false;
  }
  else
  {
    avgGroundPt /= double(successfulProjections);
    return true;
  }
}

// ***************************************************************************
Eigen::Vector3d octomap_man::rotz(const Eigen::Vector3d& vecIn, const double& psi) // rotates vecIn(x,y,z) around z axis by angle psi 
{
  Eigen::Vector3d vecOut;

  vecOut(0) = vecIn(0) * cos(psi) - vecIn(1) * sin(psi);
  vecOut(1) = vecIn(0) * sin(psi) + vecIn(1) * cos(psi);
  vecOut(2) = vecIn(2);

  return vecOut;
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

  if( vehicleType_ == "ground" ) // esdf not required for ground vehicles
    return;

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

  if( (isInitialized_ & 0x01) == 0x01 )
    return;

  create_robot_surface(octTree->getResolution());
  isInitialized_ |= 0x01;
}

// ***************************************************************************
void octomap_man::create_robot_surface(const double& voxLen)
{
  // creating robot surface in base frame to be projected to the ground
  int robWidVoxs = ceil(robWidth_ / voxLen);
  int robLenVoxs = ceil(robLength_ / voxLen);

  surfCoordsBase_.resize( robWidVoxs*robLenVoxs, 3 ); // assuming rectangular ground projection of the robot

  for(int i=0; i<robLenVoxs; i++) // rows
    for(int j=0; j<robWidVoxs; j++) // columns
    {
      surfCoordsBase_(i*robWidVoxs+j,0) = -robWidth_/2 + double(j)*voxLen;
      surfCoordsBase_(i*robWidVoxs+j,1) = -robLength_/2 + double(i)*voxLen;
      surfCoordsBase_(i*robWidVoxs+j,2) = 0;
    }
}

// ***************************************************************************
void octomap_man::update_robot_pos(const Eigen::Vector3d& robPos)
{
  robPos_ = robPos;
}

// ***************************************************************************
std::string octomap_man::vehicle_type()
{
  return vehicleType_;
}

// ***************************************************************************
bool octomap_man::get_esdf_unknown_as_occupied()
{
  return esdfUnknownAsOccupied_;
}

// ***************************************************************************
void octomap_man::set_esdf_unknown_as_occupied(const bool& esdfUnkownAsOccupied)
{
  esdfUnknownAsOccupied_ = esdfUnkownAsOccupied;
}

// ***************************************************************************
octomap_man::~octomap_man()
{
  delete octTree_;
  delete octDist_;
}
