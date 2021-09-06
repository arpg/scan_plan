#include "octomap_man.h"

// ***************************************************************************
octomap_man::octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, bool useRoughness, std::string vehicleType, double robWidth, double robLength, double groundPlaneSearchDist, const std::vector<mapping_sensor>& mapSensors, double baseFrameHeightAboveGround, double successfulProjectionsPercent, double successfulStairProjectionsPercent, double maxGroundStep, double maxGroundRoughnessThresh, double avgGroundRoughnessThresh, bool useStairs)
{
  maxDistEsdf_ = maxDistEsdf;
  esdfUnknownAsOccupied_ = esdfUnknownAsOccupied;
  useRoughness_ = useRoughness;
  useStairs_ = useStairs;

  vehicleType_ = vehicleType;
  robWidth_ = robWidth;
  robLength_ = robLength;
  radRob_ = std::max(robWidth, robLength) / 2;
  successfulProjectionsPercent_ = successfulProjectionsPercent / 100; // normalize [0,1] to avoid multiplying by 100 for comparisons
  successfulStairProjectionsPercent_ = successfulStairProjectionsPercent / 100;
  maxGroundRoughnessThresh_ = maxGroundRoughnessThresh;
  avgGroundRoughnessThresh_ = avgGroundRoughnessThresh;
  maxGroundStep_ = maxGroundStep;
  groundPlaneSearchDist_ = groundPlaneSearchDist;
  baseFrameHeightAboveGround_ = baseFrameHeightAboveGround;

  mapSensors_ = mapSensors;

  OcTreeT* dummy(new OcTreeT(0.1)); // necessary dummy only if using rough_octomap

  isInitialized_ = 0x00; // wait for the first octree msg to set this to 0x03

  #ifdef WITH_ROUGHNESS
    ROS_WARN("scan_plan: Running With Roughness");
  #else
    ROS_WARN("scan_plan: Running Without Roughness");
  #endif
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
  const double distPos = (pos2 - pos1).norm();
  double delLambda;

  if( distPos < 1e-3 )
    delLambda = 1.1;
  else
    delLambda = radRob_ / (2*distPos); // projection surfaces should overlap so a thin wall below a path is not missed

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
  const double distPos = (pos2 - pos1).norm();
  double delLambda;

  if( distPos < 1e-3 )
    delLambda = 1.1;
  else
    delLambda = radRob_ / (2*distPos);

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
bool octomap_man::validate(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  const double distPos = (pos2 - pos1).norm();
  double delLambda;

  if( distPos < 1e-3 )
    delLambda = 1.1;
  else
    delLambda = radRob_ / (2*distPos); // projection surfaces should overlap so a thin wall below a path is not missed

  double lambda = 0;
  Eigen::Vector3d pos;
  double yaw = atan2( ( pos2(1) - pos1(1) ) , ( pos2(0) - pos1(0) ) );

  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( !validate3( Eigen::Vector4d(pos(0),pos(1),pos(2),yaw) ) && ((pos-robPos_).squaredNorm() > pow(radRob_,2)) )
      return false; // under-collision

    lambda += delLambda;
  }

  return true; // collision-free
}

// ***************************************************************************
bool octomap_man::validate1(const Eigen::Vector4d& pose)
{
  const int nCollisionVoxs = 4;

  if( surfCoordsBase_.rows() < 1 ) // if the robot is represented by no surface shadow, then return unsuccessful projection
  {
    ROS_ERROR( "scan_plan: Ground robot footprint is zero" ); 
    return false;
  }

  if( (baseFrameHeightAboveGround_-maxGroundStep_) < 3*octTree_->getResolution() )
    ROS_WARN("scan_plan path validate: Not enough clearance below the path");

  if( surfCoordsBase_.rows()  < nCollisionVoxs )
    ROS_WARN("scan_plan path validate: Not enough voxels in the robot footprint");    

  int occupiedVoxs = 0;
  for(int i=0; i<surfCoordsBase_.rows(); i++)
  {
    Eigen::Vector3d pos( pose(0), pose(1), pose(2) );
    double yaw = pose(3);
    Eigen::Vector3d ptIn( rotz(surfCoordsBase_.row(i), yaw) + pos );

    octomap::OcTreeKey currKey = octTree_->coordToKey (ptIn(0), ptIn(1), ptIn(2)); // initialize key at current robot position

    #ifdef WITH_ROUGHNESS
      octomap::RoughOcTree::NodeType* currNodeTop = octTree_->search(currKey);
      currKey.k[2]--;
      octomap::RoughOcTree::NodeType* currNodeBottom = octTree_->search(currKey);
    #else
      octomap::OcTreeNode* currNodeTop = octTree_->search(currKey);
      currKey.k[2]--;
      octomap::OcTreeNode* currNodeBottom = octTree_->search(currKey);
    #endif

    if (currNodeTop != NULL && octTree_->isNodeOccupied(currNodeTop)) // if occupied
      occupiedVoxs++;
    if (currNodeBottom != NULL && octTree_->isNodeOccupied(currNodeBottom)) // if occupied
      occupiedVoxs++;
  }

  if(occupiedVoxs < nCollisionVoxs)
    return true;
  return false; // under-collision
}

// ***************************************************************************
bool octomap_man::validate2(const Eigen::Vector4d& pose)
{
  const int nCollisionVoxs = 4; 

  Eigen::Vector3d ptIn( pose(0), pose(1), pose(2) );
  octomap::OcTreeKey currKey = octTree_->coordToKey (ptIn(0), ptIn(1), ptIn(2)); // initialize key at current robot position

  #ifdef WITH_ROUGHNESS
    octomap::RoughOcTree::NodeType* currNode = octTree_->search(currKey);
  #else
    octomap::OcTreeNode* currNode = octTree_->search(currKey);
  #endif

  if (currNode == NULL)
    return true;

  if (!octTree_->isNodeOccupied(currNode)) // if free
    return true;
  
  int nOccupiedVoxs = 0;
  for( int i=-1; i<=1; i++ )
    for( int j=-1; j<=1; j++ )
      for( int k=-1; k<=1; k++ )
      {
        if( i==0 && j==0 && k==0)
          continue;
        octomap::OcTreeKey key = currKey;
        key.k[0] += i; key.k[1] += j; key.k[2] += k; // center voxel is in-collision, no need to check

        #ifdef WITH_ROUGHNESS
          octomap::RoughOcTree::NodeType* node = octTree_->search(currKey);
        #else
          octomap::OcTreeNode* node = octTree_->search(currKey);
        #endif

        if(node != NULL && octTree_->isNodeOccupied(node))
          nOccupiedVoxs++;
      }

  if( (nOccupiedVoxs+1) < nCollisionVoxs ) // +1 to include the center voxel
    return true;
  return false; // under-collision
}

// ***************************************************************************
bool octomap_man::validate3(const Eigen::Vector4d& pose) // checks the planned path and dropped path
{
  const double minDynObsHeight = 0.55;

  bool isValid = validate2(pose);
  if(!isValid || vehicleType_ == "air")
    return isValid;

  // atleast 3 voxels heigher than max ground elevation, in worst case there should be one voxel between the dropped path and ground voxel
  if( minDynObsHeight < 3 * octTree_->getResolution() )  
  {
    ROS_ERROR_THROTTLE(1, "scan_plan: validate3: min dyn obs height too small");
    return isValid;
  }

  double drop = baseFrameHeightAboveGround_ - minDynObsHeight; // distance of min height obs from robot top

  if(drop <= 0.0)
  {
    ROS_ERROR_THROTTLE(1, "scan_plan: validate3: min dyn obs height greater than robot height");
    return isValid;
  }

  Eigen::Vector4d minHeightPose(pose(0), pose(1), pose(2)-drop, pose(3));

  return validate2(minHeightPose); // the planned path is valid, check for the dropped path
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

  // if projections success is smaller than successfulProjectionsPercent, it's a cliff or severe decline so return false in that case
  int successfulProjections = 0;
  int nStairProjections = 0;

  avgGroundPt = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d currGroundPt(0,0,0);

  double maxRoughness = 0.0, avgRoughness = 0.0;

  bool isFirst = true;
  for(int i=0; i<surfCoordsBase_.rows(); i++)
  {
    double currGroundRoughness = 0.0;

    bool isOnStair;
    int castDownStatus = cast_ray_down( rotz(surfCoordsBase_.row(i), yaw) + pos, currGroundPt, currGroundRoughness, isOnStair );

    if( castDownStatus == 0 ) // under-collision
      return false;

    if( castDownStatus == -1 ) // invalid projection, could not project to ground
      continue;

    // if sucessfully projected to ground
    successfulProjections++;
    avgGroundPt += currGroundPt;
    avgRoughness += currGroundRoughness;

    if(isOnStair)
      nStairProjections++;

    if( currGroundRoughness >  maxRoughness )
      maxRoughness = currGroundRoughness;

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

  if(successfulProjections < 1)
    return false;

  avgRoughness /= double(successfulProjections);
  avgGroundPt /= double(successfulProjections); avgGroundPt(2) = maxElevation;  // put the avgGroundPt at max elevation

  if( double(nStairProjections) > double(surfCoordsBase_.rows()) * successfulStairProjectionsPercent_ )
    return true;

  if( (maxElevation - minElevation) > maxGroundStep_ )
    return false;

  if ( (maxRoughness >= maxGroundRoughnessThresh_) || (avgRoughness >= avgGroundRoughnessThresh_) )
    return false;

  if( double(successfulProjections) < double(surfCoordsBase_.rows()) * successfulProjectionsPercent_ )
    return false;

  return true;
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
int octomap_man::cast_ray_down(const Eigen::Vector3d& ptIn, Eigen::Vector3d& groundPt, double& groundRoughness, bool& isOnStair)
{
  // returns 1: ground found, 0: under collision, -1: ground not found
  // groundPt only valid if 1 is returned, groundRoughness is 0.0 if ground not found or roughness value is invalid

  octomap::OcTreeKey currKey = octTree_->coordToKey (ptIn(0), ptIn(1), ptIn(2)); // initialize key at current robot position

  #ifdef WITH_ROUGHNESS
    octomap::RoughOcTree::NodeType* currNode = octTree_->search(currKey);
  #else
    octomap::OcTreeNode* currNode = octTree_->search(currKey);
  #endif

  groundRoughness = 0.0;
  isOnStair = false;

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

    #ifdef WITH_ROUGHNESS
      groundRoughness = currNode->getRough();
      if( std::isnan(groundRoughness) || !useRoughness_ )
        groundRoughness = 0.0;

      isOnStair = octTree_->isNodeStairs(currNode);
      if( std::isnan(isOnStair) || !useStairs_ )
        isOnStair = false;
    #else
      groundRoughness = 0.0;
      isOnStair = false;
    #endif

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

  octDist_ = new DynamicEDTOctomap(maxDist, (octomap::OcTree*)octTree_, min, max, unknownAsOccupied);

  octDist_->update(true);  //This computes the distance map
  //std::cout << "Initialized dist map" << std::endl;
}

// ***************************************************************************
void octomap_man::update_octree(OcTreeT* octTree)
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
bool octomap_man::check_for_cliff(const Eigen::Vector3d& posIn, const double& headingIn)
{
  Eigen::Vector3d unitVec( cos(headingIn), sin(headingIn), 0 );

  int nVoxelsLookahead = ceil(1.5*robLength_ / octTree_->getResolution()); // one robot length lookahead

  Eigen::Vector3d ptIn;
  Eigen::Vector3d groundPt;
  double groundRoughness; bool isOnStairs;
  for( int i=1; i<nVoxelsLookahead; i++ )
  {
    ptIn = posIn + i*octTree_->getResolution()*unitVec;

    int uColl = cast_ray_down(ptIn, groundPt, groundRoughness, isOnStairs);
    if( uColl == -1 ) // if not projected, it is a cliff
      return true;
    else if( uColl == 0 ) // if under-collision, not a cliff
      return false;
  }

  return false; // if all points are projected, not a cliff
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
double octomap_man::get_base_frame_height_above_ground()
{
  return baseFrameHeightAboveGround_;
}

// ***************************************************************************
bool octomap_man::get_use_roughness()
{
  return useRoughness_;
}

// ***************************************************************************
void octomap_man::set_use_roughness(const bool& useRoughness)
{
  useRoughness_ = useRoughness;
}

// ***************************************************************************
octomap_man::~octomap_man()
{
  delete octTree_;
  delete octDist_;
}
