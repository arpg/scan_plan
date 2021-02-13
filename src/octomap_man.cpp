#include "octomap_man.h"

// ***************************************************************************
octomap_man::octomap_man(double maxDistEsdf, bool esdfUnknownAsOccupied, std::string vehicleType, double radRob, double maxGroundRoughness, double maxGroundStep, const std::vector<mapping_sensor>& mapSensors)
{
  maxDistEsdf_ = maxDistEsdf;
  esdfUnknownAsOccupied_ = esdfUnknownAsOccupied;
  vehicleType_ = vehicleType;
  radRob_ = radRob;
  maxGroundRoughness_ = maxGroundRoughness;
  maxGroundStep_ = maxGroundStep;

  mapSensors_ = mapSensors;

  isInitialized_ = false; // wait for the first octree to set this to true
}
// ***************************************************************************
double octomap_man::volumetric_gain(const Eigen::Vector3d& basePos)
{
  double volGain = 0;
  for(int i=0; i<mapSensors_.size(); i++)
    volGain += mapSensors_[i].volumetric_gain(octTree_, basePos);
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
  const double delLambda = octTree_->getResolution() / (pos2 - pos1).norm();

  double lambda = 0;
  Eigen::Vector3d pos;

  double groundRoughness;
  Eigen::Vector3d groundPt = pos1;
  Eigen::Vector3d lastGroundPt = pos1;
  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll(pos, groundRoughness, groundPt) )
      return true;
    if ( abs(groundPt(2) - lastGroundPt(2)) > maxGroundStep_ )
      return true;

    lastGroundPt = groundPt;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
bool octomap_man::u_coll_air(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
  const double delLambda = octTree_->getResolution() / (pos2 - pos1).norm();

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
bool octomap_man::u_coll(const Eigen::Vector3d& pos)
{
  double groundRoughness;
  Eigen::Vector3d groundPt;

  return u_coll(pos, groundRoughness, groundPt);
}
// ***************************************************************************
bool octomap_man::u_coll(const Eigen::Vector3d& pos, double& groundRoughness, Eigen::Vector3d& groundPt)
{
  groundRoughness = 0;
  groundPt = Eigen::Vector3d(0,0,0);

  if(vehicleType_ == "air")
    return u_coll_air(pos);
  else
    return u_coll_ground(pos, groundRoughness, groundPt);
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
bool octomap_man::u_coll_ground(const Eigen::Vector3d& pos, double& roughness, Eigen::Vector3d& groundPt) 
{
  // projects the robot's shadow image (square) to ground and checks for feasibility/collision/terrain

  //Eigen::MatrixXd surfCoords = surfCoordsBase_ + pos.transpose();

  if( surfCoordsBase_.rows() < 1 ) // if the robot is represented by no surface shadow, then it's collision-free
    return false;

  const double successfulProjectionsPercent = 0.75;  // if projections success is smaller, it's a cliff or severe decline so return false in that case
  int successfulProjections = 0;

  roughness = 0;
  groundPt = Eigen::Vector3d(0,0,0);

  double currRoughness = 0;
  Eigen::Vector3d currGroundPt(0,0,0);

  const int groundPtIndx = ceil(double(surfCoordsBase_.rows())/2) - 1; // select the mid point of the shadow projection as the groundPoint
  for(int i=0; i<surfCoordsBase_.rows(); i++)
  {
    if( !project_point_to_ground( surfCoordsBase_.row(i).transpose() + pos, currRoughness, currGroundPt ) ) // translate robot shadow point to pos and project
      continue;
    roughness += currRoughness;
    successfulProjections++;

    if(i==groundPtIndx)
      groundPt = currGroundPt;
  }

  if( double(successfulProjections) / double(surfCoordsBase_.rows()) < successfulProjectionsPercent )
    return true;

  roughness = roughness / double(successfulProjections);
  groundPt = groundPt / double(successfulProjections);

  if( roughness > maxGroundRoughness_ )
    return true;

  return false;
}

// ***************************************************************************
bool octomap_man::project_point_to_ground(const Eigen::Vector3d& pos, double& roughness, Eigen::Vector3d& groundPt)
{
  // returns if a ground is found (true) or not (false), alongwith ground roughness and point

  octomap::OcTreeKey currKey = octTree_->coordToKey (pos(0), pos(1), pos(2)); // initialize key at current robot position

  if (octTree_->search(currKey)->getLogOdds() > 0) // if the pos to project is occupied, cannot project to ground
    return false;

  for(int i=0; i < ceil(groundPlaneSearchDist_/octTree_->getResolution()); i++ ) // until the search distance is over
  {
    currKey.k[2]++;
    octomap::OcTreeNode* currNode = octTree_->search(currKey);
    if (currNode == NULL) // if unknown
      continue;
    if (currNode->getLogOdds() < 0) // if free
      continue;

    // if an occupied cell is found, it is ground, now calculate the roughness and return the point
    octomap::point3d groundPtOct = octTree_->keyToCoord(currKey);
    groundPt(0) = groundPtOct.x();
    groundPt(1) = groundPtOct.y();
    groundPt(2) = groundPtOct.z();

    std::vector<octomap::point3d> surfNormals;
    if ( octTree_->getNormals (groundPtOct, surfNormals, true) ) // treat unknown as occupied, it should never encounter an unknown cell
      roughness = surfNormals[0].angleTo( octomap::point3d(0,0,1) ); // TODO: merge in other surfNormals than just 0
    else
    {
      // for debugging, to observe the reliability of the function
      ROS_WARN("Surface normal not available, assuming (0,0,1)");
      roughness = 0;
    }
    return true;
  }

  return false;
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

  if(isInitialized_)
    return;

  // creating robot shadow in base frame to be projected to the ground
  int robLenVoxs = ceil(2*radRob_ / octTree_->getResolution());

  surfCoordsBase_.resize( pow(robLenVoxs,2), 2 ); // assuming square ground projection of the robot

  for(int i=0; i<robLenVoxs; i++) // rows
    for(int j=0; j<robLenVoxs; j++) // columns
    {
      surfCoordsBase_(i*robLenVoxs+j,0) = -radRob_ + double(j)*octTree_->getResolution();
      surfCoordsBase_(i*robLenVoxs+j,1) = -radRob_ + double(i)*octTree_->getResolution();
    }

  isInitialized_ = true;
}

// ***************************************************************************
octomap_man::~octomap_man()
{
  delete octTree_;
  delete octDist_;
}
