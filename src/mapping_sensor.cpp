#include "mapping_sensor.h"

// ***************************************************************************
mapping_sensor::mapping_sensor(double fovH, double fovV, double resH, double resV, double range, geometry_msgs::TransformStamped sensorToBase)
{
  if(fovH < 0 || fovH > 360)
  {
    ROS_ERROR("Can't create mapping_sensor object, horizontal FOV must be (0,360]");
    return;
  }
  if(fovV < 0 || fovV > 180)
  {
    ROS_ERROR("Can't create mapping_sensor object, vertical FOV must be (0,180]");
    return;
  }

  fovH_ = fovH;
  fovV_ = fovV;

  resH_ = resH;
  resV_ = resV;

  range_ = range;

  sensorToBase_ = sensorToBase;

  multiRayEndPts_.resize(resH*resV, 3);

  populate_multiray_endpts();
}
// ***************************************************************************
int mapping_sensor::n_ray_unseen_voxels(const Eigen::Vector3d& startPt, const Eigen::Vector3d& endPt, octomap::OcTree* octTree)
{
  octomap::KeyRay keyRay;

  octTree->computeRayKeys(octomap::point3d(startPt(0), startPt(1), startPt(2)),
                 octomap::point3d(endPt(0), endPt(1), endPt(2)),
                 keyRay);
  
  int nUnseenVoxels = 0;
  for(auto key = keyRay.begin(); key != keyRay.end(); ++key)
  {
    octomap::OcTreeNode* node = octTree->search(*key);
    if(node == NULL) // unseen
      nUnseenVoxels++;
    else if(octTree->isNodeOccupied(node)) // occupied
      break;
  }
  return nUnseenVoxels;
}

// ***************************************************************************
double mapping_sensor::volumetric_gain(octomap::OcTree* octTree, const geometry_msgs::TransformStamped& baseToWorld)
{
  double volGain = 0.0;

  for(int i=0; i<multiRayEndPts_.rows(); i++)
  {
    Eigen::Vector3d startPt(baseToWorld.transform.translation.x, 
                            baseToWorld.transform.translation.y, 
                            baseToWorld.transform.translation.z);
    Eigen::Vector3d endPt(multiRayEndPts_(i,0), 
                          multiRayEndPts_(i,1), 
                          multiRayEndPts_(i,2));

    endPt = transform_point(endPt, baseToWorld);

    volGain += n_ray_unseen_voxels(startPt, endPt, octTree);
  }

  return volGain*pow(octTree->getResolution(),3);
}

// ***************************************************************************
Eigen::Vector3d mapping_sensor::transform_point(const Eigen::Vector3d& eigPt, const geometry_msgs::TransformStamped& transform)
{
  geometry_msgs::Point geoPt;
  geoPt.x = eigPt(0);
  geoPt.y = eigPt(1);
  geoPt.z = eigPt(2);

  tf2::doTransform(geoPt, geoPt, transform);

  return Eigen::Vector3d(geoPt.x, geoPt.y, geoPt.z);
}

// ***************************************************************************
void mapping_sensor::populate_multiray_endpts()
{
  // assuming symmetric fovs
  // populating in body frame

  int i = 0;
  for (double h_ang = 0; h_ang < fovH_; h_ang = h_ang+fovH_/resH_)
    for (double v_ang = (90-fovV_/2); v_ang < (90+fovV_/2); v_ang = v_ang+fovV_/resV_)
    {
      Eigen::Vector3d pt;
      pt(0) = range_ * sin(v_ang) * cos(h_ang);
      pt(1) = range_ * sin(v_ang) * sin(h_ang); 
      pt(2) = range_ * cos(v_ang);

      pt = transform_point(pt, sensorToBase_);

      multiRayEndPts_(i,0) = pt(0);
      multiRayEndPts_(i,1) = pt(1);
      multiRayEndPts_(i,2) = pt(2);

      i++;
    }
}
