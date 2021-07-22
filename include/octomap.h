#ifndef OCTOMAP_H
#define OCTOMAP_H

#include "octomap_msgs/Octomap.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"

#ifdef WITH_ROUGHNESS
  #include <rough_octomap/conversions.h>
  #include <rough_octomap/RoughOcTree.h>
  typedef octomap::RoughOcTree OcTreeT;
#else
  #include <octomap_msgs/conversions.h>
  typedef octomap::OcTree OcTreeT;
#endif


#endif
