#include "scan_plan.h"

// ***************************************************************************
ph_cam_class::ph_cam_class(double* camInfoP, double* camRes, double maxDepth, geometry_msgs::TransformStamped camToBase)
{
  memcpy(camInfoP_, camInfoP, sizeof(double)*9);
  memcpy(camRes_, camRes, sizeof(int)*2);
  maxDepth_ = maxDepth;
  camToBase_ = camToBase;
  compute_polytope();
}

// ***************************************************************************
geometry_msgs::Point ph_cam_class::point2_to_point3(geometry_msgs::Point pointIn, bool direction)
{
	geometry_msgs::Point pointOut;
	
	double fx = camInfoP_[0];
	double cx = camInfoP_[2];
	double fy = camInfoP_[5];
	double cy = camInfoP_[6];
	
	//std::cout << "Camera intrinsics: " << fx << ", " << cx << ", " << fy << ", " << cy << std::endl;
		
	if(direction) // u,v,w => x,y,z
	{
		pointOut.x = (pointIn.x - cx) * pointIn.z / fx;
		pointOut.y = (pointIn.y - cy) * pointIn.z / fy;
		pointOut.z = pointIn.z;
	}
	else // x,y,z => u,v,w
	{		
		pointOut.x = (pointIn.x / pointIn.z) * fx + cx;
		pointOut.y = (pointIn.y / pointIn.z) * fy + cy;
		pointOut.z = pointIn.z;
	}
	
	return pointOut;
}

// ***************************************************************************
void ph_cam_class::compute_polytope()
{
  std::vector<geometry_msgs::Point> polyVerts;
 
  // define 2.5D vertices in camera optical frame
  geometry_msgs::Point pt; 

  pt.x = 0; pt.y = 0; pt.z = 0;
  polyVerts.push_back( pt );

  pt.x = 0; pt.y = camRes_[1]; pt.z = maxDepth_;
  polyVerts.push_back( pt );
 
  pt.x = camRes_[0]; pt.y = camRes_[1]; pt.z = maxDepth_;
  polyVerts.push_back( pt );

  pt.x = camRes_[0]; pt.y = 0; pt.z = maxDepth_;
  polyVerts.push_back( pt );

  pt.x = 0; pt.y = 0; pt.z = maxDepth_;
  polyVerts.push_back( pt );

  // project the vertices to 3D and transform them to base_link
  for (int i=0; i<polyVerts.size(); i++)
  {
    polyVerts[i] = point2_to_point3(polyVerts[i], true);
    tf2::doTransform (polyVerts[i], polyVerts[i], camToBase_);
  }
  polytope_ = polyVerts;
}

// ***************************************************************************
std::vector<geometry_msgs::Point> ph_cam_class::get_polytope()
{
  return polytope_;
}
