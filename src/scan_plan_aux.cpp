#include "scan_plan.h"

// ***************************************************************************
fov_cam_class(std::vector<double> camInfoP, std::vector<double> camRes, double maxDepth, geometry_msgs::TransformStamped camToBase)
{
  camInfoP_ = camInfoP;
  camRes_ = camRes;
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
  polyVerts.push_back( geometry_msgs::Point(0, 0, 0) );
  polyVerts.push_back( geometry_msgs::Point(0, camRes_[1], maxDepth_) );
  polyVerts.push_back( geometry_msgs::Point(camRes_[0], camRes_[1], maxDepth_) );
  polyVerts.push_back( geometry_msgs::Point(camRes_[0], 0, maxDepth_) );
  polyVerts.push_back( geometry_msgs::Point(0, 0, maxDepth_) );

  // project the vertices to 3D and transform them to base_link
  for (int i=0; i<polyVerts.size(); i++)
  {
    polyVerts[i] = point2_to_point3(polyVerts[i], true);
    doTransform (polyVerts[i], polyVerts[i], camToBase_);
  }
  polytope_ = polyVerts;
}

// ***************************************************************************
std::vector<geometry_msgs::Point> get_polytope()
{
  return polytope_;
}
