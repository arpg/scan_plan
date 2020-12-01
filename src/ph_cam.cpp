#include "ph_cam.h"

// ***************************************************************************
ph_cam::ph_cam(double camInfoP[9], double camRes[2], double maxDepth, geometry_msgs::TransformStamped camToBase)
{
  memcpy(camInfoP_, camInfoP, sizeof(double)*9);
  memcpy(camRes_, camRes, sizeof(int)*2);
  maxDepth_ = maxDepth;
  camToBase_ = camToBase;
  compute_polytope();
}

// ***************************************************************************
geometry_msgs::Point ph_cam::point2_to_point3(geometry_msgs::Point pointIn, bool direction)
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
void ph_cam::compute_polytope()
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
std::vector<geometry_msgs::Point> ph_cam::get_polytope()
{
  return polytope_;
}

// ***************************************************************************
std::vector<geometry_msgs::Point> ph_cam::transform(geometry_msgs::TransformStamped baseToWorld)
{
  std::vector<geometry_msgs::Point> polytopeTransformed;

  geometry_msgs::Point pt;
  for (int i=0; i<polytope_.size(); i++)
  {
    tf2::doTransform (polytope_[i], pt, baseToWorld);
    polytopeTransformed.push_back(pt);
  }

  return polytopeTransformed;
}

// ***************************************************************************
double distance(std::vector<geometry_msgs::Point> polytope1, std::vector<geometry_msgs::Point> polytope2)
{
  bd body1; 
  bd body2;

  body1.numpoints = polytope1.size();
  body1.coord = new double*[polytope1.size()];

  for (int i=0; i<polytope1.size(); i++)
  {
    body1.coord[i] = new double[3];
    body1.coord[i][1] = polytope1[i].x;
    body1.coord[i][2] = polytope1[i].y;
    body1.coord[i][3] = polytope1[i].z;
  }

  body2.numpoints = polytope2.size();
  body2.coord = new double*[polytope2.size()];

  for (int i=0; i<polytope2.size(); i++)
  {
    body2.coord[i] = new double[3];
    body2.coord[i][1] = polytope2[i].x;
    body2.coord[i][2] = polytope2[i].y;
    body2.coord[i][3] = polytope2[i].z;
  }

  simplex s;
  double dist = gjk(body1, body2, &s);

  for (int i=0; i<polytope1.size(); i++)
    delete[] body1.coord[i];
  delete[] body1.coord;

  for (int i=0; i<polytope2.size(); i++)
    delete[] body2.coord[i];
  delete[] body2.coord;

  return dist;
}
