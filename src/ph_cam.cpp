#include "ph_cam.h"

// ***************************************************************************
ph_cam::ph_cam(double camInfoK[9], double camRes[3], double discInt[3])
{
  std::cout << "Creating phcam object" << std::endl;

  //memcpy(camInfoK_, camInfoK, sizeof(double)*9);
  //memcpy(camRes_, camRes, sizeof(int)*2);
  //maxDepth_ = maxDepth;
  //camToBase_ = camToBase;

  disp(camInfoK, 9, "Caminfo P");
  disp(camRes, 3, "Cam Res");
  //disp(maxDepth, "Max Depth");

  set_info(camInfoK, camRes, discInt);

  //double discInt[3]={100,100,0.5};
  compute_polytope(); // computed in base frame

  print_polytope();
}

// ***************************************************************************
geometry_msgs::Point ph_cam::point2_to_point3(geometry_msgs::Point pointIn, bool direction)
{
	geometry_msgs::Point pointOut;
	
  double* camInfo = get_cam_info();
	double fx = camInfo[0];
	double cx = camInfo[1];
	double fy = camInfo[2];
	double cy = camInfo[3];
	
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

  double* camRes = get_cam_res();
  double* discInt = get_disc_int();

  disp(camRes, 3);
  disp(discInt, 3);

  polytope_.resize(0);
 
  // project the vertices to 3D in optical frame
  geometry_msgs::Point pt; 

  pt.x = 0; pt.y = 0; pt.z = 0;
  polytope_.push_back( point2_to_point3(pt, true) );

  pt.x = 0; pt.y = camRes[1]; pt.z = camRes[2];
  polytope_.push_back( point2_to_point3(pt, true) );
 
  pt.x = camRes[0]; pt.y = camRes[1]; pt.z = camRes[2];
  polytope_.push_back( point2_to_point3(pt, true) );

  pt.x = camRes[0]; pt.y = 0; pt.z = camRes[2];
  polytope_.push_back( point2_to_point3(pt, true) );

  pt.x = 0; pt.y = 0; pt.z = camRes[2];
  polytope_.push_back( point2_to_point3(pt, true) );

  // project the polytope inner points to 3D in optical frame
  for(pt.x=discInt[0]; pt.x<camRes[0]; pt.x+=discInt[0])
    for(pt.y=discInt[1]; pt.y<camRes[1]; pt.y+=discInt[1])
      for(pt.z=discInt[2]; pt.z<=camRes[2]; pt.z+=discInt[2])
        polytope_.push_back(point2_to_point3(pt, true));

}

// ***************************************************************************
void ph_cam::print_polytope()
{
  std::cout << "Polytope Vertices: " << std::endl;
  for (int i=0; i<polytope_.size(); i++)
    std::cout << polytope_[i].x << ", " << polytope_[i].y << ", " << polytope_[i].z << std::endl;
}

// ***************************************************************************
void ph_cam::set_polytope(std::vector<geometry_msgs::Point> polytope)
{
  polytope_ = polytope;
}

// ***************************************************************************
std::vector<geometry_msgs::Point> ph_cam::get_polytope()
{
  return polytope_;
}

// ***************************************************************************
void ph_cam::shrink(std::vector<bool> uCollVec)
{
// expected optical frame polytope_ and collision check for all points in polytope_

// TODO: Get everything in optical frame

  double* camRes = get_cam_res();
  double* discInt = get_disc_int();

  const double uBnd = camRes[2]+0.5;

  double maxDepth = uBnd;

  for(int i=0; i<polytope_.size(); i++)
  {
    if(uCollVec[i] == true && polytope_[i].z < maxDepth)
      maxDepth = polytope_[i].z;
  }

  if(maxDepth == 0.0 || maxDepth == 0.0+discInt[2])
    polytope_.resize(0);
  else if(maxDepth < uBnd)
  {
    set_max_depth(maxDepth-discInt[2]);
    compute_polytope();
  }
}

// ***************************************************************************
void ph_cam::transform(geometry_msgs::TransformStamped transform)
{
  for (int i=0; i<polytope_.size(); i++)
    tf2::doTransform(polytope_[i], polytope_[i], transform);
}

// ***************************************************************************
double ph_cam::distance(ph_cam& phCam)
{
  std::vector<geometry_msgs::Point> polytope1 = polytope_;
  polytope1.resize(5);
  std::vector<geometry_msgs::Point> polytope2 = phCam.get_polytope();
  polytope2.resize(5);

  //std::cout << "Here" << std::endl;
  bd body1; 
  bd body2;

  body1.numpoints = polytope1.size();
  //std::cout << "Here" << std::endl;
  body1.coord = new double*[polytope1.size()];

  //std::cout << "Here" << std::endl;

  for (int i=0; i<polytope1.size(); i++)
  {
    body1.coord[i] = new double[3];
    body1.coord[i][0] = polytope1[i].x;
    body1.coord[i][1] = polytope1[i].y;
    body1.coord[i][2] = polytope1[i].z;
  }
    //std::cout << "Here" << std::endl;
  body2.numpoints = polytope2.size();
  body2.coord = new double*[polytope2.size()];

  //std::cout << "Here" << std::endl;
  for (int i=0; i<polytope2.size(); i++)
  {
    body2.coord[i] = new double[3];
    body2.coord[i][0] = polytope2[i].x;
    body2.coord[i][1] = polytope2[i].y;
    body2.coord[i][2] = polytope2[i].z;
  }
  //std::cout << "Here" << std::endl;

  simplex s;
  double dist = gjk(body1, body2, &s);

  //std::cout << "Here" << std::endl;
  for (int i=0; i<polytope1.size(); i++)
    delete[] body1.coord[i];
  delete[] body1.coord;

  for (int i=0; i<polytope2.size(); i++)
    delete[] body2.coord[i];
  delete[] body2.coord;

  return dist;
}

// ***************************************************************************
void ph_cam::set_info(double camInfoK[9], double camRes[3], double discInt[3])
{
  info_[0] = camRes[0];
  info_[1] = camRes[1];
  info_[2] = camRes[2];
  info_[3] = discInt[0];
  info_[4] = discInt[1];
  info_[5] = discInt[2];
  info_[6] = camInfoK[0];
	info_[7] = camInfoK[2];
	info_[8] = camInfoK[4];
	info_[9] = camInfoK[5];
}

// ***************************************************************************
void ph_cam::set_max_depth(double depth)
{
  info_[2] = depth;
}

// ***************************************************************************
double* ph_cam::get_cam_info()
{
  return info_+6;
}

// ***************************************************************************
double* ph_cam::get_cam_res()
{
  return info_+0;
}

// ***************************************************************************
double* ph_cam::get_disc_int()
{
  return info_+3;
}
