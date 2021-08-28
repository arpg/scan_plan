#include "path_man.h"

// ***************************************************************************
path_man::path_man(const double& minPathLen, octomap_man* octMan)
{
  minPathLen_ = minPathLen;
  octMan_ = octMan;
}

// ***************************************************************************
double path_man::path_len(const Eigen::MatrixXd& path) // TODO: use eigen functions instead of for loop
{
  if(path.rows() < 2)
    return 0;

  double pathLen = 0.0;
  for(int i=0; i<(path.rows()-1); i++)
    pathLen += ( path.row(i+1) - path.row(i) ).norm();
  
  return pathLen;
}
// ***************************************************************************
std::pair<double, double> path_man::mean_heading_height(const Eigen::MatrixXd& path, const int& nLastPts) // TODO: use eigen functions instead of for loop
{
  if(path.rows() < 1 || nLastPts < 1)
    return std::pair<double, double>(0.0, 0.0);
  if(path.rows() < 2 || nLastPts < 2)
    return std::pair<double, double>(0.0, path(0,2));

  Eigen::Vector3d estDirVec(0.0, 0.0, 0.0);
  double avgHeight = 0.0;

  int startIndx = std::max(0, int(path.rows()) - nLastPts);

  for(int i=startIndx; i<(path.rows()-1); i++)
  {
    estDirVec += ( path.row(i+1) - path.row(i) ).normalized();

    avgHeight += path(i,2);
  }
  avgHeight += path(path.rows()-1, 2);
  avgHeight = avgHeight / double(path.rows());

  return std::pair<double,double>( atan2(estDirVec(1), estDirVec(0)), avgHeight );
}

// ***************************************************************************
void path_man::publish_empty_path(const std::string& frameId, const ros::Publisher& pathPub)
{
  nav_msgs::Path path;

  path.header.frame_id = frameId;
  path.header.stamp = ros::Time::now();
  path.poses.resize(0);

  pathPub.publish(path);
}

// ***************************************************************************
void path_man::publish_path(const Eigen::MatrixXd& eigPath, const std::string& frameId, const ros::Publisher& pathPub)
{
  //if(eigPath.rows() < 2)
  //  return;

  nav_msgs::Path path;

  path.header.frame_id = frameId;
  path.header.stamp = ros::Time::now();
  path.poses.resize(eigPath.rows());

  for(int i=0; i<eigPath.rows(); i++)
  {
    path.poses[i].header.frame_id = path.header.frame_id;
    path.poses[i].header.stamp = path.header.stamp;

    path.poses[i].pose.position.x = eigPath(i,0);
    path.poses[i].pose.position.y = eigPath(i,1);
    path.poses[i].pose.position.z = eigPath(i,2);

    path.poses[i].pose.orientation.x = 0;
    path.poses[i].pose.orientation.y = 0;
    path.poses[i].pose.orientation.z = 0;
    path.poses[i].pose.orientation.w = 1;
  }
  pathPub.publish(path);
}
// ***************************************************************************
std::pair<double, double> path_man::mean_heading_height_err(double yawIn, double heightIn, const Eigen::MatrixXd& pathIn)
{
  std::pair<double, double> meanHeadingHeightPath = mean_heading_height(pathIn, pathIn.rows());

  double yawErr = abs( yawIn - std::get<0>(meanHeadingHeightPath) );
  double heightErr = abs( heightIn - std::get<1>(meanHeadingHeightPath) );

  return std::make_pair(yawErr, heightErr);
}
// ***************************************************************************
double path_man::path_to_path_dist(const Eigen::MatrixXd& path1, const Eigen::MatrixXd& path2)
{
  // path1 : pose history path (interpolated path) 
  // path2 : candidate path

  if( (path1.rows() < 1) || (path2.rows() < 1) )
    return -1.0;

  double dist = 0;
  for(int i=0; i<path1.rows(); i++)
    dist += point_to_path_dist( Eigen::Vector3d(path1(i,0), path1(i,1), path1(i,2) ), path2 );

  return ( dist / double(path1.rows()) ); // mean of minimum 1-norm distances
}

// ***************************************************************************
double path_man::point_to_paths_dist(const Eigen::Vector3d& ptIn, const std::vector<Eigen::MatrixXd>& pathsIn)
{
  if( pathsIn.size() < 1 )
    return -1.0;

  if( pathsIn[0].rows() < 1 )
    return -1.0;

  double minDist = 0.0;
 
  for( int i=0; i<pathsIn.size(); i++ )
  {
    double dist = point_to_path_dist(ptIn, pathsIn[i]);
    if( (i == 0) || (dist < minDist) )
      minDist = dist;
  }

  return minDist;
}

// ***************************************************************************
double path_man::point_to_path_dist(const Eigen::Vector3d& ptIn, const Eigen::MatrixXd& pathIn)
{
  // return ( pathIn.rowwise() - ptIn.transpose() ).rowwise().lpNorm<1>().minCoeff(); // minimum 1-norm distance to a path

  if( pathIn.rows() < 1 )
    return -1.0;

  if( pathIn.rows() == 1 )
    return (pathIn.row(0).transpose() - ptIn).norm();

  // initialize with distance to first line seg
  double minDist = point_to_line_dist( ptIn, pathIn.row(0), pathIn.row(1) ); 

  if( pathIn.rows() == 2 )
    return minDist;

  for( int i=1; i<(pathIn.rows()-1); i++ )
  {
    double dist = point_to_line_dist( ptIn, pathIn.row(i), pathIn.row(i+1) );

    if( dist < minDist )
      minDist = dist;
  }

  return minDist;
}

// ***************************************************************************
double path_man::point_to_line_dist(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& B)
{
  // AB: Line Segment, P: Point away from the line segment
  Eigen::Vector3d AB = B - A;
  Eigen::Vector3d AP = P - A;
  
  if( AP.dot(AB) <= 0.0 )
    return AP.norm();

  Eigen::Vector3d BP = P - B;

  if( BP.dot(AB) >= 0.0 )
    return BP.norm();

  return sqrt( AB.cross(AP).squaredNorm() / AB.squaredNorm() );

}

// ***************************************************************************
// validates path without modifying it
bool path_man::validate_path_without_mod(const Eigen::MatrixXd& path, const Eigen::Vector3d& minBnd, const Eigen::Vector3d& maxBnd)
{
  Eigen::MatrixXd modPath = path;
  return validate_path(modPath, minBnd, maxBnd);
}

// ***************************************************************************
bool path_man::validate_path(Eigen::MatrixXd& path, const Eigen::Vector3d& minBnd, const Eigen::Vector3d& maxBnd)
{
  // returns false if path length changes, and the modified path

  // IMPORTANT: Requires a global map but only checks the edges which has source vertex inside the local bounds
  // Requires global map because an edge is checked all the way

  if( path.rows() == 0 )
    return true;

  if(path.rows() == 1) // path with one point is invalid, nothing to check
  {
    path.conservativeResize(0, Eigen::NoChange);
    return false;
  }

  int pathInSz = path.rows();

  for (int i=0; i<(path.rows()-1); i++) // collision check for each segment
  {
    // assuming path is being followed, the vehicle should come in local proximity to all vertices, so it's sufficient to check the source vertices for in_bounds
    if( !in_bounds(path.row(i),path.row(i+1), minBnd, maxBnd) || octMan_->validate(path.row(i), path.row(i+1)) ) 
      continue;

    if(i == 0)
    {
      path.conservativeResize(0 , Eigen::NoChange);
      break;
    }
    else
    {
      path.conservativeResize(i+1 , Eigen::NoChange);
      break;
    }
  }

  if( !path_len_check(path) )
    path.conservativeResize(0, Eigen::NoChange);

  return ( pathInSz == path.rows() );
}

// ***************************************************************************
bool path_man::in_bounds(const Eigen::Vector3d& ptInStart, const Eigen::Vector3d& ptInEnd, const Eigen::Vector3d& minBnd, const Eigen::Vector3d& maxBnd)
{

  // if one of the points is inside the box, the line connecting them is inside the box
  if( ( ptInStart(0) > minBnd(0) && ptInStart(0) < maxBnd(0) && 
        ptInStart(1) > minBnd(1) && ptInStart(1) < maxBnd(1) && 
        ptInStart(2) > minBnd(2) && ptInStart(2) < maxBnd(2) ) || 
    
      ( ptInEnd(0) > minBnd(0) && ptInEnd(0) < maxBnd(0) && 
        ptInEnd(1) > minBnd(1) && ptInEnd(1) < maxBnd(1) && 
        ptInEnd(2) > minBnd(2) && ptInEnd(2) < maxBnd(2) ) ) 
    return true;

  // Function derived from http://www.3dkingdoms.com/weekly/weekly.php?a=21
  // should independently be able to check without above but need computations before returning true
  Eigen::Vector3d mExtent = (maxBnd - minBnd) * 0.5;
	Eigen::Vector3d mM = (maxBnd + minBnd) * 0.5;

	// Put line in box space
	Eigen::Vector3d LB1 = ptInStart - mM;
	Eigen::Vector3d LB2 = ptInEnd - mM;

	// Get line midpoint and extent
	Eigen::Vector3d LMid = (LB1 + LB2) * 0.5; 
	Eigen::Vector3d L = (LB1 - LMid);
	Eigen::Vector3d LExt( abs(L(0)), abs(L(1)), abs(L(2)) );

	// Use Separating Axis Test
	// Separation vector from box center to line center is LMid, since the line is in box space
	if ( abs( LMid(0) ) > mExtent(0) + LExt(0) ) return false;
	if ( abs( LMid(1) ) > mExtent(1) + LExt(1) ) return false;
	if ( abs( LMid(2) ) > mExtent(2) + LExt(2) ) return false;
	// Crossproducts of line and each axis
	if ( abs( LMid(1) * L(2) - LMid(2) * L(1))  >  (mExtent(1) * LExt(2) + mExtent(2) * LExt(1)) ) return false;
	if ( abs( LMid(0) * L(2) - LMid(2) * L(0))  >  (mExtent(0) * LExt(2) + mExtent(2) * LExt(0)) ) return false;
	if ( abs( LMid(0) * L(1) - LMid(1) * L(0))  >  (mExtent(0) * LExt(1) + mExtent(1) * LExt(0)) ) return false;
	// No separating axis, the line intersects
	return true;
}

// ***************************************************************************
bool path_man::are_equal(const Eigen::MatrixXd& path1, const Eigen::MatrixXd& path2) // checks if to paths are the same
{
  // checks if start, end points and sizes are the same
  // could be the zero mean squared distance check but this is computationally better if used to check if a path is replaned in the end-of-task scan_plan function

  if( path1.rows() != path2.rows() )
    return false;

  if( path1.rows() == 0 && path2.rows() == 0 )
    return true;

  if( ( path1.topRows(1) - path2.topRows(1) ).lpNorm<1>() < 1e-6 && 
      ( path1.bottomRows(1) - path2.bottomRows(1) ).lpNorm<1>() < 1e-6 )
    return true;

  return false;
}

// ***************************************************************************
bool path_man::path_len_check(const Eigen::MatrixXd& path)
{

  if( path.rows() < 2 )
    return false;

  //double pathLength = path_man::path_len(path);
  return (path_len(path) > minPathLen_);
}

// ***************************************************************************
bool path_man::is_staircase(const Eigen::MatrixXd& path, const double& minStepHeight, const int& minNSteps)
{
  // checks for the elevation changes and that the elevation changes are happening at increasing or decreasing heights

  double minElevation, maxElevation;
  Eigen::Vector3d avgGroundPt;

  std::vector<double> heights; // heights at which the elevation changes are recorded

  for (int i=0; i<path.rows(); i++)
  {
    Eigen::Vector3d pos = path.row(i);
    double yaw = 0; // TODO: GET HEADING FROM PATH, CHECK THIS LOGIC AGAIN BEFORE USING THIS FUNCTION
    if( octMan_->cast_pose_down( Eigen::Vector4d(pos(0),pos(1),pos(2),yaw), avgGroundPt, minElevation, maxElevation) && abs(maxElevation - minElevation) >= minStepHeight )
      heights.push_back(minElevation);
  }

  if(heights.size() < 1) // there is no staircase on flat ground
    return false;
  if(heights.size() < minNSteps) // not enough elevation changes detected
    return false;

  // first step is rooted at zero elevation
  int nSteps = 1;
  
  double currStepHeight = heights[0];
  for (int i=0; i<heights.size(); i++)
  {
    if( abs(heights[i]-currStepHeight) >=  minStepHeight ) // check if the steps are atleast minStepHeight apart
    {
      nSteps++;
      currStepHeight = heights[i];
    }
  }
  
  if(nSteps >= minNSteps)
    return true;
  return false;
}

// ***************************************************************************
Eigen::MatrixXd path_man::interpolate(const Eigen::MatrixXd& path, const double& maxSpacePts) // maxSpacePts: maximum distance between two points
{
  if(path.rows() < 2)
    return path;

  std::vector<Eigen::Vector3d> pathOut;

  Eigen::Vector3d pos1, pos2;
  for(int i=0; i<(path.rows()-1); i++)
  {
    double delTheta = maxSpacePts / ( path.row(i+1) - path.row(i) ).norm();

    for(double theta=0; theta<=1; theta+=delTheta)
      pathOut.push_back( (1-theta) * path.row(i) + theta * path.row(i+1) );
  }

  return Eigen::MatrixXd::Map(pathOut[0].data(), 3, pathOut.size()).transpose();
}

/*
geometry_msgs::TransformStamped path_man::transform_msg(Eigen::Vector3d pos1, Eigen::Vector3d pos2, bool loc) 
{
  // for future use to associate orientations to the path points
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = worldFrameId_;
  transform.child_frame_id = baseFrameId_;

  if(loc)
  {
    transform.transform.translation.x = pos1(0);
    transform.transform.translation.y = pos1(1);
    transform.transform.translation.z = pos1(2);
  }
  else
  {
    transform.transform.translation.x = pos2(0);
    transform.transform.translation.y = pos2(1);
    transform.transform.translation.z = pos2(2);
  }

  Eigen::Vector3d pos = pos2 - pos1;

  transform.transform.rotation = yaw_to_quat(atan2(pos(1), pos(0)));

  return transform;
}
*/
