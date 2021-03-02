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
  if(eigPath.rows() < 2)
    return;

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
  // path1 : candidate path
  // path2 : pose history path
  double dist = 0;
  for(int i=0; i<path1.rows(); i++)
    dist += point_to_path_dist( Eigen::Vector3d(path1(i,0), path1(i,1), path1(i,2) ), path2 );

  return ( dist / double(path1.rows()) ); // mean of minimum 1-norm distances
}

// ***************************************************************************
double path_man::point_to_path_dist(const Eigen::Vector3d& ptIn, const Eigen::MatrixXd& pathIn)
{
  return ( pathIn.rowwise() - ptIn.transpose() ).rowwise().lpNorm<1>().minCoeff(); // minimum 1-norm distance to a path
}

// ***************************************************************************
bool path_man::validate_path(Eigen::MatrixXd& path)
{
  // returns false if path length changes, and the modified path

  if(path.rows() < 2) // path with one point is invalid, nothing to check
  {
    path.conservativeResize(0, Eigen::NoChange);
    return true;
  }

  int pathInSz = path.rows();

  for (int i=0; i<(path.rows()-1); i++) // collision check for each segment
  {
    if( !octMan_->u_coll(path.row(i), path.row(i+1)) )
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
bool path_man::path_len_check(const Eigen::MatrixXd& path)
{
  //double pathLength = path_man::path_len(path);
  return (path_len(path) > minPathLen_);
}

// ***************************************************************************

// ***************************************************************************

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
