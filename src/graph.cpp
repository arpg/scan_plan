#include "graph.h"
#include "rrt.h"

// ***************************************************************************
graph::graph(Eigen::Vector3d posRoot, double radNear, double minDistNodes, int maxEdgesPerVertex, double minVolGain, std::string frameId, octomap_man* octMan, double minManDistFrontier, const std::vector<double>& entranceMin, const std::vector<double>& entranceMax, const std::vector<double>& cGain, const double& manDistAvoidFrontier, const int& maxNAvoidFrontiers, const double& minSeparationRobots)
{
  adjList_ = new BiDirectionalGraph;

  gvert vertRoot;

  vertRoot.pos = posRoot;
  vertRoot.commSig = 0; // assumes 0 signal strength at launch
  vertRoot.isFrontier = false; // assumes root is not a frontier
  vertRoot.terrain = gvert::FLAT;

  homeVert_ = boost::add_vertex(vertRoot, *adjList_);
  isHomeVertConnected_ = false;

  minDistNodes_ = minDistNodes;
  maxEdgesPerVertex_ = maxEdgesPerVertex;
  radNear_ = radNear;
  frameId_ = frameId;
  minVolGain_ = minVolGain;
  minManDistFrontier_ = minManDistFrontier;
  manDistAvoidFrontier_ = manDistAvoidFrontier;
  maxNAvoidFrontiers_ = maxNAvoidFrontiers;
  minSeparationRobots_ = minSeparationRobots;
  cGain_ = cGain;

  entranceMin_ = Eigen::Vector3d(entranceMin[0], entranceMin[1], entranceMin[2]);
  entranceMax_ = Eigen::Vector3d(entranceMax[0], entranceMax[1], entranceMax[2]);

  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);

  std::cout << "CREATED GRAPH OBJECT" << std::endl;
  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    VertexDescriptor vertDesc = *it;
    std::cout << (*adjList_)[vertDesc].pos << std::endl;
  }

  octMan_ = octMan;
}

// ***************************************************************************
bool graph::add_vertex(const gvert& vertIn, bool ignodeMinDistNodes)
{
  VertexDescriptor vertInDesc;
  return add_vertex(vertIn,  vertInDesc, ignodeMinDistNodes);
}

// ***************************************************************************
bool graph::add_vertex(const gvert& vertIn, VertexDescriptor& vertInDesc, bool ignoreMinDistNodes)
{
  // return true if a connected vertex is added, false if either vertex is not added or added but is not connected

  // TODO: Consider using manhattan distance as the edge cost to speed up 
  bool success = false;

  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);

  vertsOutDesc_.resize(0); // descriptors of target vertices such that the connection is collision-free
  vertsOutDist_.resize(0); // distances of target vertices from the vertIn

  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    if( maxEdgesPerVertex_ < (vertsOutDesc_.size()+1) ) // only add nEdgesPerVertex_ edges
      break;

    double dist = (vertIn.pos - (*adjList_)[*it].pos).squaredNorm();

    if( !ignoreMinDistNodes && dist < minDistNodes_ )
      return false; // a node already exists at the same position

    if( dist > pow(radNear_,2) )
      continue;
    
    if( !u_coll(vertIn, (*adjList_)[*it]) )
    {
      vertsOutDesc_.push_back( *it );
      vertsOutDist_.push_back( sqrt(dist) );
    }
  }

  if( vertsOutDesc_.size() > 0 )
  {
    vertInDesc = boost::add_vertex(vertIn, *adjList_);
    success = true;
  }
  else
  {
    vertInDesc = boost::add_vertex(vertIn, *adjList_);
    success = false;
  }

  for(int i=0; i<vertsOutDesc_.size(); i++) 
    boost::add_edge( vertInDesc, vertsOutDesc_[i], vertsOutDist_[i], *adjList_ );

  if(vertIn.isFrontier)
  {
    frontier front;
    front.vertDesc = vertInDesc;
    front.volGain = 0.0;
    frontiers_.push_front(front);
  }

  if(!isHomeVertConnected_ && boost::in_degree(homeVert_, *adjList_) < 1 ) // if home was not connected and is still not connected (assuming the node is not added above unless connected to home node)
  {
    //if( !success )
    //  vertInDesc = boost::add_vertex(vertIn, *adjList_);
    homeVert_ = vertInDesc;
  }
  else if(!isHomeVertConnected_ && boost::in_degree(homeVert_, *adjList_) >= 1) // if home was not connected and is now connected
    isHomeVertConnected_ = true;

  return success;
}

// ***************************************************************************
Eigen::MatrixXd graph::plan_home(const VertexDescriptor& fromVert)
{
  return plan_shortest_path(fromVert, homeVert_);
}

// ***************************************************************************
bool graph::u_coll(const gvert vert1, const gvert vert2)
{
  return octMan_->u_coll(vert1.pos, vert2.pos);
}

// ***************************************************************************
void graph::update_occupancy(const Eigen::Vector3d& minPt, const Eigen::Vector3d& maxPt, const bool& occupiedOnly)
{
// updates occupancy of edges in a local box defined by minPt and maxPt
// IMPORTANT: Requires a global map but only checks the edges that are going through the local bounds
// Requires global map because an edge that intersects the bounds is checked all the way

  std::pair<EdgeIterator, EdgeIterator> edgeItr = boost::edges(*adjList_);

  for(EdgeIterator it=edgeItr.first; it!=edgeItr.second; ++it)
  {
    if( !in_bounds(*it, minPt, maxPt) )
      continue;
 
    double edgeWeight = boost::get(boost::edge_weight, *adjList_, *it);
    if( occupiedOnly && edgeWeight > 0 )
      continue;

    //std::cout << "Checking for collision to check occupancy" << std::endl;
    if( octMan_->u_coll( get_src_pos(*it), get_tgt_pos(*it) ) ) // if edge intersects the input bounds and is in collision
    { 
      // set edge weight to negative so the edges filter out when graph is filtered
      if(edgeWeight > 0)
        boost::put(boost::edge_weight, *adjList_, *it, -1 * edgeWeight );
    }
    else
    {
      // set edge weight to positive so the edges don't filter out when graph is filtered
      if(edgeWeight < 0)
        boost::put(boost::edge_weight, *adjList_, *it, -1 * edgeWeight );
    }
  }
}

// ***************************************************************************
bool graph::in_bounds(const EdgeDescriptor& edgeD, const Eigen::Vector3d& minPt, const Eigen::Vector3d& maxPt)
{
  return path_man::in_bounds(get_src_pos(edgeD), get_tgt_pos(edgeD), minPt, maxPt);
}

// ***************************************************************************
void graph::publish_frontiers(ros::Publisher& frontiersPub)
{
  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = frameId_;
  poseArr.header.stamp = ros::Time::now();

  for (frontier& front : frontiers_)
  { 
    geometry_msgs::Pose pose;
    pose.position.x = (*adjList_)[front.vertDesc].pos(0);
    pose.position.y = (*adjList_)[front.vertDesc].pos(1);
    pose.position.z = (*adjList_)[front.vertDesc].pos(2);

    pose.orientation.w = 1;

    poseArr.poses.push_back(pose);
  }

  frontiersPub.publish(poseArr);
}

// ***************************************************************************
void graph::publish_viz(ros::Publisher& vizPub)
{
  // vertices
  visualization_msgs::Marker vertices;
  vertices.header.frame_id = frameId_;
  vertices.header.stamp = ros::Time::now();
  vertices.ns = "vertices";
  vertices.id = 0;
  vertices.type = visualization_msgs::Marker::SPHERE_LIST;
  vertices.action = visualization_msgs::Marker::MODIFY;
  vertices.pose.orientation.w = 1;
  geometry_msgs::Vector3 scale;
	scale.x = 0.05;
	scale.y = 0.05;
	scale.z = 0.05;
	vertices.scale = scale;
  
  geometry_msgs::Point vertex;
  std_msgs::ColorRGBA color;

  std::pair<VertexIterator, VertexIterator> vertItr = boost::vertices(*adjList_);
  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    vertex.x = (*adjList_)[*it].pos(0);
    vertex.y = (*adjList_)[*it].pos(1);
    vertex.z = (*adjList_)[*it].pos(2);
    vertices.points.push_back(vertex);

    color.r = 0; color.g = 1; color.b = 0; color.a = 1;
    vertices.colors.push_back(color);
  }

  // edges
  visualization_msgs::Marker free_edges;
  free_edges.header.frame_id = frameId_;
  free_edges.header.stamp = ros::Time::now();
  free_edges.ns = "free_edges";
  free_edges.id = 0;
  free_edges.type = visualization_msgs::Marker::LINE_LIST;
  free_edges.action = visualization_msgs::Marker::ADD;
  free_edges.pose.orientation.w = 1; 
  scale.x = 0.025;
	scale.y = 0.025;
	scale.z = 0.025;
	free_edges.scale = scale;

  visualization_msgs::Marker occupied_edges = free_edges;
  occupied_edges.ns = "occupied_edges";
  
  geometry_msgs::Point vertex1;
  geometry_msgs::Point vertex2;
  
  std::pair<EdgeIterator, EdgeIterator> edgeItr = boost::edges(*adjList_);
  for(EdgeIterator it=edgeItr.first; it!=edgeItr.second; ++it)
  {
    VertexDescriptor vertexDesc = boost::source(*it, *adjList_); // source vertex
    vertex1.x = (*adjList_)[vertexDesc].pos(0);
    vertex1.y = (*adjList_)[vertexDesc].pos(1);
    vertex1.z = (*adjList_)[vertexDesc].pos(2);
    
    vertexDesc = boost::target(*it, *adjList_); // target vertex
    vertex2.x = (*adjList_)[vertexDesc].pos(0);
    vertex2.y = (*adjList_)[vertexDesc].pos(1);
    vertex2.z = (*adjList_)[vertexDesc].pos(2);
  
   if( get(boost::edge_weight, *adjList_, *it) == 0 )
     ROS_ERROR("Zero Edge Weight");

    if( get(boost::edge_weight, *adjList_, *it) < 0 )
    {
      occupied_edges.points.push_back(vertex1);
      occupied_edges.points.push_back(vertex2);
      
      color.r = 1; color.g = 0; color.b = 0; color.a = 1;
      occupied_edges.colors.push_back(color);
      occupied_edges.colors.push_back(color);
    }
    else
    {
      free_edges.points.push_back(vertex1);
      free_edges.points.push_back(vertex2);
      
      color.r = 0; color.g = 1; color.b = 0; color.a = 1;
      free_edges.colors.push_back(color);
      free_edges.colors.push_back(color);
    }
  }

  // vertices + edges
  visualization_msgs::MarkerArray vizMsg;
  vizMsg.markers.push_back(vertices);
  vizMsg.markers.push_back(free_edges);
  vizMsg.markers.push_back(occupied_edges);

  // frontiers
  visualization_msgs::Marker frontViz;
  frontViz.header.frame_id = frameId_;
  frontViz.header.stamp = ros::Time::now();
  frontViz.ns = "frontiers";
  frontViz.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  frontViz.action = visualization_msgs::Marker::ADD;
  scale.x = 0.5;
	scale.y = 0.5;
	scale.z = 0.5;
	frontViz.scale = scale;

  int idFrontViz = 0;
  for(frontier& front: frontiers_)
  {
    frontViz.pose.position.x = (*adjList_)[front.vertDesc].pos(0);
    frontViz.pose.position.y = (*adjList_)[front.vertDesc].pos(1);
    frontViz.pose.position.z = (*adjList_)[front.vertDesc].pos(2)+1.0;
    frontViz.pose.orientation.w = 1;
    
    frontViz.id = idFrontViz++;
    
    frontViz.color.r = 1;
    frontViz.color.g = 1;
    frontViz.color.b = 1;
    frontViz.color.a = 1;

    frontViz.text = std::to_string(front.volGain);
    vizMsg.markers.push_back(frontViz);
  }
  
  vizPub.publish(vizMsg);
}

// ***************************************************************************
graph::~graph()
{
  delete adjList_;
}

// ***************************************************************************
void graph::remove_frontier(const Eigen::Vector3d& pos)
{
  for(frontier& front: frontiers_)
  {
    if( (get_pos(front.vertDesc) - pos).lpNorm<1>() < 0.001 )
      (*adjList_)[front.vertDesc].isFrontier = false;
  }

  graph* gPtr = this;
  frontiers_.remove_if( [gPtr, &pos](const frontier& front) -> bool {return ( (gPtr->get_pos(front.vertDesc)-pos).lpNorm<1>() < 0.001 );} ); // lambda expression
}

// ***************************************************************************
void graph::update_frontiers_vol_gain()
{
  if(frontiers_.empty())
    return;

  int sz = 0;
  for(frontier& front: frontiers_)
    sz++;
  std::cout << "Num Frontiers: " <<  sz << std::endl;
  for(frontier& front: frontiers_)
  {
    front.volGain = octMan_->volumetric_gain( (*adjList_)[front.vertDesc].pos );
    if(front.volGain < minVolGain_)
      (*adjList_)[front.vertDesc].isFrontier = false;
  }
  double minV = minVolGain_;
  frontiers_.remove_if( [&minV](const frontier& front) -> bool {return (front.volGain < minV);} ); // lambda expression
}

// ***************************************************************************
std::forward_list<frontier> graph::pull_well_separated_frontiers(std::forward_list<frontier> frontiersIn, const std::vector<Eigen::MatrixXd>& pathsIn)
{
// extract frontiers that are far from pathsIn

  if( frontiersIn.empty() ) 
    return frontiersIn;

  std::forward_list<frontier> frontiers;
  frontier maxCstFrontier;

  double maxCst = -1.0; // separation cannot be less than zero
  for(frontier& front: frontiersIn)
  {
   double separationDist = path_man::point_to_paths_dist(get_pos(front.vertDesc), pathsIn);
   if( (separationDist < 0.0) || (separationDist >= minSeparationRobots_) ) // push the well-separated ones to the list
     frontiers.push_front(front);
   else if( separationDist > maxCst ) // from not -well-separated ones, find the one with maximum separation
   {
     maxCst = separationDist;
     maxCstFrontier = front;
   }
  }

  // since frontiersIn should not be empty, frontiers or maxCstFrontier should have/be atleast one valid frontier
  // if can't find the one with good enough distance, find the one with max distance

  if( frontiers.empty() )
    frontiers.push_front(maxCstFrontier);
  else
    frontiers.reverse();
  return frontiers;
}

// ***************************************************************************
std::forward_list<frontier> graph::ignore_avoid_frontiers()
{
  std::forward_list<frontier> frontiers;

  for(frontier& front: frontiers_)
  {
   if( !is_avoid_frontier(get_pos(front.vertDesc)) )
     frontiers.push_front(front);
  }

  frontiers.reverse();
  return frontiers;
}

// ***************************************************************************
bool graph::is_avoid_frontier(const Eigen::Vector3d& pos)
{
  for(int i=0; i<avoidFrontiers_.size(); i++)
  {
    if( ( avoidFrontiers_[i] - pos ).lpNorm<1>() < manDistAvoidFrontier_ )
      return true;
  }

  return false;
}

// ***************************************************************************
bool graph::add_path(Eigen::MatrixXd& path, bool containFrontier)
{
  if(path.rows() < 1)
    return false;

  bool success= false;

  // first check if the new frontier is distant enough from the existing ones

  int fIndx = path.rows()-1; // potential frontier index = last point on path
  Eigen::Vector3d frontPt( path(fIndx,0), path(fIndx,1), path(fIndx,2) );
  if( containFrontier && !frontiers_.empty() && (closest_frontier_man_dist(frontPt) < minManDistFrontier_) ) // man_dist >= 0 cz frontiers_ is not empty
    return false;
  
  gvert vert;
  for(int i=0; i<path.rows(); i++)
  {
    vert.pos = path.row(i);
 
    vert.commSig = 0;
    
    if( containFrontier && i==(path.rows()-1) && !is_entrance(path.row(i)) )
      vert.isFrontier = true;
    else
      vert.isFrontier = false;

    vert.terrain = gvert::UNKNOWN;

    //std::cout << "Adding vertex" << std::endl;
    success |= add_vertex(vert);
  }

  return success;
}

// ***************************************************************************
frontier graph::closest_frontier(const Eigen::Vector3d& ptIn, double& dist, std::forward_list<frontier> frontiersIn) // closest frontier to the ptIn
{
  if(frontiersIn.empty())
  {
    frontier front;
    dist = -1.0;
    front.volGain = -1.0;
    return front;
  }  

  frontier closestFront = frontiersIn.front(); // pick the first frontier as the best guess
  double minDist = (get_pos(closestFront.vertDesc) - ptIn).lpNorm<1>();

  for(frontier& front: frontiersIn)
  {
    double manDist = (get_pos(front.vertDesc) - ptIn).lpNorm<1>();
    if( manDist < minDist )
    {
      closestFront = front;
      minDist = manDist;
    }
  }

  dist = minDist;
  return closestFront;
}

// ***************************************************************************
double graph::closest_frontier_man_dist(const Eigen::Vector3d& ptIn) // closest frontier to the ptIn
{
  double dist;
  closest_frontier(ptIn, dist, frontiers_);
  return dist;  
}

// ***************************************************************************
std::vector<VertexDescriptor> graph::find_vertices_inside_box(const Eigen::Vector3d& minBnds, const Eigen::Vector3d& maxBnds)
{
  std::vector<VertexDescriptor> vertsInBox;

  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);
  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    Eigen::Vector3d pos = (*adjList_)[*it].pos;
    if(pos(0) < minBnds(0) || pos(0) > maxBnds(0))
      continue;
    if(pos(1) < minBnds(1) || pos(1) > maxBnds(1))
      continue;
    if(pos(2) < minBnds(2) || pos(2) > maxBnds(2))
      continue;

    vertsInBox.push_back(*it);
  }
  return vertsInBox;
}

// ***************************************************************************
Eigen::Vector3d graph::get_pos(const VertexDescriptor& vertexD)
{
  return (*adjList_)[vertexD].pos;
}

// ***************************************************************************
Eigen::Vector3d graph::get_src_pos(const EdgeDescriptor& edgeD) // source pos
{
  return get_pos( boost::source(edgeD, *adjList_) );
}

// ***************************************************************************
Eigen::Vector3d graph::get_tgt_pos(const EdgeDescriptor& edgeD) // target pos
{
  return get_pos( boost::target(edgeD, *adjList_) );
}

// ***************************************************************************
Eigen::MatrixXd graph::plan_shortest_path(const VertexDescriptor& fromVertex, const VertexDescriptor& toVertex)
{
  positive_edge_weight<EdgeWeightMap> filter(boost::get(boost::edge_weight, *adjList_));
  FilteredGraph fGraph(*adjList_, filter);

  std::vector<VertexDescriptor> pM(boost::num_vertices(fGraph));
  std::vector<double> dM(boost::num_vertices(fGraph));
  try 
  {
    // call astar named parameter interface
    boost::astar_search ( fGraph, fromVertex, 
                          distance_heuristic<FilteredGraph, double> (toVertex, adjList_),
                          boost::predecessor_map( boost::make_iterator_property_map(pM.begin(), boost::get(boost::vertex_index, fGraph)) )
                          .distance_map( boost::make_iterator_property_map(dM.begin(), boost::get(boost::vertex_index, fGraph)) )
                          .visitor( goal_visitor<FilteredGraph, VertexDescriptor>(toVertex) ) );
  
  } 
  catch(found_goal fG) 
  { 
    // found a path to the goal
    std::list<Eigen::Vector3d> shortestPath;
    for(VertexDescriptor vert = toVertex;; vert = pM[vert]) 
    {
      shortestPath.push_front( get_pos(vert) );
      if(pM[vert] == vert)
        break;
    }

    Eigen::MatrixXd shortestPathEig(shortestPath.size(),3);

    int itr = 0;
    for(Eigen::Vector3d& vert: shortestPath)
    {
      shortestPathEig.row(itr) = vert;
      itr++;
    }

    return shortestPathEig;
  }
  
  return Eigen::MatrixXd(0,0);
}

// ***************************************************************************
void graph::add_avoid_frontier(const Eigen::Vector3d& avoidFrontier)
{
  if( avoidFrontiers_.size() >= maxNAvoidFrontiers_ )
    clear_avoid_frontiers();

  avoidFrontiers_.push_back(avoidFrontier);
}

// ***************************************************************************
void graph::remove_avoid_frontier(const Eigen::Vector3d& avoidFrontier)
{
  for(int i=0; i<avoidFrontiers_.size(); i++)
    if( ( avoidFrontiers_[i] - avoidFrontier ).lpNorm<1>() < 1e-3 )
    {
      avoidFrontiers_.erase(avoidFrontiers_.begin() + i);
      return; // return cz iterators may shift up after erase operation
    }
}

// ***************************************************************************
void graph::clear_avoid_frontiers()
{
  avoidFrontiers_.resize(0);
}

// ***************************************************************************
Eigen::MatrixXd graph::plan_to_frontier(const VertexDescriptor& fromVertex, const int& nTotalTries, const std::vector<Eigen::MatrixXd>& pathsIn)
{
  std::forward_list<frontier> frontiers = ignore_avoid_frontiers();
  if( frontiers.empty() )
    return Eigen::MatrixXd(0,0);

  Eigen::MatrixXd pathOut(0,0);

  if( (pathsIn.size() > 0) && (pathsIn[0].rows() > 0) )
  {
    std::forward_list<frontier> frontiersSeparated = pull_well_separated_frontiers(frontiers, pathsIn);
    if( !frontiersSeparated.empty() )
      pathOut = plan_to_frontier(fromVertex, nTotalTries, pathsIn, frontiersSeparated);
  }

  if( pathOut.rows() > 1 )
    return pathOut;
  else
    pathOut = plan_to_frontier(fromVertex, nTotalTries, pathsIn, frontiers);

  if( pathOut.rows() > 1 )
    return pathOut;
  else
    return Eigen::MatrixXd(0,0);
}

// ***************************************************************************
Eigen::MatrixXd graph::plan_to_frontier(const VertexDescriptor& fromVertex, const int& nTotalTries, const std::vector<Eigen::MatrixXd>& pathsIn, std::forward_list<frontier> frontiers)
{
  // plan to frontier from the given list
  // only to be used with the main plan_to_frontier function

  double dummyDist;
  frontier closestFrontier = closest_frontier( get_pos(fromVertex), dummyDist, frontiers ); // dummyDist >= 0 cz frontiers is not empty
  Eigen::MatrixXd pathToClosestFrontier = plan_shortest_path(fromVertex, closestFrontier.vertDesc);
  double pathLenToClosestFrontier = path_man::path_len(pathToClosestFrontier);

  Eigen::MatrixXd pathToRecentFrontier;

  int nTries = 0;
  for(frontier& front: frontiers) // assuming frontiers are arranged from most recent to old, choose the most recent from the existing list
  {
    if( nTries >= nTotalTries )
      break;
    
    pathToRecentFrontier = plan_shortest_path(fromVertex, front.vertDesc);

    double pathLenToRecentFrontier = path_man::path_len(pathToRecentFrontier);

    if( pathLenToRecentFrontier > 0 && pathLenToClosestFrontier > 0 )
    { 
      if( pathLenToRecentFrontier < pathLenToClosestFrontier )
        return pathToRecentFrontier;
      else
        return pathToClosestFrontier;
    }
    else if( pathLenToRecentFrontier > 0 && pathLenToClosestFrontier <= 0 )
      return pathToRecentFrontier;
    else if( pathLenToClosestFrontier > 0 && pathLenToRecentFrontier <= 0 )
      return pathToClosestFrontier;
   
    nTries++;
  }
}
// ***************************************************************************
frontier graph::get_recent_frontier(const Eigen::Vector3d& posIn) // returns frontier with <= 0 volGain if none found, avoids avoidFrontiers
{
  std::forward_list<frontier> frontiers = ignore_avoid_frontiers();

  if(frontiers.empty())
  {
    frontier front;
    front.volGain = -1.0;
    return front;
  }

  int sz = 0;
  for(frontier& front: frontiers)
    sz++;
  std::cout << "Num Frontiers: " <<  sz << std::endl;

  for(frontier& front: frontiers) // assuming frontiers are arranged from most recent to old, choose the most recent with enough/atleast min vol gain
  {
    if( front.volGain >= minVolGain_ )
      return front;
  }

  frontier front;
  front.volGain = -1.0;
  return front;
}

// ***************************************************************************
double graph::frontier_cost_alpha(const frontier& frontIn, const Eigen::Vector3d& robPos)
{
  return cGain_[0]*(robPos-get_pos(frontIn.vertDesc)).lpNorm<1>() - cGain_[1]*frontIn.volGain;
}

// ***************************************************************************
bool graph::is_valid(const VertexDescriptor& vertDesc)
{
  bool isValid = false;
  
  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);
  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    if(*it == vertDesc)
      return true;
  }
  return false;
}

// ***************************************************************************
bool graph::is_entrance(const Eigen::Vector3d& ptIn)
{
  if( ((ptIn-entranceMin_).array() > 0).all() && ((ptIn-entranceMax_).array() < 0).all() ) // if ptIn is greater than min and less than max
    return true;

  return false;
}

// ***************************************************************************
bool graph::is_empty_frontiers()
{
  return frontiers_.empty();
}

// ***************************************************************************
void graph::set_min_dist_nodes(const double& dist)
{
  minDistNodes_ = dist;
}

// ***************************************************************************
double graph::get_min_separation_robots()
{
  return minSeparationRobots_;
}


// ***************************************************************************
double graph::volumetric_gain(Eigen::Vector3d ptIn, octomap::OcTree* octTree, double sensRange) // stale function
{

  octomap::point3d_list node_centers;

  std::cout << ptIn.transpose() << std::endl;
  //getchar();

  octTree->getUnknownLeafCenters( node_centers, octomap::point3d(ptIn(0)-sensRange, ptIn(1)-sensRange, ptIn(2)-sensRange), octomap::point3d(ptIn(0)+sensRange, ptIn(1)+sensRange, ptIn(2)+sensRange) ); 
  return node_centers.size();

  // assuming sensor FOV is cube around the robot
  octomap::OcTreeKey octKeySrc = octTree->coordToKey( octomap::point3d(ptIn(0),ptIn(1),ptIn(2)) );

  sensRange = sensRange + octTree->getResolution(); // go for an additional voxel for thoroughness
  int depthInVoxels = std::floor( sensRange / octTree->getResolution());

  double volGain = 0.0;

  // start and end key_types (typedef uint16_t) in all dimensions
  octomap::key_type xS = (octKeySrc.k[0])-depthInVoxels;
  octomap::key_type xE = (octKeySrc.k[0])+depthInVoxels;

  octomap::key_type yS = (octKeySrc.k[1])-depthInVoxels;
  octomap::key_type yE = (octKeySrc.k[1])+depthInVoxels;

  octomap::key_type zS = (octKeySrc.k[2])-depthInVoxels;
  octomap::key_type zE = (octKeySrc.k[2])+depthInVoxels;

  // go through all key_types
  octomap::OcTreeKey octKey;
  for(octKey.k[0]=xS; octKey.k[0]<=xE; octKey.k[0]++)
    for(octKey.k[1]=yS; octKey.k[1]<=yE; octKey.k[1]++)
      for(octKey.k[2]=zS; octKey.k[2]<=zE; octKey.k[2]++)
      {
        octomap::OcTreeNode* octNode = octTree->search(octKey);
        if(octNode == NULL) // if the voxel is unseen
          continue;
        if(octNode->getLogOdds() > 0) // if the voxel is occupied
          continue;

        volGain += n_unseen_neighbors(octTree, &octKey);
      }

  return volGain*octTree->getResolution();
}

// ***************************************************************************
int graph::n_unseen_neighbors(octomap::OcTree* octTree, octomap::OcTreeKey* octKey) // stale function
{
  int nN = 0;

  octomap::OcTreeKey key(*octKey);
  key.k[0] += 1; 
  if(octTree->search(key) == NULL)
    nN++;

  key.k[0] -= 2; 
  if(octTree->search(key) == NULL)
    nN++;

  key.k[0] += 1; key.k[1] += 1; 
  if(octTree->search(key) == NULL)
    nN++;

  key.k[1] -= 2; 
  if(octTree->search(key) == NULL)
    nN++;

  key.k[1] += 1; key.k[2] += 1; 
  if(octTree->search(key) == NULL)
    nN++;

  key.k[2] -= 2; 
  if(octTree->search(key) == NULL)
    nN++;

  return nN;
}


