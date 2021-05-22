#include "graph.h"
#include "rrt.h"

// ***************************************************************************
graph::graph(Eigen::Vector3d posRoot, double radNear, double radNearest, double minVolGain, std::string frameId, octomap_man* octMan, double minManDistFrontier, const std::vector<double>& entranceMin, const std::vector<double>& entranceMax, const std::vector<double>& cGain, const double& manDistAvoidFrontier)
{
  adjList_ = new BiDirectionalGraph;

  gvert vertRoot;

  vertRoot.pos = posRoot;
  vertRoot.commSig = 0; // assumes 0 signal strength at launch
  vertRoot.isFrontier = false; // assumes root is not a frontier
  vertRoot.terrain = gvert::FLAT;

  homeVert_ = boost::add_vertex(vertRoot, *adjList_);
  isHomeVertConnected_ = false;

  radNearest_ = radNearest;
  radNear_ = radNear;
  frameId_ = frameId;
  minVolGain_ = minVolGain;
  minManDistFrontier_ = minManDistFrontier;
  manDistAvoidFrontier_ = manDistAvoidFrontier;
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
bool graph::add_vertex(const gvert& vertIn)
{
  VertexDescriptor vertInDesc;
  return add_vertex(vertIn,  vertInDesc);
}

// ***************************************************************************
bool graph::add_vertex(const gvert& vertIn, VertexDescriptor& vertInDesc)
{
  // TODO: Consider using manhattan distance as the edge cost to speed up 
  bool success = false;
  bool vertexPresent = false;

  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);

  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    double dist = (vertIn.pos - (*adjList_)[*it].pos).squaredNorm();
    if( dist > pow(radNear_,2) )
      continue;
    //std::cout << "Adding vertex " << vertIn.pos.transpose() << " to " << (*adjList_)[*it].pos.transpose() << std::endl;
    //std::cout << "Collision check: " << u_coll(vertIn, (*adjList_)[*it]) << std::endl;
    //std::cout << "Source vertex already added: " << vertexPresent << std::endl;
    if( !u_coll(vertIn, (*adjList_)[*it]) && !vertexPresent )
    {
      //std::cout << "Help0" << std::endl;
      vertInDesc = boost::add_vertex(vertIn, *adjList_);
      boost::add_edge( vertInDesc, *it, sqrt(dist), *adjList_ ); //vertIn is added if not already present
      success = true;
      vertexPresent = true;
      //std::cout << "Help1" << std::endl;
    }
    else if( !u_coll(vertIn, (*adjList_)[*it]) && vertexPresent )
      boost::add_edge( vertInDesc, *it, sqrt(dist), *adjList_ );

    //std::cout << "Help2" << std::endl;
  }

  if(vertIn.isFrontier && success)
  {
    frontier front;
    front.vertDesc = vertInDesc;
    front.volGain = 0.0;
    frontiers_.push_front(front);
  }

  if(!isHomeVertConnected_ && boost::in_degree(homeVert_, *adjList_) < 1 ) // if home was not connected and is still not connected (assuming the node is not added above unless connected to home node)
  {
    vertInDesc = boost::add_vertex(vertIn, *adjList_);
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
  // Function derived from http://www.3dkingdoms.com/weekly/weekly.php?a=21

  Eigen::Vector3d mExtent = (maxPt - minPt) * 0.5;
	Eigen::Vector3d mM = (maxPt + minPt) * 0.5;

	// Put line in box space
	Eigen::Vector3d LB1 = get_src_pos(edgeD) - mM;
	Eigen::Vector3d LB2 = get_tgt_pos(edgeD) - mM;

	// Get line midpoint and extent
	Eigen::Vector3d LMid = (LB1 + LB2) * 0.5; 
	Eigen::Vector3d L = (LB1 - LMid);
	Eigen::Vector3d LExt = L.cwiseAbs();

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
  visualization_msgs::Marker edges;
  edges.header.frame_id = frameId_;
  edges.header.stamp = ros::Time::now();
  edges.ns = "edges";
  edges.id = 0;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.action = visualization_msgs::Marker::ADD;
  edges.pose.orientation.w = 1; 
  scale.x = 0.025;
	scale.y = 0.025;
	scale.z = 0.025;
	edges.scale = scale;
  
  geometry_msgs::Point vertex1;
  geometry_msgs::Point vertex2;
  
  std::pair<EdgeIterator, EdgeIterator> edgeItr = boost::edges(*adjList_);
  for(EdgeIterator it=edgeItr.first; it!=edgeItr.second; ++it)
  {
    VertexDescriptor vertexDesc = boost::source(*it, *adjList_); // source vertex
    vertex1.x = (*adjList_)[vertexDesc].pos(0);
    vertex1.y = (*adjList_)[vertexDesc].pos(1);
    vertex1.z = (*adjList_)[vertexDesc].pos(2);
    edges.points.push_back(vertex1);

    vertexDesc = boost::target(*it, *adjList_); // target vertex
    vertex2.x = (*adjList_)[vertexDesc].pos(0);
    vertex2.y = (*adjList_)[vertexDesc].pos(1);
    vertex2.z = (*adjList_)[vertexDesc].pos(2);
    edges.points.push_back(vertex2);

    if( get(boost::edge_weight, *adjList_, *it) < 0 )
    {
      color.r = 0; color.g = 1; color.b = 0; color.a = 1;
      edges.colors.push_back(color);
      edges.colors.push_back(color);
    }
    else
    {
      color.r = 1; color.g = 0; color.b = 0; color.a = 1;
      edges.colors.push_back(color);
      edges.colors.push_back(color);
    }
  }

  // vertices + edges
  visualization_msgs::MarkerArray vizMsg;
  vizMsg.markers.push_back(vertices);
  vizMsg.markers.push_back(edges);

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

  int idFrontViz = -1;
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
  if( containFrontier && !frontiers_.empty() && (closest_frontier_man_dist(frontPt) < minManDistFrontier_) )
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
    success = add_vertex(vert);
  }

  return success;
}

// ***************************************************************************
frontier graph::closest_frontier(const Eigen::Vector3d& ptIn, double& dist) // closest frontier to the ptIn
{
  if(frontiers_.empty())
  {
    frontier front;
    dist = -1.0;
    front.volGain = -1.0;
    return front;
  }  

  frontier closestFront = frontiers_.front(); // pick the first frontier as the best guess
  double minDist = (get_pos(closestFront.vertDesc) - ptIn).lpNorm<1>();

  for(frontier& front: frontiers_)
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
  closest_frontier(ptIn, dist);
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
  avoidFrontiers_.push_back(avoidFrontier);
}

// ***************************************************************************
void graph::clear_avoid_frontiers()
{
  avoidFrontiers_.resize(0);
}

// ***************************************************************************
Eigen::MatrixXd graph::plan_to_frontier(const VertexDescriptor& fromVertex, const int& nTotalTries)
{
  std::forward_list<frontier> frontiers = ignore_avoid_frontiers();
  if(frontiers.empty())
    return Eigen::MatrixXd(0,0);

  double dummyDist;
  frontier closestFrontier = closest_frontier( get_pos(fromVertex), dummyDist );
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

  return Eigen::MatrixXd(0,0);
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


