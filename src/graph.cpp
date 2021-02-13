#include "graph.h"
#include "rrt.h"

// ***************************************************************************
graph::graph(Eigen::Vector3d posRoot, double radNear, double radRob, double minVolGain, std::string frameId, octomap_man* octMan)
{
  adjList_ = new BiDirectionalGraph;

  gvert vertRoot;

  vertRoot.pos = posRoot;
  vertRoot.commSig = 0; // assumes 0 signal strength at launch
  vertRoot.isFrontier = false; // assumes root is not a frontier
  vertRoot.terrain = gvert::FLAT;

  boost::add_vertex(vertRoot, *adjList_);

  radNear_ = radNear;
  radRob_ = radRob;
  frameId_ = frameId;
  minVolGain_ = minVolGain;

  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);

  std::cout << "CREATED GRAPH OBJECT" << std::endl;
  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    VertexDescriptor vertDesc = *it;
    std::cout << (*adjList_)[vertDesc].pos << std::endl;
  }

  octMan_ = octMan;

  //TODO: radRob_ and radNear_ assignments 
}

// ***************************************************************************
bool graph::add_vertex(const gvert vertIn)
{
  bool success = false;
  bool vertexPresent = false;
  VertexDescriptor vertInDesc;

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
      boost::add_edge( vertInDesc, *it, dist, *adjList_ ); //vertIn is added if not already present
      success = true;
      vertexPresent = true;
      //std::cout << "Help1" << std::endl;
    }
    else if( !u_coll(vertIn, (*adjList_)[*it]) && vertexPresent )
      boost::add_edge( vertInDesc, *it, dist, *adjList_ );

    //std::cout << "Help2" << std::endl;
  }

  if(vertIn.isFrontier && success)
  {
    frontier front;
    front.vertDesc = vertInDesc;
    front.volGain = 0.0;
    frontiers_.push_front(front);
  }

  return success;
}

// ***************************************************************************
bool graph::u_coll(const gvert vert1, const gvert vert2)
{
  return octMan_->u_coll(vert1.pos, vert2.pos);
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
  vertices.action = visualization_msgs::Marker::ADD;
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

    color.r = 0; color.g = 1; color.b = 0; color.a = 1;
    edges.colors.push_back(color);

    vertexDesc = boost::target(*it, *adjList_); // target vertex
    vertex2.x = (*adjList_)[vertexDesc].pos(0);
    vertex2.y = (*adjList_)[vertexDesc].pos(1);
    vertex2.z = (*adjList_)[vertexDesc].pos(2);
    edges.points.push_back(vertex2);

    color.r = 0; color.g = 1; color.b = 0; color.a = 1;
    edges.colors.push_back(color);
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
void graph::update_frontiers_vol_gain()
{
  if(frontiers_.empty())
    return;

  int sz = 0;
  for(frontier& front: frontiers_)
    sz++;
  std::cout << "Num Frontiers: " <<  sz << std::endl;
  for(frontier& front: frontiers_)
    front.volGain = octMan_->volumetric_gain( (*adjList_)[front.vertDesc].pos );
  double minV = minVolGain_;
  frontiers_.remove_if( [&minV](const frontier& front) -> bool {return (front.volGain < minV);} ); // lambda expression
}

// ***************************************************************************
bool graph::add_path(Eigen::MatrixXd& path, bool containFrontier)
{
  bool success= false;

  gvert vert;
  for(int i=0; i<path.rows(); i++)
  {
    vert.pos = path.row(i);
    vert.commSig = 0;
    
    if( containFrontier && i==(path.rows()-1) )
      vert.isFrontier = true;
    else
      vert.isFrontier = false;

    vert.terrain = gvert::UNKNOWN;

    std::cout << "Adding vertex" << std::endl;
    success = add_vertex(vert);
  }

  return success;
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


