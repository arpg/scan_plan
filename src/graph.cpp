#include "graph.h"
#include "rrt.h"

// ***************************************************************************
graph::graph(Eigen::Vector3d posRoot, double radNear)
{
  adjList_ = new BiDirectionalGraph;

  gvert vertRoot;

  vertRoot.pos = posRoot;
  vertRoot.commSig = 0; // assumes 0 signal strength at launch
  vertRoot.isFrontier = false; // assumes root is not a frontier
  vertRoot.terrain = gvert::FLAT;

  boost::add_vertex(vertRoot, *adjList_);

  radNear_ = radNear;

  std::pair<VertexIterator, VertexIterator> vertItr = vertices(*adjList_);

  std::cout << "CREATED GRAPH OBJECT" << std::endl;
  for(VertexIterator it=vertItr.first; it!=vertItr.second; ++it)
  {
    VertexDescriptor vertDesc = *it;
    std::cout << (*adjList_)[vertDesc].pos << std::endl;
  }

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

    if( !u_coll(vertIn, (*adjList_)[*it]) && !vertexPresent )
    {
      vertInDesc = boost::add_vertex(vertIn, *adjList_);
      boost::add_edge( vertInDesc, *it, dist, *adjList_ ); //vertIn is added if not already present
      success = true;
      vertexPresent = true;
    }
    else if( !u_coll(vertIn, (*adjList_)[*it]) && vertexPresent )
      boost::add_edge( vertInDesc, *it, dist, *adjList_ );

    if(vertIn.isFrontier && success)
      frontierVerts_.push_front(vertInDesc);
  }

  return success;
}

// ***************************************************************************
bool graph::u_coll(const gvert vert1, const gvert vert2)
{
  double delLambda = 0.2;

  double lambda = 0;
  Eigen::Vector3d pos;
  while(lambda <= 1)
  {
    pos = (1-lambda)*vert1.pos + lambda*vert2.pos; 

    if ( rrt::u_coll_octomap(pos, radRob_, octDist_) )
      return true;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
void graph::publish_viz(ros::Publisher& vizPub, std::string frameId)
{
  // vertices
  visualization_msgs::Marker vertices;
  vertices.header.frame_id = frameId;
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
  edges.header.frame_id = frameId;
  edges.header.stamp = ros::Time::now();
  edges.ns = "edges";
  edges.id = 0;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.action = visualization_msgs::Marker::ADD;
  edges.pose.orientation.w = 1; 
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

    color.r = 0; color.g = 1; color.b = 0; color.a = 1;
    edges.colors.push_back(color);
  }

  // vertices + edges
  visualization_msgs::MarkerArray vizMsg;
  vizMsg.markers.push_back(vertices);
  vizMsg.markers.push_back(edges);
  
  vizPub.publish(vizMsg);
}

// ***************************************************************************
graph::~graph()
{
  delete adjList_;
}


// ***************************************************************************

// ***************************************************************************


