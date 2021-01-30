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
graph::~graph()
{
  delete adjList_;
}


// ***************************************************************************

// ***************************************************************************


