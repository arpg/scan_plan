#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <random>
#include <chrono>
#include <forward_list>
#include "disp.h"

#include "dynamicEDT3D/dynamicEDTOctomap.h"
#include "matplotlib-cpp/matplotlibcpp.h"

#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"

// ***************************************************************************
struct gvert
{
  enum Terrain { FLAT, CLIFF, STEPS, UNKNOWN };

  Eigen::Vector3d pos; // position
  int commSig; // communication strength (0-100)
  bool isFrontier; // if the node is a frontier
  
  Terrain terrain;
};

// ***************************************************************************
typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, gvert, EdgeWeightProperty > BiDirectionalGraph;
typedef boost::graph_traits<BiDirectionalGraph>::edge_iterator EdgeIterator;
typedef boost::graph_traits<BiDirectionalGraph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<BiDirectionalGraph>::vertex_descriptor VertexDescriptor;

// ***************************************************************************

class graph
{

private: 
  BiDirectionalGraph* adjList_;

  double minBnds_[3];
  double maxBnds_[3];

  double radNear_;
  double radRob_; // robot radius
  
  DynamicEDTOctomap* octDist_;

  std::forward_list<VertexDescriptor> frontierVerts_;
 
  // if a node in the graph is removed, the iterators may change, frontier nodes cannot be tracked using VertexIterator in that case
  // Preference 1. Don't remove anything from the graph
  // Preference 2. Keep the vertices, only remove edges
  // Preference 3. Go through all vertices again and populate frontierVerts_ again
 
public:
  ~graph();
  graph(Eigen::Vector3d, double);
  
  bool add_vertex(const gvert);
  bool u_coll(const gvert, const gvert);  
  void publish_viz(ros::Publisher&, std::string);
};

#endif
