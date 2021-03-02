#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <vector>
#include <random>
#include <chrono>
#include <forward_list>
#include "disp.h"

#include "octomap_man.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
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
struct frontier
{
  VertexDescriptor vertDesc;
  double volGain;
};

// ***************************************************************************
class graph
{

private: 
  BiDirectionalGraph* adjList_;

  double minBnds_[3];
  double maxBnds_[3];

  double radNear_;
  double radNearest_;
  double radRob_; // robot radius
  double minVolGain_;
  
  octomap_man* octMan_;

  std::forward_list<frontier> frontiers_;
  double minManDistFrontier_;

  std::string frameId_;
 
  // if a node in the graph is removed, the iterators may change, frontier nodes cannot be tracked using VertexIterator in that case
  // Preference 1. Don't remove anything from the graph
  // Preference 2. Keep the vertices, only remove edges
  // Preference 3. Go through all vertices again and populate frontierVerts_ again
 
public:
  ~graph();
  graph(Eigen::Vector3d posRoot, double radNear, double radNearest, double radRob, double minVolGain, std::string frameId, octomap_man* octMan, double);
  
  bool add_vertex(const gvert);
  bool u_coll(const gvert, const gvert);  
  void publish_viz(ros::Publisher&);
  void publish_frontiers(ros::Publisher&);
  void update_octomap(DynamicEDTOctomap*, octomap::OcTree*);

  std::vector<VertexDescriptor> find_vertices_inside_box(const Eigen::Vector3d&, const Eigen::Vector3d&);
  Eigen::MatrixXd plan_shortest_path(const VertexDescriptor& fromVertex, const VertexDescriptor& toVertex);
  frontier get_best_frontier();

  double volumetric_gain(Eigen::Vector3d, octomap::OcTree*, double);
  int n_unseen_neighbors(octomap::OcTree*, octomap::OcTreeKey*);
  void update_frontiers_vol_gain();
  bool add_path(Eigen::MatrixXd& path, bool containFrontier);
  Eigen::Vector3d get_pos(const VertexDescriptor&);
  double closestFrontierManDist(const Eigen::Vector3d& ptIn);
};

// ***************************************************************************
template <class Graph, class CostType>
class distance_heuristic : public boost::astar_heuristic< Graph, CostType >
{
private:
  VertexDescriptor goalVert_;
  BiDirectionalGraph* adjList_;
public:
  distance_heuristic(VertexDescriptor goalVert, BiDirectionalGraph* adjList): goalVert_(goalVert), adjList_(adjList) {}
  CostType operator()(const VertexDescriptor& u)
  {
    return ( (*adjList_)[u].pos - (*adjList_)[goalVert_].pos ).norm();
  }
};

// ***************************************************************************
struct found_goal {}; // exception for search termination

// ***************************************************************************
// visitor that terminates when we find the goal
template <class Graph, class Vertex>
class goal_visitor : public boost::default_astar_visitor
{
private:
  Vertex goalVert_;
public:
  goal_visitor(Vertex goalVert) : goalVert_(goalVert) {}
  void examine_vertex(const Vertex& u, const Graph& g) 
  {
    if(u == goalVert_)
      throw found_goal();
  }
};

#endif
