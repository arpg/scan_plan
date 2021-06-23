#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <vector>
#include <random>
#include <chrono>
#include <forward_list>
#include "disp.h"

#include "octomap_man.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"

#include "path_man.h"

// ***************************************************************************
// edge predicate function to return true if edge weight is positive 
template <typename EdgeWeightMap>
struct positive_edge_weight {
  positive_edge_weight() { }
  positive_edge_weight(EdgeWeightMap weight) : m_weight(weight) { }
  template <typename Edge>
  bool operator()(const Edge& e) const {
    return 0 < get(m_weight, e);
  }
  EdgeWeightMap m_weight;
};

// ***************************************************************************
struct gvert
{
  enum Terrain { FLAT, CLIFF, STEPS, UNKNOWN };

  Eigen::Vector3d pos; // position
  int commSig; // communication strength (0-100)
  bool isFrontier; // if the node is a frontier
  
  Terrain terrain;
};

// *************************************************************************** // TODO: Checkout other graph types, mutable, undirected? Any faster?
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, gvert, EdgeWeightProperty > BiDirectionalGraph;
typedef boost::property_map<BiDirectionalGraph, boost::edge_weight_t>::type EdgeWeightMap;
typedef boost::filtered_graph<BiDirectionalGraph, positive_edge_weight<EdgeWeightMap> > FilteredGraph;
typedef boost::graph_traits<BiDirectionalGraph>::edge_iterator EdgeIterator;
typedef boost::graph_traits<BiDirectionalGraph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<BiDirectionalGraph>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<BiDirectionalGraph>::edge_descriptor EdgeDescriptor;

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
  double minDistNodes_;
  double minVolGain_;
 
  int maxEdgesPerVertex_;
  
  octomap_man* octMan_;

  std::forward_list<frontier> frontiers_;
  double minManDistFrontier_;

  std::string frameId_;
  VertexDescriptor homeVert_;
  bool isHomeVertConnected_ = false;

  Eigen::Vector3d entranceMin_;
  Eigen::Vector3d entranceMax_;

  std::vector<double> cGain_;

  std::vector<Eigen::Vector3d> avoidFrontiers_; // only valid for get_best_frontier function for now
  double manDistAvoidFrontier_;

  std::vector<VertexDescriptor> vertsOutDesc_; // preventing memory allocation in add_vertex functions
  std::vector<double> vertsOutDist_; // preventing memory allocation in add_vertex functions

 
  // if a node in the graph is removed, the iterators may change, frontier nodes cannot be tracked using VertexIterator in that case
  // Preference 1. Don't remove anything from the graph
  // Preference 2. Keep the vertices, only remove edges
  // Preference 3. Go through all vertices again and populate frontierVerts_ again
 
public:
  ~graph();
  graph(Eigen::Vector3d posRoot, double radNear, double minDistNodes, int maxEdgesPerVertex, double minVolGain, std::string frameId, octomap_man* octMan, double, const std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const double& );
  
  bool add_vertex(const gvert&);
  bool add_vertex(const gvert&, VertexDescriptor&);
  bool u_coll(const gvert, const gvert);  
  void publish_viz(ros::Publisher&);
  void publish_frontiers(ros::Publisher&);
  void update_octomap(DynamicEDTOctomap*, octomap::OcTree*);

  std::vector<VertexDescriptor> find_vertices_inside_box(const Eigen::Vector3d&, const Eigen::Vector3d&);
  Eigen::MatrixXd plan_shortest_path(const VertexDescriptor& fromVertex, const VertexDescriptor& toVertex);
  frontier get_recent_frontier(const Eigen::Vector3d&);

  double volumetric_gain(Eigen::Vector3d, octomap::OcTree*, double);
  int n_unseen_neighbors(octomap::OcTree*, octomap::OcTreeKey*);
  void update_frontiers_vol_gain();
  bool add_path(Eigen::MatrixXd& path, bool containFrontier);
  Eigen::Vector3d get_pos(const VertexDescriptor&);
  double closest_frontier_man_dist(const Eigen::Vector3d& ptIn);
  bool is_valid(const VertexDescriptor& vertDesc);

  Eigen::MatrixXd plan_home(const VertexDescriptor&);
  bool is_entrance(const Eigen::Vector3d&);
  bool is_empty_frontiers();
  double frontier_cost_alpha(const frontier&, const Eigen::Vector3d&);

  void add_avoid_frontier(const Eigen::Vector3d&);
  void remove_avoid_frontier(const Eigen::Vector3d&);
  void clear_avoid_frontiers();
  std::forward_list<frontier> ignore_avoid_frontiers();
  bool is_avoid_frontier(const Eigen::Vector3d&);
  void remove_frontier(const Eigen::Vector3d&);

  bool in_bounds(const EdgeDescriptor& edgeD, const Eigen::Vector3d& minPt, const Eigen::Vector3d& maxPt);
  void update_occupancy(const Eigen::Vector3d& minPt, const Eigen::Vector3d& maxPt, const bool& occupiedOnly);
  Eigen::Vector3d get_src_pos(const EdgeDescriptor& edgeD);
  Eigen::Vector3d get_tgt_pos(const EdgeDescriptor& edgeD);
  frontier closest_frontier(const Eigen::Vector3d& ptIn, double& dist);
  Eigen::MatrixXd plan_to_frontier(const VertexDescriptor& fromVertex, const int& nTotalTries);
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
