#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <ompl/datastructures/NearestNeighbors.h>
#include "view_motion_planner/viewpose.h"
#include "view_motion_planner/robot_manager.h"
#include "view_motion_planner/octree_manager.h"
#include "view_motion_planner/views_voxels_motion.h"
#include "roi_viewpoint_planner_msgs/VmpConfig.h"
#include <gurobi_c++.h>
#include <boost/graph/graphviz.hpp>
#include <fstream>

namespace view_motion_planner
{

using ViewposeGraph = boost::adjacency_list<
boost::hash_setS, // OutEdgeListSelector
boost::vecS, // VertexListSelector
boost::undirectedS, // DirectedSelector
ViewposePtr, // VertexProperty
TrajectoryPtr, // EdgeProperty
boost::no_property, // GraphProperty
boost::listS>; // EdgeListSelector

using Vertex = boost::graph_traits<ViewposeGraph>::vertex_descriptor;
using Edge = boost::graph_traits<ViewposeGraph>::edge_descriptor;

class ViewposeGraphManager;

class VertexUtilityComp
{
private:
  ViewposeGraphManager *graph_manager;

public:
  VertexUtilityComp(ViewposeGraphManager *graph_manager) : graph_manager(graph_manager) {}

  bool operator() (const Vertex &v1, const Vertex &v2) const;
};

class ViewposeGraphManager
{
private:
  const std::string map_frame;
  const std::string ws_frame;
  const std::string pose_frame;

  ViewposeGraph graph;

  boost::shared_mutex graph_mtx;

  std::unordered_map<ViewposePtr, Vertex> vertex_map;

  //std::unordered_map<Vertex, int> vertex_id; // Used by optimizer
  //std::unordered_map<int, Vertex> id_vertex; // Used by optimizer
  // vertex_index is used to map unvisited vertex to index in optimizer matrix
  std::unordered_map<Vertex, int> vertex_index; // Used by optimizer

  std::unordered_set<std::size_t> unvisited_vertices; // Used by optimizer
  //int unique_vertex_id; // Used by optimizer

  std::shared_ptr<RobotManager> robot_manager;
  std::shared_ptr<OctreeManager> octree_manager;
  std::shared_ptr<ViewsVoxelsMotion> graph_optimizer; // Used by optimizer

  std::unique_ptr<ompl::NearestNeighbors<ViewposePtr>> neighbors_joint;
  std::unique_ptr<ompl::NearestNeighbors<ViewposePtr>> neighbors_vpsim;

  Vertex current_start_vertex;
  ViewposePtr highest_ig_pose = nullptr;
  ViewposePtr highest_util_pose = nullptr;
  rviz_visual_tools::RvizVisualToolsPtr vt_searched_graph;

  typedef boost::heap::fibonacci_heap<Vertex, boost::heap::compare<VertexUtilityComp>> ValueHeap;
  typedef std::unordered_map<Vertex, ValueHeap::handle_type> VertexHandleMap;
  std::unordered_set<Vertex> expanded_vertices;
  size_t current_start_vertex_number;

  ValueHeap priorityQueue;

  ros::Publisher viewArrowVisPub;
  ros::Publisher poseArrayPub;

  ros::Publisher inflatedOctomapPub;
  ros::Publisher viewVoxelPub;

public:
  ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager,
                       const std::shared_ptr<OctreeManager> &octree_manager,
                       const std::shared_ptr<ViewsVoxelsMotion> &graph_optimizer,
                       const rviz_visual_tools::RvizVisualToolsPtr &vt_searched_graph,
                       const std::string &map_frame, const std::string &ws_frame, const std::string &pose_frame);

  double getVertexDistancePose(ViewposePtr a, ViewposePtr b);
  double getVertexDistanceJoints(ViewposePtr a, ViewposePtr b);
  double getVertexDistanceVpSimilarity(ViewposePtr a, ViewposePtr b);

  double getNearestVertexDistanceByVpSimilarity(const ViewposePtr &vp);

  boost::shared_mutex &getGraphMutex()
  {
    return graph_mtx;
  }

  ViewposeGraph& getGraph()
  {
    return graph;
  }

  const ViewposeGraph& getGraph() const
  {
    return graph;
  }

  Vertex addViewpose(const ViewposePtr &vp)
  {
    Vertex v = boost::add_vertex(vp, graph);
    neighbors_joint->add(vp);
    neighbors_vpsim->add(vp);
    vertex_map[vp] = v;
    // add a value to unvisited_vertices
    unvisited_vertices.insert(v); // Used by optimizer
    return v;
  }

  void removeVertex(const Vertex &v)
  {
    const ViewposePtr vp = graph[v];
    vertex_map.erase(vp);
    unvisited_vertices.erase(v); // Used by optimizer
    neighbors_joint->remove(vp);
    neighbors_vpsim->remove(vp);
    boost::remove_vertex(v, graph);
  }

  void clear()
  {
    boost::unique_lock lock(graph_mtx);
    graph.clear();
    neighbors_joint->clear();
    neighbors_vpsim->clear();
    highest_ig_pose = nullptr;
    highest_util_pose = nullptr;
    vertex_map.clear();
    expanded_vertices.clear();
    current_start_vertex_number = 0;
    priorityQueue.clear();
    ROS_INFO_STREAM("Graph cleared");

    //unique_vertex_id = 0; // Used by optimizer
    unvisited_vertices.clear(); // Used by optimizer
    vertex_index.clear(); // Used by optimizer
    //vertex_id.clear(); // Used by optimizer
    //id_vertex.clear(); // Used by optimizer

    graph_optimizer->clear(); // Used by optimizer
    ROS_INFO_STREAM("Optimizer cleared");
  }

  size_t connectNeighbors(const Vertex &v, size_t num_neighbors, double max_traj_length, double traj_step = 0.1);

  void publishViewposeVisualizations(const std::vector<ViewposePtr> &vps);

  void visualizeGraph(bool visualize_expanded, bool visualize_unexpanded);
  void visualizeOptimizedGraph(bool visualize_expanded, bool visualize_unexpanded);


  void publishViewVoxels(const std::shared_ptr<octomap_vpp::InflatedRoiOcTree> octree, const octomap::KeySet roi_voxels, const octomap::KeySet occ_voxels, const octomap::KeySet free_voxels);
  void publishViewVoxels(const std::shared_ptr<octomap_vpp::RoiOcTree> octree, const octomap::KeySet roi_voxels, const octomap::KeySet occ_voxels, const octomap::KeySet free_voxels);

  // octree pointer
  void publishInflatedOctomap(const std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflated_octree);

  void initStartPose(const Vertex &v);

  void markAsVisited(const Vertex &v);

  void cleanupAfterMove();

  bool expand();

  Vertex getCurrentStartVertex()
  {
    return current_start_vertex;
  }

  std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr> getNextTrajectory();

  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> getNextTrajectories(double cost_limit);

  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> getOptimizedTrajectories(double cost_limit);

  static bool fixTrajectoryStartPoint(const robot_trajectory::RobotTrajectoryPtr &traj, const moveit::core::RobotStatePtr &state);

  void motionCostMatrix(); // Used by optimizer

  void motionCostConstraints(); // Used by optimizer

  bool viewVoxelMatrix(); // Used by optimizer

  bool optimize(); // Used by optimizer

  void remove_visited_vertices(const Vertex &v) // Used by optimizer
  {
    // // if vertex is present, then remove it
    // if (vertex_index.find(v) != vertex_index.end())
    // {
    //   //id_vertex.erase(vertex_id[v]);
    //   vertex_index.erase(v);
    // }
    // if vertex is present, then remove it
    if (unvisited_vertices.find(v) != unvisited_vertices.end())
    {
      unvisited_vertices.erase(v);
    }
  }

  void add_current_vertex(const Vertex &v)
  {
    unvisited_vertices.insert(v);
  }
};

// // define the struct to hold Vertex and information gain. replace by std::size_t or view_motion_planner::Vertex
// struct VertexInfo {
//     Vertex vertex;
//     int information_gain;

//     VertexInfo(const Vertex& v, int gain)
//         : vertex(v), information_gain(gain) {}
// };

// // comparator function to sort in decreasing order of information gain
// bool compareByInformationGain(const VertexInfo& a, const VertexInfo& b) {
//     return a.information_gain > b.information_gain;
// }


} // namespace view_motion_planner

// custom hash function for octomap::OcTreeKey
namespace std {
  template <>
  struct hash<octomap::OcTreeKey> {
    std::size_t operator()(const octomap::OcTreeKey& k) const {
      // Implement a hash combining function for OcTreeKey members
      return std::hash<uint16_t>()(k.k[0]) ^ (std::hash<uint16_t>()(k.k[1]) << 1) ^ (std::hash<uint16_t>()(k.k[2]) << 2);
    }
  };
}
