#include "view_motion_planner/viewpose_graph.h"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
//#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
//#include <ompl/datastructures/NearestNeighborsLinear.h>
//#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

namespace view_motion_planner
{

bool VertexUtilityComp::operator() (const Vertex &v1, const Vertex &v2) const
{
  const ViewposePtr &vp1 = graph_manager->getGraph()[v1];
  const ViewposePtr &vp2 = graph_manager->getGraph()[v2];
  //return vp1->accumulated_cost < vp2->accumulated_cost;
  return vp1->accumulated_infogain > vp2->accumulated_infogain;
}

ViewposeGraphManager::ViewposeGraphManager(const std::shared_ptr<RobotManager> &robot_manager,
                                           const std::shared_ptr<OctreeManager> &octree_manager,
                                           const std::shared_ptr<ViewsVoxelsMotion> &graph_optimizer,
                                           const rviz_visual_tools::RvizVisualToolsPtr &vt_searched_graph,
                                           const std::string &map_frame, const std::string &ws_frame, const std::string &pose_frame)
  : robot_manager(robot_manager), octree_manager(octree_manager), graph_optimizer(graph_optimizer),
    neighbors_joint(new ompl::NearestNeighborsGNAT<ViewposePtr>()), neighbors_vpsim(new ompl::NearestNeighborsGNAT<ViewposePtr>()),
    vt_searched_graph(vt_searched_graph), current_start_vertex_number(0), priorityQueue(VertexUtilityComp(this)),
    map_frame(map_frame), ws_frame(ws_frame), pose_frame(pose_frame)
{
  // vertexes stored in joint space
  neighbors_joint->setDistanceFunction(std::bind(&ViewposeGraphManager::getVertexDistanceJoints, this, std::placeholders::_1, std::placeholders::_2));
  // vertexes stored in viewpoint similarity space
  neighbors_vpsim->setDistanceFunction(std::bind(&ViewposeGraphManager::getVertexDistanceVpSimilarity, this, std::placeholders::_1, std::placeholders::_2));

  ros::NodeHandle nh;
  // viewArrowVisPub publishes visualization of viewposes as arrows from the viewpoint to the target voxel with the color indicating the target type
  viewArrowVisPub = nh.advertise<visualization_msgs::MarkerArray>("roi_vp_marker", 1);
  // poseArrayPub publishes the array of actual viewposes which are published as markers
  poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("vp_array", 1);

  // to visualize the inflated octomap
  inflatedOctomapPub = nh.advertise<octomap_msgs::Octomap>("inflatedOctomap", 1, true);
  // to visualize the voxels selected by the view planner
  viewVoxelPub = nh.advertise<visualization_msgs::Marker>("viewVoxels", 1, true);
}

double ViewposeGraphManager::getVertexDistancePose(ViewposePtr a, ViewposePtr b)
{
  const geometry_msgs::Point &pa = a->pose.position;
  const geometry_msgs::Point &pb = b->pose.position;
  double x = pb.x - pa.x;
  double y = pb.y - pa.y;
  double z = pb.z - pa.z;
  return x*x + y*y + z*z;
}

double ViewposeGraphManager::getVertexDistanceJoints(ViewposePtr a, ViewposePtr b)
{
  return a->state->distance(*(b->state), robot_manager->getJointModelGroup());
}

double ViewposeGraphManager::getVertexDistanceVpSimilarity(ViewposePtr a, ViewposePtr b)
{
  return octree_manager->computeViewpointDissimilarity(a, b);
}

double ViewposeGraphManager::getNearestVertexDistanceByVpSimilarity(const ViewposePtr &vp)
{
  if (neighbors_vpsim->size() == 0)
    return std::numeric_limits<double>::infinity(); //return infinity if neighbors_vpsim is empty
    
  ViewposePtr nb = neighbors_vpsim->nearest(vp);
  return neighbors_vpsim->getDistanceFunction()(vp, nb);
}

size_t ViewposeGraphManager::connectNeighbors(const Vertex &v, size_t num_neighbors, double max_traj_length, double traj_step)
{
  size_t successful_connections = 0;
  std::vector<ViewposePtr> neighbors;

  // if fully connected, connect all neighbors
  if(config.fully_connected)
  {
    neighbors_joint->list(neighbors);
  }
  else
  {
    // even though num_neighbors neighbors are retrieved, they may get rejected due to already visited, traj cost or invalid states
    // so retrieve more neighbors than required, then check for counter and break if it exceeded num_neighbors
    // neighbors_joint->nearestK(graph[v], num_neighbors, neighbors);
    neighbors_joint->nearestK(graph[v], num_neighbors, neighbors);
  }

  for (const ViewposePtr &nb : neighbors)
  {
    // check if max number of neighbors is reached in case of non-fully connected graph
    if ((!config.fully_connected) && (successful_connections >= num_neighbors))
      break;

    Vertex nv = vertex_map.at(nb);
    
    std::pair<Edge, bool> check_edge = boost::edge(v, nv, graph);
    if (check_edge.second) // edge already exists
      continue;

    // if this vertex is already visited, then don't add the edge
    if (unvisited_vertices.count(nv) == 0)
      continue;

    moveit::core::RobotStatePtr from = graph[v]->state;
    moveit::core::RobotStatePtr to = nb->state;

    TrajectoryPtr t(new Trajectory());
    t->cost = from->distance(*to, robot_manager->getJointModelGroup());
    // if fully connected, connect all neighbors regardless of the cost
    if ((t->cost > max_traj_length) && (!config.fully_connected)) // don't compute trajectory
      continue;

    t->traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
    t->bw_traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
    t->traj->addSuffixWayPoint(from, 0);
    t->bw_traj->addPrefixWayPoint(from, 0);
    bool found_traj = true;
    if (t->cost > traj_step) // interpolate points
    {
      size_t num_steps = static_cast<size_t>(t->cost / traj_step);
      double frac_step = 1.0 / num_steps;
      for (size_t i = 1; i < num_steps; i++) // don't check first and last state (assume valid)
      {
        moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*from));
        from->interpolate(*to, i * frac_step, *temp_state, robot_manager->getJointModelGroup());
        temp_state->update();

        bool state_valid = robot_manager->isValidState(temp_state);
        if (!state_valid) // don't add edge
        {
          found_traj = false;
          break;
        }
        t->traj->addSuffixWayPoint(temp_state, 0);
        t->bw_traj->addPrefixWayPoint(temp_state, 0);
      }
    }
    if (found_traj)
    {
      t->traj->addSuffixWayPoint(to, 0);
      t->bw_traj->addPrefixWayPoint(to, 0);

      trajectory_processing::TimeOptimalTrajectoryGeneration trajectory_processor;
      found_traj = (trajectory_processor.computeTimeStamps(*(t->traj), 1.0, 1.0) && trajectory_processor.computeTimeStamps(*(t->bw_traj), 1.0, 1.0));
      if (!found_traj)
        continue;

      t->traj_start_state = from;
      t->bw_traj_start_state = to;

      /*auto [edge, success] = */boost::add_edge(v, nv, t, graph);
      //t->cost = std::accumulate(t->traj->getWayPointDurations().begin(), t->traj->getWayPointDurations().end(), 0);
      // last_collision_check_start_vertex is an indicator of the state of the graph when the last collision check was performed
      // if the state of the graph has changed, then the collision check should be performed again
      t->last_collision_check_start_vertex = current_start_vertex_number;
      successful_connections++;
    }
  }
  return successful_connections;
}

void ViewposeGraphManager::publishViewposeVisualizations(const std::vector<ViewposePtr> &vps)
{
  static size_t last_marker_count = 0;
  visualization_msgs::MarkerArray markers;
  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = map_frame;
  poseArr.header.stamp = ros::Time::now();
  bool first = true;
  std::string ns = "graphPoses";
  for (size_t i = 0; i < vps.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    switch (vps[i]->type)
    {
    case TARGET_FILTER:
      marker.color = COLOR_YELLOW;
      break;
    case TARGET_ROI:
      marker.color = COLOR_RED;
      break;
    case TARGET_OCC:
      marker.color = COLOR_GREEN;
      break;
    case TARGET_BORDER:
      marker.color = COLOR_BLUE;
      break;
    }
    //marker.lifetime = ros::Duration(2);
    marker.points.push_back(vps[i]->pose.position);
    marker.points.push_back(octomap::pointOctomapToMsg(vps[i]->origin));
    markers.markers.push_back(marker);

    /*visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.pose.position = viewpoints[i].pose.position;
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.z = 0.05;
    textMarker.color = color;
    //textMarker.lifetime = ros::Duration(2);
    textMarker.text = std::to_string(viewpoints[i].infoGain) + ", " + std::to_string(viewpoints[i].distance) + ", " + std::to_string(viewpoints[i].utility);
    markers.markers.push_back(textMarker);*/

    poseArr.poses.push_back(vps[i]->pose);
  }
  ROS_INFO_STREAM("Last Marker count namespace: " << ns << ": " << last_marker_count << "; current: " << vps.size());
  for (size_t i = vps.size(); i < last_marker_count; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);
    /*visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(textMarker);*/
  }
  last_marker_count = vps.size();
  viewArrowVisPub.publish(markers);
  poseArrayPub.publish(poseArr);
}

void ViewposeGraphManager::visualizeGraph(bool visualize_expanded, bool visualize_unexpanded)
{
  std::vector<ViewposePtr> vps;
  ROS_INFO_STREAM("Visualizing searched graph");
  vt_searched_graph->deleteAllMarkers();
  boost::shared_lock lock(graph_mtx);
  std::queue<Vertex> toVisualizeQueue;
  std::unordered_set<ViewposePtr> bestUtilityPoses;
  std::unordered_set<Vertex> visualizedSet;
  toVisualizeQueue.push(current_start_vertex);
  //ROS_INFO_STREAM("Computing highest utility path");
  for (ViewposePtr vp = config.goal_select_type == Vmp_BEST_UTILITY ? highest_util_pose : highest_ig_pose; vp != nullptr && ros::ok(); vp=vp->pred)
  {
    bestUtilityPoses.insert(vp);
  }
  size_t verts_bu = 0, verts_vis = 0, verts_exp = 0, verts_ue = 0;
  while(!toVisualizeQueue.empty() && ros::ok())
  {
    //ROS_INFO_STREAM("Visualize queue: " << toVisualizeQueue.size());
    Vertex v = toVisualizeQueue.front();
    toVisualizeQueue.pop();
    visualizedSet.insert(v);
    ViewposePtr vp = graph[v];
    vps.push_back(vp);
    bool on_bu_path = bestUtilityPoses.count(vp) > 0;
    for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
    {
      ViewposePtr target = graph[ei->m_target];
      if (visualizedSet.count(ei->m_target) > 0) // already visualized
        continue;

      bool visualize = false;
      rviz_visual_tools::colors line_color = rviz_visual_tools::DEFAULT;
      if (on_bu_path && bestUtilityPoses.count(target) > 0) // is on best utility path
      {
        line_color = rviz_visual_tools::CYAN;
        verts_bu++;
        visualize = true;
      }
      else  if (vp->visited && target->visited) // has already been visited
      {
        line_color = rviz_visual_tools::BLACK;
        verts_vis++;
        visualize = true;
      }
      else if (target->pred != nullptr) // has been expanded from this node
      {
        line_color = rviz_visual_tools::DARK_GREY;
        verts_exp++;
        visualize = visualize_expanded;
      }
      else // not yet expanded
      {
        line_color = rviz_visual_tools::GREY;
        verts_ue++;
        visualize = visualize_unexpanded;
      }

      if (visualize)
        vt_searched_graph->publishLine(graph[ei->m_source]->pose.position, graph[ei->m_target]->pose.position, line_color);

      toVisualizeQueue.push(ei->m_target);
    }
  }
  ROS_INFO_STREAM("Visualization done; " << verts_bu << " bu, " << verts_vis << " vis, " << verts_exp << " exp, " << verts_ue << " ue");
  vt_searched_graph->trigger();
  publishViewposeVisualizations(vps);
}


// void ViewposeGraphManager::visualizeOptimizedGraph(bool visualize_expanded, bool visualize_unexpanded)
// {
//   ROS_INFO_STREAM("Visualizing searched graph");

//   //int selected_views = graph_optimizer->optimal_paths.size();
//   //std::vector<ViewposePtr> vps(selected_views, nullptr);
//   std::vector<ViewposePtr> vps;

//   vt_searched_graph->deleteAllMarkers();

//   boost::shared_lock lock(graph_mtx);
//   //std::unordered_set<Vertex> visualizedSet;

//   // Iterate through all the vertices
//   for (const auto& pair : vertex_index)
//   {
//     // Get the vertex id for the edge
//     int v_id = pair.second;

//     // Check if the vertex is selected
//     if (graph_optimizer->view_selection[v_id].get(GRB_DoubleAttr_X) == 1.0)
//     {
//       // Get the vertex
//       Vertex v = pair.first;
//       //visualizedSet.insert(v);

//       // Get the viewpose for the vertex
//       ViewposePtr vp = graph[v];
//       vps.push_back(vp);

//       // Get all the edges from this vertex
//       for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
//       {
//         // Get the target vertex for the edge
//         Vertex target_v = boost::target(*ei, graph);
//         int target_id = vertex_index[target_v];

        // bool visualize = false;
        // rviz_visual_tools::colors line_color = rviz_visual_tools::DEFAULT;

        // // If the edge is selected, then cyan, else grey
        // if (graph_optimizer->path_selection[v_id][target_id].get(GRB_DoubleAttr_X) == 1.0)
        // {
        //   line_color = rviz_visual_tools::CYAN;
        //   visualize = visualize_expanded;
        // }
        // else
        // {
        //   line_color = rviz_visual_tools::GREY;
        //   visualize = visualize_unexpanded;
        // }
        // if (visualize)
        // {
        //   vt_searched_graph->publishLine(graph[ei->m_source]->pose.position, graph[ei->m_target]->pose.position, line_color);
        // }
//       }
//     }
//     else
//     {
//       continue;
//     }
//   }
//   vt_searched_graph->trigger();
//   publishViewposeVisualizations(vps);
// }


void ViewposeGraphManager::visualizeOptimizedGraph(bool visualize_expanded, bool visualize_unexpanded)
{
  ROS_INFO_STREAM("Visualizing searched graph");

  //int selected_views = graph_optimizer->optimal_paths.size();
  //std::vector<ViewposePtr> vps(selected_views, nullptr);
  std::vector<ViewposePtr> vps;

  vt_searched_graph->deleteAllMarkers();

  boost::shared_lock lock(graph_mtx);
  //std::unordered_set<Vertex> visualizedSet;

  std::unordered_map<int, Vertex> index_vertex;
  index_vertex.reserve(vertex_index.size());
  for (const auto& pair : vertex_index) {
    index_vertex[pair.second] = pair.first;
  }

  // Iterate through all the selected path
  for (int p = 0; p < graph_optimizer->optimal_paths.size() - 1; p++)
  {
    // Get the vertex id for the edge
    int start_vertex_index = graph_optimizer->optimal_paths[p];
    int target_vertex_index = graph_optimizer->optimal_paths[p+1];

    // Get the vertex
    Vertex start_vertex = index_vertex[start_vertex_index];
    Vertex target_vertex = index_vertex[target_vertex_index];

    // Get the viewpose for the vertex
    ViewposePtr vp = graph[target_vertex];
    vps.push_back(vp);

    bool visualize = false;
    rviz_visual_tools::colors line_color = rviz_visual_tools::DEFAULT;

    bool exists;
    Edge e;
    // check if an edge exists between the two vertices start_vertex and target_vertex
    boost::tie(e, exists) = boost::edge(start_vertex, target_vertex, graph);

    // If the edge is selected and is an existing edge, then cyan.
    if (exists)
    {
      line_color = rviz_visual_tools::CYAN;
      visualize = visualize_expanded;
    }
    // If the edge doesnt exist, then grey
    else
    {
      line_color = rviz_visual_tools::GREY;
      visualize = visualize_expanded;
    }
    if (visualize)
    {
      // fix the below line
      vt_searched_graph->publishLine(graph[start_vertex]->pose.position, graph[target_vertex]->pose.position, line_color);
    }
  }
  
  vt_searched_graph->trigger();
  publishViewposeVisualizations(vps);
}

void ViewposeGraphManager::initStartPose(const Vertex &v)
{
  current_start_vertex = v;
  current_start_vertex_number++;
  //graph_optimizer->start_view_id = vertex_id[v]; // Used by optimizer
  priorityQueue.push(v);
  expanded_vertices.insert(v);
}

void ViewposeGraphManager::markAsVisited(const Vertex &v)
{
  graph[v]->visited = true;
}

void ViewposeGraphManager::cleanupAfterMove()
{
  boost::unique_lock lock(graph_mtx);
  markAsVisited(current_start_vertex);
  priorityQueue.clear();
  for (const Vertex &v : expanded_vertices)
  {
    if (graph[v])
      graph[v]->clearPredecessor();
  }
  expanded_vertices.clear();
}

bool ViewposeGraphManager::expand()
{
  if (priorityQueue.empty())
    return false;

  boost::unique_lock lock(graph_mtx);

  Vertex v = priorityQueue.top();
  ViewposePtr vp = graph[v];
  priorityQueue.pop();

  moveit::core::RobotStatePtr cur_state = vp->state;
  std::vector<Edge> edges_to_remove;
  for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
  {

    TrajectoryPtr t = graph[*ei];
    if (t->last_collision_check_start_vertex != current_start_vertex_number) // last collision check performed before move
    {
      const robot_trajectory::RobotTrajectoryPtr &traj = cur_state == t->traj->getFirstWayPointPtr() ? t->traj : t->bw_traj;
      if (!robot_manager->isValidTrajectory(traj))
      {
        edges_to_remove.push_back(*ei);
        continue;
      }
      t->last_collision_check_start_vertex = current_start_vertex_number;
    }

    ViewposePtr target = graph[ei->m_target];
    if (ei->m_target == current_start_vertex || target->visited) // not a valid vertex to visit
    {
      continue;
    }
    if (target->pred) // already visited before
    {
      //TODO rewire
      //if (vp->accumulated_infogain > target->pred->accumulated_infogain) // set new predecessor
      continue;
    }
    target->addPredecessor(vp, t);
    if (!target->visited) // don't compute new cells for already visited targets
      octree_manager->computePoseObservedCells(octomap_vpp::poseToOctomath(target->pose), target->freeCells, target->occCells, target->unkCells);

    target->computeUtility();
    if (!highest_ig_pose || target->accumulated_infogain > highest_ig_pose->accumulated_infogain)
    {
      highest_ig_pose = target;
    }
    if (!highest_util_pose || target->accumulated_utility > highest_util_pose->accumulated_utility)
    {
      highest_util_pose = target;
    }
    //ValueHeap::handle_type handle =
    priorityQueue.push(ei->m_target);
    expanded_vertices.insert(ei->m_target);
  }
  for (const Edge &e : edges_to_remove)
  {
    boost::remove_edge(e, graph);
  }
  return true;
}

bool ViewposeGraphManager::optimize()
{
  // check if the initial camera pose vertex is set
  // This may be cause of further iterations being stopped
  if (priorityQueue.empty())
  {
    ROS_INFO_STREAM("Failing due to empty priority queue");
    return false;
  }

  boost::unique_lock lock(graph_mtx);

  // Create the connection matrix
  bool success = viewVoxelMatrix();

  // if (!success)
  // {
  //   ROS_INFO_STREAM("Failing due to 0 voxels selected");
  //   return false;
  // }

  // Set the full coverage constraints
  graph_optimizer->fullCoverageConstraints();

  // Set the model properties before optimizing
  graph_optimizer->setModelProperties();

  // Solve the optimization problem
  bool sol_exist = graph_optimizer->solvePath();

  ROS_INFO_STREAM("Optimization done");

  // If only one node is selected or if the optimization fails, then return false
  //if ((graph_optimizer->optimal_paths.size() < 3) || !sol_exist)
  if ((graph_optimizer->optimal_paths.size() < 2) || !sol_exist)
  {
    ROS_ERROR_STREAM("Optimal path is from start to virtual end");
    return false;
  }
  else if (!sol_exist)
  {
    ROS_ERROR_STREAM("Solution doesnt exist");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Optimal path is generated");
    return true;
  }
}

std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr> ViewposeGraphManager::getNextTrajectory()
{
  boost::shared_lock lock(graph_mtx);
  ViewposePtr vp = config.goal_select_type == Vmp_BEST_UTILITY ? highest_util_pose : highest_ig_pose;
  if (vp)
  {
    while(vp->pred && vp->pred->pred != nullptr)
    {
      vp = vp->pred;
    }
  }
  robot_trajectory::RobotTrajectoryPtr traj = getTrajectoryForState(vp->pred_edge, vp->pred);
  return {vertex_map[vp], traj};
}

std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> ViewposeGraphManager::getNextTrajectories(double cost_limit)
{
  boost::shared_lock lock(graph_mtx);
  ViewposePtr vp = config.goal_select_type == Vmp_BEST_UTILITY ? highest_util_pose : highest_ig_pose;
  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> next_trajectories;
  if (vp)
  {
    while(vp->pred)
    {
      if (vp->accumulated_cost < cost_limit || vp->pred->pred == nullptr) // always add at least one trajectory if available
      {
        next_trajectories.push_back({vertex_map[vp], getTrajectoryForState(vp->pred_edge, vp->pred), vp->pred_edge->cost});
      }
      vp = vp->pred;
    }
  }
  std::reverse(next_trajectories.begin(), next_trajectories.end());
  return next_trajectories;
}

std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> ViewposeGraphManager::getOptimizedTrajectories(double cost_limit)
{
  boost::shared_lock lock(graph_mtx);

  // number of trajectories
  int num_traj = (graph_optimizer->optimal_paths.size()) - 1;

  // variable to store the trajectories. If 20 pose, then 19 trajectories
  //std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> next_trajectories(num_traj);
  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> next_trajectories;

  double current_cost = 0.0;

  // index_vertex is a map with key as the index and value as the vertex
  // for faster vertex searching by index
  std::unordered_map<int, Vertex> index_vertex;
  index_vertex.reserve(vertex_index.size());
  for (const auto& pair : vertex_index) {
    index_vertex[pair.second] = pair.first;
  }


  // Iterate through each selected vertex in the optimal path
  //for (int i = 0; i < num_traj; i++)
  for (int i = 0; i < std::min(10, num_traj); i++) // sometimes, a very high number of trajectories would be generated. To prevent that, limit to 10 trajectories per execution.
  {
    // Get the vertex for the id
    Vertex v_origin = index_vertex[graph_optimizer->optimal_paths[i]];

    // Get the next vertex
    Vertex v_end = index_vertex[graph_optimizer->optimal_paths[i+1]];

    // Get the trajectory for the edge from v_origin to v_end
    // TrajectoryPtr t = graph[boost::edge(v_origin, v_end, graph).first];

    // TO ENSURE IF THE EDGE EXISTS OR NOT
    auto edge_pair = boost::edge(v_origin, v_end, graph);
    if (edge_pair.second) {
      TrajectoryPtr t = graph[edge_pair.first];
      current_cost += t->cost;
      next_trajectories.push_back({v_end, getTrajectoryForState(t, graph[v_origin]), t->cost});
      //next_trajectories[i] = {v_end, getTrajectoryForState(t, graph[v_origin]), t->cost};
    }
    else if (config.generate_traj_for_opt_path)
    {
      ROS_WARN_STREAM("Edge does not exist between indexes" << vertex_index[v_origin] << " and " << vertex_index[v_end]);
      ROS_WARN_STREAM("Edge does not exist between vertices" << v_origin << " and " << v_end);
      ROS_WARN_STREAM("Generating a trajectory between " << vertex_index[v_origin] << " and " << vertex_index[v_end]);

      // Generate a trajectory here
      moveit::core::RobotStatePtr from = graph[v_origin]->state;
      moveit::core::RobotStatePtr to = graph[v_end]->state;

      TrajectoryPtr t(new Trajectory());
      t->cost = from->distance(*to, robot_manager->getJointModelGroup());

      t->traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
      t->bw_traj.reset(new robot_trajectory::RobotTrajectory(robot_manager->getRobotModel(), robot_manager->getJointModelGroup()));
      t->traj->addSuffixWayPoint(from, 0);
      t->bw_traj->addPrefixWayPoint(from, 0);
      bool found_traj = true;
      if (t->cost > 0.1) // interpolate points
      {
        size_t num_steps = static_cast<size_t>(t->cost / 0.1);
        double frac_step = 1.0 / num_steps;
        for (size_t i = 1; i < num_steps; i++) // don't check first and last state (assume valid)
        {
          moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*from));
          from->interpolate(*to, i * frac_step, *temp_state, robot_manager->getJointModelGroup());
          temp_state->update();

          bool state_valid = robot_manager->isValidState(temp_state);
          if (!state_valid) // don't add edge
          {
            found_traj = false;
            ROS_WARN_STREAM("The state is invalid. So stopping the trajectory generation");
            t = nullptr;
            break;
          }
          t->traj->addSuffixWayPoint(temp_state, 0);
          t->bw_traj->addPrefixWayPoint(temp_state, 0);
        }
      }
      if (found_traj)
      {
        t->traj->addSuffixWayPoint(to, 0);
        t->bw_traj->addPrefixWayPoint(to, 0);

        trajectory_processing::TimeOptimalTrajectoryGeneration trajectory_processor;
        found_traj = (trajectory_processor.computeTimeStamps(*(t->traj), 1.0, 1.0) && trajectory_processor.computeTimeStamps(*(t->bw_traj), 1.0, 1.0));
        if (!found_traj)
        {
          ROS_WARN_STREAM("The trajectory generation failed");
          t = nullptr;
          break;
        }

        t->traj_start_state = from;
        t->bw_traj_start_state = to;

        /*auto [edge, success] = */boost::add_edge(v_origin, v_end, t, graph);
  
        t->last_collision_check_start_vertex = current_start_vertex_number;
        
        ROS_INFO_STREAM("Trajectory is generated between " << vertex_index[v_origin] << " and " << vertex_index[v_end]);

        // Add it to the list
        current_cost += t->cost;
        next_trajectories.push_back({v_end, getTrajectoryForState(t, graph[v_origin]), t->cost});
      }
    }
    else
    {
      ROS_WARN_STREAM("Edge does not exist between indexes" << vertex_index[v_origin] << " and " << vertex_index[v_end]);
      ROS_WARN_STREAM("Edge does not exist between vertices" << v_origin << " and " << v_end);
      TrajectoryPtr t = nullptr;
      break;
    }

    // // LATER CAN DECIDE A LONGER TRAJECTORY OR MULTIPLE SMALLER TRAJECTORIES
    // if (current_cost > cost_limit)
    // {
    //   break;
    // }

    // vertex, trajectory to reach the vertex, cost to reach the vertex
    // getTrajectoryForState(trajectory, trajectory start viewpose)
    //next_trajectories[i] = {v_end, getTrajectoryForState(t, graph[v_origin]), t->cost};
  }

  // cleanup memory
  index_vertex.clear();

  return next_trajectories;
}

bool ViewposeGraphManager::fixTrajectoryStartPoint(const robot_trajectory::RobotTrajectoryPtr &traj, const moveit::core::RobotStatePtr &state)
{
  moveit::core::RobotStatePtr cur_start = traj->getFirstWayPointPtr();
  traj->addPrefixWayPoint(state, 0);
  trajectory_processing::TimeOptimalTrajectoryGeneration trajectory_processor;
  bool success = trajectory_processor.computeTimeStamps(*traj, 1.0, 1.0);
  return success;
}

/*Generates the motion cost matrix*/
void ViewposeGraphManager::motionCostMatrix()
{
  // Resize the motion cost 2D matrix to accumulate the motion cost between the viewposes
  int matrix_size = graph_optimizer->num_of_view;
  ROS_INFO_STREAM("1. Number of views including virtual end view: " << graph_optimizer->num_of_view << " or " << vertex_index.size()+1 << " or " << matrix_size);

  graph_optimizer->views_motion.resize(matrix_size, std::vector<double>(matrix_size));

  if (config.motion_cost_mat_init = Vmp_Infinite_cost)
  {
    // Initialize the motion cost matrix to GRB_INFINITY.
    for (auto& row : graph_optimizer->views_motion) {
      row.assign(matrix_size, GRB_INFINITY);
    }
  }
  else if (config.motion_cost_mat_init = Vmp_Constant_100_cost)
  {
    // Initialize the motion cost matrix to 100.
    for (auto& row : graph_optimizer->views_motion) {
      row.assign(matrix_size, 100);
    }
  }
  else 
  {
    for (const auto& [vertex1, index1] : vertex_index)
    {
      for (const auto& [vertex2, index2] : vertex_index)
      {
        moveit::core::RobotStatePtr from = graph[vertex1]->state;
        moveit::core::RobotStatePtr to = graph[vertex2]->state;
        if (config.motion_cost_mat_init = Vmp_2times_actual_cost)
          graph_optimizer->views_motion[index1][index2] = 2 * (from->distance(*to, robot_manager->getJointModelGroup()));
        else
          graph_optimizer->views_motion[index1][index2] = 10 * (from->distance(*to, robot_manager->getJointModelGroup()));
      }
    }
  }

  //ROS_INFO_STREAM("Number of views including virtual end view: " << graph_optimizer->num_of_view << " or " << vertex_index.size()+1 << " or " << matrix_size);
  for (int i = 0; i < matrix_size; ++i) {
    // path cost to the virtual end node is 0
    // Set the last row to 0
    graph_optimizer->views_motion[matrix_size-1][i] = 0;
    // path cost to the virtual end node is 0
    // Set the last column to 0
    graph_optimizer->views_motion[i][matrix_size-1] = 0;
    // Path cost from a node to itself is 0
    graph_optimizer->views_motion[i][i] = 0;
  }
  ROS_INFO_STREAM("Number of views including virtual end view: " << graph_optimizer->num_of_view << " or " << vertex_index.size()+1 << " or " << matrix_size);

  graph_optimizer->start_view_id = vertex_index[current_start_vertex];
  ROS_INFO_STREAM("Current start vertex is: " << current_start_vertex << "...........................");
  ROS_INFO_STREAM("Current start index is: " << graph_optimizer->start_view_id << "...........................");

  // for (const auto& pair : vertex_index)
  // {
  //   ROS_INFO_STREAM("The vertex is " << pair.first << " and the index is " << pair.second);
  // }

  std::vector<Edge> edges_to_remove; // edges to be removed
  // Iterate through each vertex in the dictionary
  for (const auto& pair : vertex_index)
  {
    // Get the starting vertex id for the edge
    int start_id = pair.second;
    // Get the starting vertex for the edge
    Vertex start_v = pair.first;

    moveit::core::RobotStatePtr cur_state = graph[start_v]->state;

    // Iterate through all the edges from the starting vertex
    for (auto [ei, ei_end] = boost::out_edges(start_v, graph); ei != ei_end; ei++)
    {
      // Get the target vertex for the edge
      Vertex target_v = boost::target(*ei, graph);

      // Check if target vertex is present in the selected vertices(unvisited vertex), then continue
      if (vertex_index.find(target_v) == vertex_index.end())
        continue;
      
      // Get the target vertex id for the edge
      int target_id = vertex_index[target_v];

      TrajectoryPtr t = graph[*ei];

      // Collision check if the edge is created in another robot state.
      if (t->last_collision_check_start_vertex != current_start_vertex_number) // last collision check performed before move
      {
        const robot_trajectory::RobotTrajectoryPtr &traj = cur_state == t->traj->getFirstWayPointPtr() ? t->traj : t->bw_traj;
        if (!robot_manager->isValidTrajectory(traj))
        {
          edges_to_remove.push_back(*ei);
          continue;
        }
        t->last_collision_check_start_vertex = current_start_vertex_number;
      }

      // TrajectoryPtr t = graph[*ei];

      // Set the motion cost between the start and target viewposes
      graph_optimizer->views_motion[start_id][target_id] = t->cost;
      graph_optimizer->views_motion[target_id][start_id] = t->cost;
      // debugging to find the highest cost in the matrix
      // if(t->cost > highest_cost)
      // {
      //   highest_cost = t->cost;
      // }
    }
    for (const Edge &e : edges_to_remove)
    {
      boost::remove_edge(e, graph);
    }
    // clear the vector for next iteration
    edges_to_remove.clear();
  }
  //ROS_INFO_STREAM("Highest cost in the matrix is: " << highest_cost);


  ROS_INFO_STREAM("Number of views including virtual end view: " << graph_optimizer->num_of_view << " or " << vertex_index.size()+1 << " or " << matrix_size);
  ROS_INFO_STREAM("Start vertex is: " << current_start_vertex);
  ROS_INFO_STREAM("Start vertex index in matrix: " << vertex_index[current_start_vertex]);
  ROS_INFO_STREAM("Virtual end view id: " << graph_optimizer->virtual_end_view_id);

  // Print Motion Costs Between Viewpoints
  if (graph_optimizer->print_log)
  {
    std::cout << "Motion cost: " << std::endl;
    for (int i = 0; i < matrix_size; i++)
      std::cout << "\t" << i;
    std::cout << std::endl;
    for (int i = 0; i < matrix_size; i++) {
      std::cout << i << "\t";
      for (int j = 0; j < matrix_size; j++)
        std::cout << graph_optimizer->views_motion[i][j] << "\t";
      std::cout << std::endl;
    }
  }
}

void ViewposeGraphManager::motionCostConstraints()
{
  // Populate the vertex_index for the vertices having more than 2 edges
  int unique_idx = 0;
  // clear before populating
  vertex_index.clear();

  if(config.connected_subgraph)
  {
    // Select only the subgraph that is connected with the start vertex

    // option 1 - using connected_components
    // ros::Time start_time1 = ros::Time::now();
    
    // Vector to store the component number for each vertex
    std::vector<int> component(boost::num_vertices(graph));
    // Call connected_components to label each vertex with its component number
    int num_components = boost::connected_components(graph, &component[0]);
    // Get the component number for start vertex
    int start_vertex_component_num = component[current_start_vertex];
    // Get the part of the graph that is connected to the start vertex
    for (std::size_t v = 0; v < component.size(); ++v)
    {
      // If the vertex belongs to the subgraph and is unvisited
      if ((component[v] == start_vertex_component_num) && (unvisited_vertices.count(v) > 0))
      {
        // check for number of edges
        if (boost::out_degree(v, graph) > 2)
        {
          // assign unique matrix index to the vertex
          vertex_index[v] = unique_idx;
          unique_idx++;
          // ROS_INFO_STREAM("Vertex: " << v << " has been selected");
        }
      }
    }
    // ros::Duration time_taken1 = ros::Time::now() - start_time1;
    // ROS_WARN_STREAM("Option1 Time taken to find the connected subgraph: " << time_taken1);

    // ///*** option 2 - using breadth_first_search ***///
    // unique_idx = 0;
    // vertex_index.clear();
    // ros::Time start_time2 = ros::Time::now();
    
    // std::queue<std::size_t> q; // Queue for BFS
    // q.push(current_start_vertex);
    // while (!q.empty()) {
    //   // Dequeue a vertex from the queue
    //   std::size_t v = q.front();
    //   q.pop();
    //   vertex_index[v] = unique_idx;
    //   unique_idx++;
    //   // Get all neighboring vertices of the dequeued vertex v
    //   for (auto [ei, ei_end] = boost::out_edges(v, graph); ei != ei_end; ei++)
    //   {
    //     std::size_t neighbor = boost::target(*ei, graph);
    //     // If the neighbor vertex has not been visited and is part of the unvisited vertices
    //     if ((vertex_index.find(neighbor) == vertex_index.end()) && (unvisited_vertices.count(neighbor) > 0))
    //     {
    //       // check for number of edges
    //       if (boost::out_degree(neighbor, graph) > 2)
    //       {
    //         vertex_index[neighbor] = unique_idx;
    //         unique_idx++;
    //         // add to queue
    //         q.push(neighbor);
    //         ROS_INFO_STREAM("Vertex: " << neighbor << " has been selected");
    //       }
    //     }
    //   }
    // }

    // ros::Duration time_taken2 = ros::Time::now() - start_time2;
    // ROS_WARN_STREAM("Option2 Time taken to find the connected subgraph: " << time_taken2);
    // ///*** option 2 - using breadth_first_search ***///
  }
  else
  {
    // iterate through all the unvisited vertices
    for (const auto& v : unvisited_vertices)
    {
      size_t num_edges = boost::out_degree(v, graph);
      // the value can be changed to higher than 2 and perform comparisons
      // why 3? 1 self loop edge, 1 incoming and 1 outgoing edge
      if (num_edges > 2)
      {
        // assign unique matrix index to the vertex
        vertex_index[v] = unique_idx;
        unique_idx++;
      }
    }
  }

  // Get the number of unvisited view poses from the graph
  graph_optimizer->num_of_view = vertex_index.size() + 1; // including virtual end node
  //graph_optimizer->num_of_view = vertex_id.size() + 1;
  //graph_optimizer->num_of_view = boost::num_vertices(graph) + 1;

  // Set the virtual end view id
  ROS_INFO_STREAM("Unique id is : " << unique_idx);
  graph_optimizer->virtual_end_view_id = vertex_index.size();

  ROS_INFO_STREAM("Number of unvisited views in the graph: " << vertex_index.size());

  // Create motion cost matrix
  motionCostMatrix();
  ROS_INFO_STREAM("Unique id is : " << unique_idx);

  // Create variables
  graph_optimizer->createVariables();

  // Set objective
  graph_optimizer->setObjective();

  // Set view and path relation constraints
  graph_optimizer->viewPathConstraints();

  // set callback function
  graph_optimizer->setCallback();
}


bool ViewposeGraphManager::viewVoxelMatrix()
{
  // Create an id map for all view voxels
  graph_optimizer->num_of_voxel = 0;
  graph_optimizer->voxel_id_map.clear();

  // 1. Identify the inflated voxels or missing surface voxel that can be viewed from the sampled viewposes

  // create a map with voxel id as key and the viewed vertexes as value
  std::unordered_map<octomap::OcTreeKey, std::vector<Vertex>> voxel_viewedVertex;
  // create a map with voxel id as key and the count of viewed vertexes as value
  std::unordered_map<octomap::OcTreeKey, int> voxel_viewedVertexCount;

  // iterate through each voxel in current_filtered_targets and check which all viewposes has the voxel in its view. Also check for occlusions
  octomap::KeySet filtered_target_voxels, roi_target_voxels, target_voxels;
  if (config.target_voxel_type == Vmp_FILTERED_VOXEL) // only missing surface or inflated voxels
    target_voxels = octree_manager->getFilteredTargetsKeys();
  else if (config.target_voxel_type == Vmp_ROI_VOXEL) // only ROI conour voxels
    target_voxels = octree_manager->getRoiTargetsKeys();
  else // both missing surface or inflated voxels and ROI contour voxels
  {
    filtered_target_voxels = octree_manager->getFilteredTargetsKeys();
    roi_target_voxels = octree_manager->getRoiTargetsKeys();
    target_voxels.insert(filtered_target_voxels.begin(), filtered_target_voxels.end());
    target_voxels.insert(roi_target_voxels.begin(), roi_target_voxels.end());
  }
  //octomap::KeySet target_voxels = octree_manager->getFilteredTargetsKeys();
  //std::vector<octomap::point3d> target_voxel_pos = octree_manager->getTargetVoxels();


  for (const auto& key: target_voxels)
  {
    for (const auto& [vertex, unique_index] : vertex_index)
    {
      ViewposePtr vp = graph[vertex];
      
      // a. check if the voxel is within the field of view of the viewpose

      // camera position
      octomap::point3d cam_pos(vp->pose.position.x, vp->pose.position.y, vp->pose.position.z);
      // target voxel position
      octomap::point3d target_pos;
      if (config.target_voxel_type == Vmp_FILTERED_VOXEL) // only missing surface or inflated voxels
      {
        if (config.enable_superellipsoid_roi_sampling)
          target_pos = octree_manager->getPlanningTree()->keyToCoord(key);
        else
          target_pos = octree_manager->getInflatedTree()->keyToCoord(key);
        //octomap::point3d target_pos = target_voxel_pos[idx];
      }
      else if (config.target_voxel_type == Vmp_ROI_VOXEL) // only ROI conour voxels
        target_pos = octree_manager->getPlanningTree()->keyToCoord(key);
      else // both missing surface or inflated voxels and ROI contour voxels
      {
        if (roi_target_voxels.find(key) != roi_target_voxels.end())
          target_pos = octree_manager->getPlanningTree()->keyToCoord(key);
        else
        {
          if (config.enable_superellipsoid_roi_sampling)
            target_pos = octree_manager->getPlanningTree()->keyToCoord(key);
          else
            target_pos = octree_manager->getInflatedTree()->keyToCoord(key);
        }
      }
      // camera direction
      octomath::Vector3 camera_dir = (vp->origin - cam_pos).normalize();
      // target voxel vector from camera
      octomath::Vector3 target_vector = (target_pos - cam_pos).normalize();

      // compute the dot product
      double dot_product = camera_dir.dot(target_vector);

      // compute the angle between the vectors in degrees
      double angle_degrees = std::acos(dot_product) * 180.0 / M_PI;

      // check if the angle is within the FOV - 60 degrees is set as the FOV. (30 degrees on either side)
      bool is_fov = (angle_degrees <= (60 / 2.0));
      
      // // Visualization
      // visualization_msgs::MarkerArray markers;
      // std::string ns = "graphPoses";
      // visualization_msgs::Marker marker1;
      // marker1.header.frame_id = map_frame;
      // marker1.header.stamp = ros::Time();
      // marker1.ns = ns;
      // marker1.type = visualization_msgs::Marker::ARROW;
      // marker1.action = visualization_msgs::Marker::ADD;
      // marker1.pose.position.x = 0;
      // marker1.pose.position.y = 0;
      // marker1.pose.position.z = 0;
      // marker1.pose.orientation.x = 0.0;
      // marker1.pose.orientation.y = 0.0;
      // marker1.pose.orientation.z = 0.0;
      // marker1.pose.orientation.w = 1.0;
      // marker1.scale.x = 0.01;
      // marker1.scale.y = 0.02;
      // marker1.id = 1;
      // marker1.color = COLOR_RED;
      // marker1.points.push_back(vp->pose.position);
      // marker1.points.push_back(octomap::pointOctomapToMsg(vp->origin));
      // markers.markers.push_back(marker1);
      // visualization_msgs::Marker marker2;
      // marker2.header.frame_id = map_frame;
      // marker2.header.stamp = ros::Time();
      // marker2.ns = ns;
      // marker2.type = visualization_msgs::Marker::ARROW;
      // marker2.action = visualization_msgs::Marker::ADD;
      // marker2.pose.position.x = 0;
      // marker2.pose.position.y = 0;
      // marker2.pose.position.z = 0;
      // marker2.pose.orientation.x = 0.0;
      // marker2.pose.orientation.y = 0.0;
      // marker2.pose.orientation.z = 0.0;
      // marker2.pose.orientation.w = 1.0;
      // marker2.scale.x = 0.01;
      // marker2.scale.y = 0.02;
      // marker2.id = 2;
      // marker2.color = COLOR_GREEN;
      // marker2.points.push_back(vp->pose.position);
      // marker2.points.push_back(octomap::pointOctomapToMsg(target_pos));
      // markers.markers.push_back(marker2);
      // viewArrowVisPub.publish(markers);
      // ROS_ERROR_STREAM("The angle in degrees is " << angle_degrees << " and the result is " << is_fov << endl);
      // std::cin.get();

      // b. check if the voxel is occluded by any other voxel using ray casting
      if (is_fov)
      {
        octomap::KeyRay ray;
        bool ret = octree_manager->getPlanningTree()->computeRayKeys(target_pos, cam_pos, ray);
        if (!ret)
        {
          ROS_WARN_STREAM("Could not compute ray keys");
          continue;
        }
        // go through the ray and check if there is any occlusion
        auto rayIt = ray.begin();
        bool occlusion = false;
        for (auto rayEnd = ray.end(); rayIt != rayEnd; rayIt++)
        {
          octomap_vpp::RoiOcTreeNode *node = octree_manager->getPlanningTree()->search(*rayIt);
          // if occlusion, then continue
          if (node && octree_manager->getPlanningTree()->isNodeOccupied(node))
          {
            occlusion = true;
            break;
          }
        }
        
        // if a occlusion has occured, then skip to next viewpose
        if (occlusion)
        {
          continue;
        }
        else
        {
          voxel_viewedVertex[key].push_back(vertex); // add the vertex to the list of viewed vertexes for the voxel
          voxel_viewedVertexCount[key]++; // increment the count of viewed vertexes for the voxel
        }
      }
      else
      {
        continue;
      }
    }
  }

  // 2. Reduce the number of target voxels to be added to the constraints. In case if it is less, then add using explorative method
  int index = 0;
  if (voxel_viewedVertex.size() > config.number_of_target_voxels)
  {
    // select the top or low viewed config.number_of_target_voxels voxels as target voxels
    if (config.select_high_views_voxels || config.select_low_views_voxels)
    {
      ROS_WARN_STREAM("Max/Min viewed targets");
      // store the sorted map in a vector in descending or ascending order of the number of views
      std::vector<std::pair<octomap::OcTreeKey, int>> most_viewed_voxels(voxel_viewedVertexCount.begin(), voxel_viewedVertexCount.end());

      if (config.select_high_views_voxels)
        std::sort(most_viewed_voxels.begin(), most_viewed_voxels.end(), [](const auto& a, const auto& b) {return a.second > b.second;}); // sort the vector in descending order
      else
        std::sort(most_viewed_voxels.begin(), most_viewed_voxels.end(), [](const auto& a, const auto& b) {return a.second < b.second;}); // sort the vector in ascending order

      // get the top config.number_of_target_voxels voxels
      for (size_t i = 0; i < std::min(most_viewed_voxels.size(), static_cast<size_t>(config.number_of_target_voxels)); ++i)
      {
        // add the voxel to the list of target voxels
        graph_optimizer->voxel_id_map[most_viewed_voxels[i].first] = index++; // assign unique id to the voxel
      }
    }
    else
    {
      ROS_WARN_STREAM("Randomly selecting targets");

      // randomly select config.number_of_target_voxels voxels
      std::random_device rd;
      std::mt19937 gen(rd());
      // std::mt19937 gen(std::random_device rd());


      if (config.target_voxel_type != Vmp_ROI_AND_FILTERED_VOXEL)
      {
        // to store the random voxels
        std::vector<octomap::OcTreeKey> random_targets;
        random_targets.reserve(config.number_of_target_voxels);

        auto it = voxel_viewedVertexCount.begin();
        for (size_t i = 0; i < config.number_of_target_voxels  && it != voxel_viewedVertexCount.end(); ++i, ++it) {
          random_targets.push_back(it->first);
        }

        // apply reservoir sampling for the remaining elements
        for (size_t upper = config.number_of_target_voxels; it != voxel_viewedVertexCount.end(); ++it, ++upper) {
          std::uniform_int_distribution<size_t> dist(0, upper);
          size_t j = dist(gen);
          if (j < config.number_of_target_voxels) {
            random_targets[j] = it->first;
          }
        }

        // Convert the reservoir vector back to a KeySet
        for (const auto& key : random_targets)
          graph_optimizer->voxel_id_map[key] = index++; // assign unique id to the voxel
      }
      else
      {
        std::vector<octomap::OcTreeKey> filtered_candidates, roi_candidates;

        for (const auto& [key, count] : voxel_viewedVertexCount)
        {
          if (filtered_target_voxels.find(key) != filtered_target_voxels.end())
            filtered_candidates.push_back(key);
          else
            roi_candidates.push_back(key);
        }

        // select the count based on the proportion of roi and filtered voxels that got selected
        int filtered_target_count = std::min(int(config.number_of_target_voxels * config.filtered_target_percentage), int(filtered_candidates.size()));
        int roi_target_count = std::min(config.number_of_target_voxels - filtered_target_count, int(roi_candidates.size()));
        int total_count = filtered_target_count + roi_target_count; // sometimes the sum can be less than the number of target voxels

        // random shuffling
        std::random_shuffle(filtered_candidates.begin(), filtered_candidates.end());
        std::random_shuffle(roi_candidates.begin(), roi_candidates.end());

        // resize to keep the count
        if (filtered_candidates.size() > filtered_target_count)
          filtered_candidates.resize(filtered_target_count);
        if (roi_candidates.size() > roi_target_count)
          roi_candidates.resize(roi_target_count);

        // add the filtered voxels
        for (const auto& key : filtered_candidates)
          graph_optimizer->voxel_id_map[key] = index++; // assign unique id to the voxel
        for (const auto& key : roi_candidates)
          graph_optimizer->voxel_id_map[key] = index++; // assign unique id to the voxel
      }
    }
  }
  else
  {
    ROS_WARN_STREAM("Less than number_of_target_voxels targets");
    // use only the target voxels available
    for (const auto& [key, count] : voxel_viewedVertexCount)
    {
      graph_optimizer->voxel_id_map[key] = index++; // assign unique id to the voxel
    }
  }

  // 2 options - via unique voxel in viewVoxelMatrix or via exploration constraints in the model
  // if exploration is mode is enabled or if the number of target voxels is less than 5, then identify the top informative exploration poses
  if (config.add_exploration_targets || voxel_viewedVertex.size() < 5)
  {
    ROS_WARN_STREAM("Adding exploration targets with maximum information gain");
    ValueHeap vertexQueue(VertexUtilityComp(this)); // store and sort the vertex based on information gain
    // std::vector<std::VertexInfo> vertex_infogain; // store and sort the vertex based on information gain
    // iterate through each vertex in the vertex_index. only unvisited vertex would be present here.
    for (const auto& [vertex, unique_index]: vertex_index)
    {
      // get each viewposeptr from the vertex_index
      ViewposePtr vp = graph[vertex];

      // compute its information gain - amount of unknown voxels that can be viewed
      octree_manager->computePoseObservedCells(octomap_vpp::poseToOctomath(vp->pose), vp->freeCells, vp->occCells, vp->unkCells);
      vp->accumulated_infogain = vp->unkCells.size();

      // add each of them to a vector and sort it out based on the utility
      //vertex_infogain.emplace_back(vertex, vp->unkCells.size());
      vertexQueue.push(vertex);
    }

    // Option 1 - Use 2*top config.number_of_exploration_targets viewposes and set as constraint in the model - ALLOW RELAXED SELECTION
    if (config.set_expl_targets_as_constraints)
    {
      std::vector<int> exploration_view_ids;
      // get top 2*top config.number_of_exploration_targets viewposes
      for (size_t i = 0; i < std::min(vertexQueue.size(), static_cast<size_t>(2 * config.number_of_exploration_targets)); ++i)
      {
        // get the unique ids of these viewposes in the form std::vector<int>
        exploration_view_ids.push_back(vertex_index[vertexQueue.top()]); // push the id of the top vertex
        vertexQueue.pop(); // remove it from the queue
      }
      // set the constraints
      graph_optimizer->explorationConstraints(exploration_view_ids);
    }
    // Option 2 - Set top config.number_of_exploration_targets viewposes via random voxel - FORCED TO BE SELECTED
    else
    {
      // get top config.number_of_exploration_targets of them
      for (size_t i = 0; i < std::min(vertexQueue.size(), static_cast<size_t>(config.number_of_exploration_targets)); ++i)
      {
        octomap::OcTreeKey randomKey(rand() % 100, rand() % 100, rand() % 100); // random key to force the selection of this viewpose
        //voxel_viewedVertex[randomKey].push_back(vertex_infogain[i].vertex); // add the vertex with a voxel
        voxel_viewedVertex[randomKey].push_back(vertexQueue.top());
        graph_optimizer->voxel_id_map[randomKey] = index++; // assign unique id to the voxel
        vertexQueue.pop();
      }
    }
  }

  // 3. Create the view voxel matrix
  graph_optimizer->num_of_voxel = graph_optimizer->voxel_id_map.size();
  ROS_INFO_STREAM("Number of voxels: " << graph_optimizer->num_of_voxel);

  // return false if no target voxels are selected
  if (graph_optimizer->num_of_voxel < 1)
  {
    ROS_WARN_STREAM("No voxels selected");
    graph_optimizer->num_of_voxel = 1;
    graph_optimizer->connection_graph.resize(graph_optimizer->num_of_view, std::vector<bool>(graph_optimizer->num_of_voxel));

    for(int row=0; row<graph_optimizer->num_of_view; row++)
      graph_optimizer->connection_graph[row][0] = 0;

    ROS_WARN_STREAM("SIZE OF CONNECTION GRAPH: " << graph_optimizer->connection_graph.size());
    ROS_WARN_STREAM("CURRENT START VERTEX: " << current_start_vertex);
    ROS_WARN_STREAM("vertex_index[current_start_vertex]: " << vertex_index[current_start_vertex]);
    graph_optimizer->connection_graph[vertex_index[current_start_vertex]][0] = 1;
    ROS_WARN_STREAM("So creating 1 voxel to ensure the exploration motion is executed");
    return false;
  }
  // Create connection graph
  graph_optimizer->connection_graph.resize(graph_optimizer->num_of_view, std::vector<bool>(graph_optimizer->num_of_voxel));

  // Initialize with 0
  for (auto& row : graph_optimizer->connection_graph) {
      row.assign(graph_optimizer->num_of_voxel, 0);
  }

  // iterate through each key and check their viewposes and fill the matrix
  for (const auto& [key, voxel_id]: graph_optimizer->voxel_id_map)
  {
    for (const auto& vertex : voxel_viewedVertex[key])
    {
      // voxel_id_map is not used
      graph_optimizer->connection_graph[vertex_index[vertex]][voxel_id] = 1;
      //ROS_INFO_STREAM("Vertex: " << vertex << " is viewing voxel: " << voxel_id << std::endl);
    }
  }

  ROS_INFO_STREAM("Connection graph is created");

  // ROS_INFO_STREAM("Start vertex is : " << current_start_vertex);
  // for (const auto& [key, voxel_id]: graph_optimizer->voxel_id_map)
  // {
  //   for (const auto& vertex : voxel_viewedVertex[key])
  //   {
  //     ROS_INFO_STREAM("Vertexes viewing the target voxels " << voxel_id << " is " << vertex);
  //   }
  // }


  // Print the connection graph
  if (graph_optimizer->print_log)
    {
			std::cout << "connection_graph:" << std::endl;
			std::cout << '\t';
			for (int i = 0; i < graph_optimizer->num_of_voxel; i++) {
				std::cout << i << '\t';
			}
			std::cout << std::endl;
			for (int i = 0; i < graph_optimizer->num_of_view; i++) {
				std::cout << i << '\t';
				for (int j = 0; j < graph_optimizer->num_of_voxel; j++) {
					std::cout << graph_optimizer->connection_graph[i][j] << '\t';
				}
				std::cout << std::endl;
			}
		}
  return true;
}

void ViewposeGraphManager::publishViewVoxels(const std::shared_ptr<octomap_vpp::InflatedRoiOcTree> octree, const octomap::KeySet roi_voxels, const octomap::KeySet occ_voxels, const octomap::KeySet free_voxels)
{
  static const std_msgs::ColorRGBA COLOR_RED = []{std_msgs::ColorRGBA c; c.r = 1.f; c.g = 0.f; c.b = 0.f; c.a = 1.f; return c; } ();
  static const std_msgs::ColorRGBA COLOR_GREEN = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 1.f; c.b = 0.f; c.a = 1.f; return c; } ();
  static const std_msgs::ColorRGBA COLOR_BLUE = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 0.f; c.b = 1.f; c.a = 1.f; return c; } ();

  // Change to selected voxels
  const size_t NUM_P = roi_voxels.size() + occ_voxels.size() + free_voxels.size();
  if (NUM_P == 0)
  {
    ROS_WARN("No targets");
    return;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = map_frame;
  m.header.stamp = ros::Time();
  m.ns = "viewVoxels";
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.01;
  m.scale.y = 0.01;
  m.color.a = 1.0;
  m.points.reserve(NUM_P);
  m.colors.reserve(NUM_P);
  for (const auto& key : roi_voxels)
  {
    m.points.push_back(octomap::pointOctomapToMsg(octree->keyToCoord(key)));
    m.colors.push_back(COLOR_RED);
  }
  for (const auto& key : occ_voxels)
  {
    m.points.push_back(octomap::pointOctomapToMsg(octree->keyToCoord(key)));
    m.colors.push_back(COLOR_GREEN);
  }
  for (const auto& key : free_voxels)
  {
    m.points.push_back(octomap::pointOctomapToMsg(octree->keyToCoord(key)));
    m.colors.push_back(COLOR_BLUE);
  }
  viewVoxelPub.publish(m);
}

void ViewposeGraphManager::publishViewVoxels(const std::shared_ptr<octomap_vpp::RoiOcTree> octree, const octomap::KeySet roi_voxels, const octomap::KeySet occ_voxels, const octomap::KeySet free_voxels)
{
  static const std_msgs::ColorRGBA COLOR_RED = []{std_msgs::ColorRGBA c; c.r = 1.f; c.g = 0.f; c.b = 0.f; c.a = 1.f; return c; } ();
  static const std_msgs::ColorRGBA COLOR_GREEN = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 1.f; c.b = 0.f; c.a = 1.f; return c; } ();
  static const std_msgs::ColorRGBA COLOR_BLUE = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 0.f; c.b = 1.f; c.a = 1.f; return c; } ();

  // Change to selected voxels
  const size_t NUM_P = roi_voxels.size() + occ_voxels.size() + free_voxels.size();
  if (NUM_P == 0)
  {
    ROS_WARN("No targets");
    return;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = map_frame;
  m.header.stamp = ros::Time();
  m.ns = "viewVoxels";
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.01;
  m.scale.y = 0.01;
  m.color.a = 1.0;
  m.points.reserve(NUM_P);
  m.colors.reserve(NUM_P);
  for (const auto& key : roi_voxels)
  {
    m.points.push_back(octomap::pointOctomapToMsg(octree->keyToCoord(key)));
    m.colors.push_back(COLOR_RED);
  }
  for (const auto& key : occ_voxels)
  {
    m.points.push_back(octomap::pointOctomapToMsg(octree->keyToCoord(key)));
    m.colors.push_back(COLOR_GREEN);
  }
  for (const auto& key : free_voxels)
  {
    m.points.push_back(octomap::pointOctomapToMsg(octree->keyToCoord(key)));
    m.colors.push_back(COLOR_BLUE);
  }
  viewVoxelPub.publish(m);
}


// use the below only if a config condition is set to true
void ViewposeGraphManager::publishInflatedOctomap(const std::shared_ptr<octomap_vpp::InflatedRoiOcTree> ocTree)
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  // change to inflated tree
  bool msg_generated = octomap_msgs::fullMapToMsg(*ocTree, map_msg);
  if (msg_generated)
  {
    inflatedOctomapPub.publish(map_msg);
  }
}

} // namespace view_motion_planner
