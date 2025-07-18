#include "view_motion_planner/view_motion_planner.h"
#include <boost/graph/connected_components.hpp>
#include <boost/graph/incremental_components.hpp>

namespace view_motion_planner
{

ViewMotionPlanner::ViewMotionPlanner(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer,
                                     const std::string &map_frame, const std::string &ws_frame, const std::string &pose_frame,
                                     const std::string &robot_description_param_name, const std::string &group_name, const std::string &ee_link_name,
                                     double tree_resolution, size_t graph_builder_threads, bool update_planning_tree, bool initialize_evaluator)
  : nh_(nh),
    random_engine(std::random_device{}()),
    map_frame(map_frame),
    ws_frame(ws_frame),
    pose_frame(pose_frame),
    NUM_GRAPH_BUILDER_THREADS(graph_builder_threads),
    update_planning_tree(update_planning_tree),
    eval_initialized(initialize_evaluator),
    eval_running(false),
    eval_start(false),
    config_server(config_mutex),
    vt_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_graph")),
    vt_searched_graph(new rviz_visual_tools::RvizVisualTools(map_frame, "vm_searched_graph")),
    robot_manager(new RobotManager(nh, tfBuffer, pose_frame, robot_description_param_name, group_name, ee_link_name)),
    vt_robot_state(new moveit_visual_tools::MoveItVisualTools(map_frame, "vm_robot_state", robot_manager->getPlanningSceneMonitor())),
    graph_optimizer(new ViewsVoxelsMotion()),
    octree_manager(new OctreeManager(nh, tfBuffer, map_frame, ws_frame, pose_frame, tree_resolution, random_engine, robot_manager, 100, update_planning_tree, initialize_evaluator)),
    graph_manager(new ViewposeGraphManager(robot_manager, octree_manager, graph_optimizer, vt_searched_graph, map_frame, ws_frame, pose_frame)),
    trolley_remote(ros::NodeHandle(), ros::NodeHandle("/trollomatic"))
{
  // publishes the RED colored workspace cube
  workspacePub = nh.advertise<visualization_msgs::Marker>("workspace", 1, true);
  // publishes the GREEN colored sampling region cube
  samplingRegionPub = nh.advertise<visualization_msgs::Marker>("samplingRegion", 1, true);
  // updates the configurations based on the input from the rqt plugin
  config_server.setCallback(boost::bind(&ViewMotionPlanner::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2));
  vt_robot_state->loadMarkerPub(true);
  vt_robot_state->loadRobotStatePub("planned_state");
  vt_robot_state->setManualSceneUpdating();
}

ViewMotionPlanner::~ViewMotionPlanner()
{
  graph_builder_condition.shutdown();
  pose_visualizer_condition.shutdown();
  graph_visualizer_condition.shutdown();
  //graph_builder_condition.waitForShutdown();
  //pose_visualizer_condition.waitForShutdown();
  //graph_visualizer_condition.waitForShutdown();
  for (boost::thread &t : graph_builder_threads)
  {
    if (t.joinable())
      t.join();
  }
  if (pose_visualize_thread.joinable())
    pose_visualize_thread.join();
  if (graph_visualize_thread.joinable())
    graph_visualize_thread.join();
}

void ViewMotionPlanner::reconfigureCallback(VmpConfig &new_config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure callback called, level " << level);

  // Adjust sensor range if necessary
  if (level & (1 << 5)) // minimum sensor range
  {
    if (new_config.sensor_max_range < new_config.sensor_min_range)
      new_config.sensor_max_range = new_config.sensor_min_range;
  }
  else if (level & (1 << 6)) // maximum sensor range
  {
    if (new_config.sensor_min_range > new_config.sensor_max_range)
      new_config.sensor_min_range = new_config.sensor_max_range;
  }

  // Adjust target ratio if necessary
  double ratio_diff = 1 - (new_config.roi_target_ratio + new_config.expl_target_ratio + new_config.border_target_ratio);
  if (ratio_diff != 0.0)
  {
    if (level & (1 << 9)) // roi_target_ratio
    {
      double other_sum = new_config.expl_target_ratio + new_config.border_target_ratio;
      double expl_diff, border_diff;
      if (other_sum != 0.0)
      {
        expl_diff = ratio_diff * new_config.expl_target_ratio / other_sum;
        border_diff = ratio_diff * new_config.border_target_ratio / other_sum;
      }
      else
      {
        expl_diff = border_diff = 0.5 * ratio_diff;
      }
      new_config.expl_target_ratio += expl_diff;
      new_config.border_target_ratio += border_diff;
    }
    else if (level & (1 << 10)) // expl_target_ratio
    {
      new_config.border_target_ratio += ratio_diff;
      if (new_config.border_target_ratio < 0)
      {
        new_config.roi_target_ratio += new_config.border_target_ratio;
        new_config.border_target_ratio = 0;
      }
    }
    else if (level & (1 << 11)) // border_target_ratio
    {
      new_config.expl_target_ratio += ratio_diff;
      if (new_config.expl_target_ratio < 0)
      {
        new_config.roi_target_ratio += new_config.expl_target_ratio;
        new_config.expl_target_ratio = 0;
      }
    }
  }

  bool publish_workspace = false;
  bool publish_sampling_region = false;

  // Adjust workspace/sampling region if necessary
  if (level & (1 << 16)) // workspace minimum
  {
    if (new_config.ws_max_x < new_config.ws_min_x)
      new_config.ws_max_x = new_config.ws_min_x;
    if (new_config.ws_max_y < new_config.ws_min_y)
      new_config.ws_max_y = new_config.ws_min_y;
    if (new_config.ws_max_z < new_config.ws_min_z)
      new_config.ws_max_z = new_config.ws_min_z;

    publish_workspace = true;
  }
  else if (level & (1 << 17)) // workspace maximum
  {
    if (new_config.ws_min_x > new_config.ws_max_x)
      new_config.ws_min_x = new_config.ws_max_x;
    if (new_config.ws_min_y > new_config.ws_max_y)
      new_config.ws_min_y = new_config.ws_max_y;
    if (new_config.ws_min_z > new_config.ws_max_z)
      new_config.ws_min_z = new_config.ws_max_z;

    publish_workspace = true;
  }

  if (level & (1 << 18)) // sampling region minimum
  {
    if (new_config.sr_max_x < new_config.sr_min_x)
      new_config.sr_max_x = new_config.sr_min_x;
    if (new_config.sr_max_y < new_config.sr_min_y)
      new_config.sr_max_y = new_config.sr_min_y;
    if (new_config.sr_max_z < new_config.sr_min_z)
      new_config.sr_max_z = new_config.sr_min_z;

    publish_sampling_region = true;
  }
  else if (level & (1 << 19)) // sampling region maximum
  {
    if (new_config.sr_min_x > new_config.sr_max_x)
      new_config.sr_min_x = new_config.sr_max_x;
    if (new_config.sr_min_y > new_config.sr_max_y)
      new_config.sr_min_y = new_config.sr_max_y;
    if (new_config.sr_min_z > new_config.sr_max_z)
      new_config.sr_min_z = new_config.sr_max_z;

    publish_sampling_region = true;
  }

  if (level & (1 << 20))
  {
    if (new_config.fully_connected)
      new_config.motion_cost_mat_init=0;
  }

  if (level & (1 << 27))
  {
    if (new_config.select_voxels_with_free_neigh)
      new_config.select_voxels_on_front = false;
  }
  if (level & (1 << 28))
  {
    if (new_config.select_voxels_on_front)
      new_config.select_voxels_with_free_neigh = false;

    if (new_config.select_high_views_voxels)
      new_config.select_low_views_voxels = false;
  }
  if (level & (1 << 29))
  {
    if (new_config.select_low_views_voxels)
      new_config.select_high_views_voxels = false;
  }

  config = new_config;

  if (publish_workspace) publishWorkspaceMarker();
  if (publish_sampling_region) publishSamplingRegionMarker();
}

void ViewMotionPlanner::publishWorkspaceMarker()
{
  visualization_msgs::Marker cube_msg;
  cube_msg.header.frame_id = ws_frame;
  cube_msg.header.stamp = ros::Time::now();
  cube_msg.ns = "workspace";
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::Marker::LINE_LIST;
  cube_msg.color = COLOR_RED;
  cube_msg.scale.x = 0.01;
  cube_msg.pose.orientation.w = 1.0; // normalize quaternion

  octomap::point3d min(config.ws_min_x, config.ws_min_y, config.ws_min_z);
  octomap::point3d max(config.ws_max_x, config.ws_max_y, config.ws_max_z);
  //ROS_INFO_STREAM("WS: " << min << max);
  addCubeEdges(min, max, cube_msg.points);

  workspacePub.publish(cube_msg);
}

void ViewMotionPlanner::publishSamplingRegionMarker()
{
  visualization_msgs::Marker cube_msg;
  cube_msg.header.frame_id = ws_frame;
  cube_msg.header.stamp = ros::Time::now();
  cube_msg.ns = "samplingRegion";
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::Marker::LINE_LIST;
  cube_msg.color = COLOR_GREEN;
  cube_msg.scale.x = 0.01;
  cube_msg.pose.orientation.w = 1.0; // normalize quaternion

  octomap::point3d min(config.sr_min_x, config.sr_min_y, config.sr_min_z);
  octomap::point3d max(config.sr_max_x, config.sr_max_y, config.sr_max_z);
  //ROS_INFO_STREAM("SR: " << min << max);
  addCubeEdges(min, max, cube_msg.points);

  samplingRegionPub.publish(cube_msg);
}

void ViewMotionPlanner::poseVisualizeThread()
{
  pose_visualizer_condition.confirm_start();
  for (ros::Rate rate(0.2); ros::ok() && !pose_visualizer_condition.isShutdown(); rate.sleep())
  {
    pose_visualizer_condition.wait();

    if (pose_visualizer_condition.isShutdown())
      break;

    observationPoseMtx.lock_shared();
    if (observationPoses.size() > 0)
    {
      std::uniform_int_distribution<size_t> distribution(0, observationPoses.size() - 1);
      size_t i = distribution(random_engine);
      ViewposePtr vp = observationPoses[i];

      vt_robot_state->publishRobotState(vp->state);

      //octomap::pose6d viewpose = robot_manager->getRobotStatePose(vp.state);
      //geometry_msgs::Point vpm = octomap::pointOctomapToMsg(viewpose.trans());
      //octomap::point3d_collection endpoints = computeVpRaycastEndpoints(viewpose);

      octomap::pose6d viewpose = octomap_vpp::poseToOctomath(vp->pose);
      geometry_msgs::Point vpm = vp->pose.position;
      octomap::point3d_collection endpoints = computeVpRaycastEndpoints(viewpose);

      vt_robot_state->deleteAllMarkers();
      for (const octomap::point3d &ep : endpoints)
      {
        geometry_msgs::Point epm = octomap::pointOctomapToMsg(ep);
        vt_robot_state->publishLine(vpm, epm, rviz_visual_tools::RvizVisualTools::intToRvizColor(0));
      }
      vt_robot_state->trigger();
    }
    observationPoseMtx.unlock_shared();
  }
  pose_visualizer_condition.confirm_shutdown();
}

void ViewMotionPlanner::graphVisualizeThread()
{
  graph_visualizer_condition.confirm_start();
  for (ros::Rate rate(1); ros::ok() && !graph_visualizer_condition.isShutdown(); rate.sleep())
  {
    graph_visualizer_condition.wait();

    if (graph_visualizer_condition.isShutdown())
      break;

    boost::shared_lock lock(graph_manager->getGraphMutex());
    ROS_INFO_STREAM("Computing connected components");
    ros::Time startTime = ros::Time::now();
    using VPCMap = std::unordered_map<Vertex, size_t>;
    using VPCPropertyMap = boost::associative_property_map<VPCMap>;
    VPCMap component_map;
    VPCPropertyMap component_property_map(component_map);
    size_t num = boost::connected_components(graph_manager->getGraph(), component_property_map);
    ROS_INFO_STREAM("Computing connected components took " << (ros::Time::now() - startTime) << ", " << num << " components");
    startTime = ros::Time::now();
    vt_graph->deleteAllMarkers();
    for (auto [ei, ei_end] = boost::edges(graph_manager->getGraph()); ei != ei_end; ei++)
    {
      vt_graph->publishLine(graph_manager->getGraph()[ei->m_source]->pose.position, graph_manager->getGraph()[ei->m_target]->pose.position, rviz_visual_tools::RvizVisualTools::intToRvizColor(component_map[ei->m_source] % 15));
      // visual_tools_->publishTrajectoryLine(graph_manager->getGraph()[*ei].traj, robot_manager->getJointModelGroup(), visual_tools_->intToRvizColor(component_map[ei->m_source] % 15));
    }
    vt_graph->trigger();
    ROS_INFO_STREAM("Viewpose graph published, took " << (ros::Time::now() - startTime));
  }
  graph_visualizer_condition.confirm_shutdown();
}


/*
1. Sample a target voxel
2. Sample a viewpose
If the viewpose is valid,
3. Reject viewpose if it's view similarity is too high
4. Add the viewpose to the graph, vertex_map, neighbors_joint, neighbors_vpsim, vertex_id, id_vertex
5. Connect to neighbors based on predefined conditions
6. Remove the viewpose from all the datastructures if it has 0 neighbours
*/
void ViewMotionPlanner::graphBuilderThread()
{
  graph_builder_condition.confirm_start();
  //static const size_t MAX_GRAPH_VERTICES = 10000;
  std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
  while(ros::ok() && !graph_builder_condition.isShutdown())
  {
    graph_builder_condition.wait();

    if (graph_builder_condition.isShutdown())
      break;

    // Sample a target voxel
    double target_selector = target_select_dist(random_engine);
    TargetType type;
    if (target_selector <= config.roi_target_ratio)
      type = TARGET_ROI;
    else if (target_selector <= config.roi_target_ratio + config.expl_target_ratio)
      type = TARGET_OCC;
    else
      type = TARGET_BORDER;

    // Sample a viewpose
    ViewposePtr vp = octree_manager->sampleRandomViewPose(type);
    if (vp)
    {
      boost::unique_lock lock(graph_manager->getGraphMutex());
      // Reject viewpose if it's view similarity is too high
      if (graph_manager->getNearestVertexDistanceByVpSimilarity(vp) < config.vpd_threshold) // VP is too similar to existing edge
      {
        continue;
      }
      // Add vertex to graph, vertex_map, neighbors_joint, neighbors_vpsim, unvisited_vertices
      Vertex v = graph_manager->addViewpose(vp);
      // Connect to neighbors based on predefined conditions
      size_t connections = graph_manager->connectNeighbors(v, static_cast<size_t>(config.max_nb_connect_count), config.max_nb_connect_dist);
      // If 0 neighbour, then discard the vertex

      ROS_INFO_STREAM("Connections for vertex " << v << " is " << connections);
      if (config.discard_unconnected_vertices && connections == 0)
      {
        // Remove vertex from graph, vertex_map, neighbors_joint, neighbors_vpsim, unvisited_vertices
        graph_manager->removeVertex(v);
        ROS_INFO_STREAM("Discarded vertex with 0 connections : " << v);
      }
    }
  }
  graph_builder_condition.confirm_shutdown();
}

void ViewMotionPlanner::computeStateObservedVoxels(const moveit::core::RobotStatePtr &state, octomap::KeySet &freeCells, octomap::KeySet &occCells, octomap::KeySet &unkCells)
{
  octomap::pose6d viewpose = octree_manager->transform(robot_manager->getRobotStatePose(state), pose_frame, map_frame);
  octree_manager->computePoseObservedCells(viewpose, freeCells, occCells, unkCells);
}

void ViewMotionPlanner::updateInflatedTreeTargets()
{
  octomap::KeySet potential_roi_voxels; // used to store the filtered out potential ROI voxels

  // 1. Generate the inflated tree

  // Inflate the Octomap
  ROS_INFO_STREAM("planningTree resolution is " << octree_manager->getPlanningTree()->getResolution());
  auto inflatedTree = octree_manager->getPlanningTree()->computeInflatedRois(config.scale_inflated_voxel_resolution * octree_manager->getPlanningTree()->getResolution(), config.inflation_radius);
  ROS_INFO_STREAM("inflatedTree resolution is " << inflatedTree->getResolution());
  // Store the reference to the inflated tree in the octree manager to be used in later stages
  octree_manager->setInflatedTree(inflatedTree);

  // To visualize the inflated tree
  if (config.visualize_inflatedTree)
  {
    graph_manager->publishInflatedOctomap(inflatedTree);
  }


  // 2. Extract the inflated ROI voxels that are unknown in the planning tree within the current sampling space
  for (auto it=inflatedTree->begin_leafs(), end=inflatedTree->end_leafs(); it != end; it++)
  {
    // Check if it is within the sampling space
    if (!(octree_manager->isInSamplingRegion(octree_manager->transform(inflatedTree->keyToCoord(it.getKey()), map_frame, ws_frame))))
      continue; // ROI not in sampling region

    auto node = octree_manager->getPlanningTree()->search(it.getCoordinate());
    // If the voxel is unknown in the planning tree
    if (node == nullptr || node->getLogOdds() == 0.0)
    {
      potential_roi_voxels.insert(it.getKey()); // potential ROI voxel which can be further filtered
    }
    // else
    // {
    //   ROS_INFO_STREAM("Voxel is not unknown in the planning tree");
    // }
  }

  // 3. Reducing the number of voxels to be used as samples by filtering
  octomap::KeySet filtered_voxels; // to store the filtered voxels
  
  // The below filtering process is performed only if the number of potential ROI voxels is greater than 10
  // Why we set 10 instead of config.number_of_target_voxels is that, if we set config.number_of_target_voxels to for example 4,
  // then there is no certanity that all the 4 filtered voxels will generate a viewpose. So, it is better to have a lower limit of 10

  // 3.1 Filter the keys having a free voxel in the neighbourhood
  if (config.select_voxels_with_free_neigh && potential_roi_voxels.size() > 10)
  {
    for (const auto& key : potential_roi_voxels)
    {
      // check if there is a free voxel in neighbourhood of the potential_roi_voxels in the planning tree
      if (octree_manager->getPlanningTree()->hasNeighborInState(octree_manager->getPlanningTree()->coordToKey(inflatedTree->keyToCoord(key)), octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::FREE_NONROI, octomap_vpp::NB_18))
        filtered_voxels.insert(key);
    }
  }
  // 3.2 Filter the keys that are in the visible surface contour of the sampling space 
  else if (config.select_voxels_on_front && potential_roi_voxels.size() > 10)
  {
    octomap::KeySet unseen_keys = potential_roi_voxels; // iterate and keep the unseen keys within this variable
    octomap::KeySet non_occlusion_voxels; // store the non-planningTree-occlusion voxel keys

    // Sample some points in the workspace, cast rays in planningTree and check for occlusion
    // if the key is not occluded in atleast 1 view, then add it to the non-occlusion voxels
    for (int ray = 0; ray < config.number_of_visibility_check_points; ray++)
    {
      // Sample a point in the workspace
      octomap::point3d cam_point = octree_manager->transform(octree_manager->sampleRandomWorkspacePoint(), ws_frame, map_frame);

      // Check for occlusion with the planning tree
      octomap::KeyRay ray1;

      for (auto it = unseen_keys.begin(); it != unseen_keys.end(); )
      {
        // ROS_WARN_STREAM("key is " << inflatedTree->keyToCoord(*it) << " cam_point is " << cam_point);
        bool ret1 = octree_manager->getPlanningTree()->computeRayKeys(inflatedTree->keyToCoord(*it), cam_point, ray1);
        if (!ret1)
        {
          ROS_WARN_STREAM("Could not compute ray keys");
          ++it;
          continue;
        }
        // Iterate through the ray and check if there is any occlusion
        auto rayIt = ray1.begin();
        bool occlusion = false;
        for (auto rayEnd = ray1.end(); rayIt != rayEnd; rayIt++)
        {
          // search for occlusion in the planning tree
          octomap_vpp::RoiOcTreeNode *node = octree_manager->getPlanningTree()->search(*rayIt);
          // if occlusion, then continue
          if (node && octree_manager->getPlanningTree()->isNodeOccupied(node))
          {
            occlusion = true;
            break;
          }
        }
        if (occlusion) // if occluded view, continue to next voxel key
        {
          ++it;
          continue;
        }
        else // if not occluded in atleast one workspace point, then add it to the non-occlusion voxels
        {
          non_occlusion_voxels.insert(*it); // add the key to the non-occlusion voxels
          // unseen_keys.erase(*it);
          it = unseen_keys.erase(it); // key is already seen, so remove it from the unseen_keys to be not used in next iteration
        }
      }
    }

    octomap::KeySet lookup_set = non_occlusion_voxels; // lookup_set is a reference used to check if the key is in the non-occlusion voxels
    // Proceed to next step of checking the occlusion in the inflated tree
    if (non_occlusion_voxels.size() > 10)
    {
      // If atleast 1 non-occluded view, the voxel key is selected
      for (int ray = 0; ray < config.number_of_visibility_check_points; ray++)
      {
        // Sample a point in the workspace
        octomap::point3d cam_point = octree_manager->sampleRandomWorkspacePoint();

        // Check for occlusion with the inflated tree
        octomap::KeyRay ray2;

        for (auto it = non_occlusion_voxels.begin(); it != non_occlusion_voxels.end(); )
        {
          bool ret2 = inflatedTree->computeRayKeys(inflatedTree->keyToCoord(*it), cam_point, ray2);
          if (!ret2)
          {
            ROS_WARN_STREAM("Could not compute ray keys");
            ++it;
            continue;
          }
          // Iterate through the ray and check if there is any occlusion
          auto rayIt2 = ray2.begin();
          bool occlusion = false;
          for (auto rayEnd = ray2.end(); rayIt2 != rayEnd; rayIt2++)
          {
            if (*rayIt2 == *it) // if the key is the same as the voxel key, then continue
            {
              continue;
            }
            octomap_vpp::InflatedRoiOcTreeNode *node = inflatedTree->search(*rayIt2);

            if (node)
            {
              octomap::OcTreeKey& ray_key = *rayIt2;

              // Check if this key is in the non-collision roi voxels
              if (lookup_set.find(ray_key) != lookup_set.end())
              {
                occlusion = true;
                break;
              }
            }
          }
          if (!occlusion) // No occlusion for this voxel key in inflatedTree
          {
            filtered_voxels.insert(*it); // add it to the final filtered voxels
            //non_occlusion_voxels.erase(*it);
            it = non_occlusion_voxels.erase(it); // remove the key from the non-occlusion voxels to be not used in the next iteration
          }
          else
          {
            ++it; // If occlusion, then continue to the next voxel key
          }
        }
      }
    }
    else
    {
      filtered_voxels.clear();
      //filtered_voxels = non_occlusion_voxels;
      filtered_voxels = std::move(non_occlusion_voxels);
    }
  }
  // 4. Visualize the results
  if (config.visualize_viewVoxels)
  {
    // // Publish all the potential ROI voxels in the unknown region
    // octomap::KeySet empty_set1;
    // octomap::KeySet empty_set2;
    // graph_manager->publishViewVoxels(inflatedTree, potential_roi_voxels, empty_set1, empty_set2);
    // std::cout << "Total number of potential ROI voxels in unknown region are " << potential_roi_voxels.size() << std::endl;
    // // std::cin.get();

    // Publish selected and non-selected voxels from the potential_roi_voxels
    octomap::KeySet not_selected;
    for (const auto& key : potential_roi_voxels) {
      if (filtered_voxels.find(key) == filtered_voxels.end()) {
          not_selected.insert(key);
      }
    }
    octomap::KeySet empty_set;
    graph_manager->publishViewVoxels(inflatedTree, filtered_voxels, empty_set, not_selected);
    std::cout << "Total number of filtered voxels in unknown region are " << filtered_voxels.size() << std::endl;
    std::cout << "Total number of removed voxels in unknown region are " << not_selected.size() << std::endl;
    // std::cin.get();
  }

  // 5. If filtering process is done, then set that as the target voxels
  if (filtered_voxels.size() > 10) // make sure atleast 10 voxels are selected for sampling. If not enough, then use all the potential_roi_voxels
  {
    potential_roi_voxels.clear();
    //potential_roi_voxels = filtered_voxels;
    potential_roi_voxels = std::move(filtered_voxels);
  }

  // 6. Add to targets that will be used for sampling and also in the optimization
  octree_manager->updateFilteredTargetsKeys(potential_roi_voxels);
  octree_manager->updateFilteredTargets(potential_roi_voxels);
}


void ViewMotionPlanner::updateMissingSurface()
{
  if (config.enable_superellipsoid_roi_sampling)
  {
    superellipsoid_msgs::MissingEllipsoidSurfaceParamsConstPtr msg = ros::topic::waitForMessage<superellipsoid_msgs::MissingEllipsoidSurfaceParams>("/capsicum_superellipsoid_detector/vmp_missing_surfaces", nh_, ros::Duration(2.0));
    if (!msg)
    {
      ROS_ERROR_STREAM("Missing surface message not received");
      return;
    }
    callbackMissingSurface(*msg);
  }
}

void ViewMotionPlanner::callbackMissingSurface(const superellipsoid_msgs::MissingEllipsoidSurfaceParams& msg)
{
  auto t1 = ros::Time::now();
  // Insert the points into current_filtered_targets to be used for sampling and also to be set as target voxels in toptimization
  octree_manager->setMissingShapes(msg);
  ROS_INFO_STREAM("Received and pre processed the missing surface message in " << (ros::Time::now() - t1).toSec() << " seconds");

  auto t2 = ros::Time::now();
  // Filter these points further to be used in the sampling process
  auto [selected, notselected] = octree_manager->filterMissingShapes();
  if (config.visualize_viewVoxels)
  {
    octomap::KeySet empty_set;
    graph_manager->publishViewVoxels(octree_manager->getPlanningTree(), selected, empty_set, notselected);
    ROS_INFO_STREAM("Number of selected missing surface voxels: " << selected.size());
    ROS_INFO_STREAM("Number of not selected missing surface voxels: " << notselected.size());
  }
  ROS_INFO_STREAM("Filtered the missing surface points in " << (ros::Time::now() - t2).toSec() << " seconds");
}

std::optional<Vertex> ViewMotionPlanner::initCameraPoseGraph()
{
  boost::unique_lock lock(graph_manager->getGraphMutex());
  ViewposePtr cam_vp(new Viewpose());
  cam_vp->state = robot_manager->getCurrentState();
  if (!cam_vp->state) return std::nullopt;
  cam_vp->pose = octree_manager->transform(robot_manager->getCurrentPose(), pose_frame, map_frame);
  Vertex cam_vert = graph_manager->addViewpose(cam_vp);
  return cam_vert;
}

/*
1. Resume graphBuilderThread for building the graph until graph_building_time
2. Current camere pose is initialized and added to the graph - initCameraPoseGraph()
3. Store the current vertex as the start vertex and increment the state of the graph. Priority queue is added with current start vertex - initStartPose()
4. After sleeping for graph_building_time, connect the neighbors of the current start vertex and check the number of out edges. If it is greater than 0, return true
*/
bool ViewMotionPlanner::buildGraph()
{
  resumeGraphBuilderThreads();
  std::optional<Vertex> cam_vert = initCameraPoseGraph();
  if (!cam_vert)
  {
    ROS_ERROR("Camera pose not initialized");
    return false;
  }
  // When initStartPose is called, the state of the graph is incremented using current_start_vertex_number++ and the start vertex is stored in current_start_vertex
  graph_manager->initStartPose(*cam_vert);

  ros::Duration(config.graph_building_time).sleep();
  ROS_ERROR("Completed building the graph");
  for (ros::Rate rate(1); ros::ok() && config.mode >= Vmp_BUILD_GRAPH; rate.sleep()) // connect start pose if not connected
  {
    boost::unique_lock lock(graph_manager->getGraphMutex());
    graph_manager->connectNeighbors(*cam_vert, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
    size_t num_out_edges = boost::out_degree(*cam_vert, graph_manager->getGraph());
    ROS_INFO_STREAM("Number of out edges from cam: " << num_out_edges);
    if (num_out_edges > 0){
      return true;
    }
  }
  return false;
}

bool ViewMotionPlanner::searchPath()
{
  if (config.pause_graph_building_on_search)
    pauseGraphBuilderThreads();

  ros::Time start_expand(ros::Time::now());
  bool has_expanded = false;
  while (ros::ok())
  {
    bool success = graph_manager->expand();

    if (!success || (ros::Time::now() - start_expand).toSec() > config.graph_search_time)
      break;

    has_expanded = true;
  }
  if (config.visualize_graph)
    graph_manager->visualizeGraph(config.visualize_expanded, config.visualize_unexpanded);

  return has_expanded;
}

/*
1. Motion cost matrix
2. View voxel matrix
3. Define the optimization problem
4. Optimize
*/
bool ViewMotionPlanner::optimizePath()
{
  if (config.pause_graph_building_on_search)
  {
    ROS_INFO_STREAM("Pausing graph builder threads during search");
    pauseGraphBuilderThreads();
  }

  // Visualize the sampled view poses
  // if (config.visualize_graph)
  // {
  //   std::vector<ViewposePtr> vps;

  //   // iterate through the graph and get the viewposes
  //   for (auto [vi, vi_end] = boost::vertices(graph_manager->getGraph()); vi != vi_end; vi++)
  //   {
  //     vps.push_back(graph_manager->getGraph()[*vi]);
  //   }

  //   graph_manager->publishViewposeVisualizations(vps);
  //   ROS_INFO_STREAM("Press enter to continue");
  //   std::cin.get();
  // }

  // Visualize the graph
  std::ofstream fout("graph.dot");
  // sudo apt install graphviz
  // dot -Tpng graph.dot -o graph.png
  write_graphviz(fout, graph_manager->getGraph());

  // Motion cost matrix and constraints generation
  ros::Time StartTime1(ros::Time::now());
  graph_manager->motionCostConstraints();
  ROS_INFO_STREAM("Motion cost constraints generation time: " << (ros::Time::now() - StartTime1) << " s");

  // View voxel matrix and constraints generation and optimization
  ros::Time StartTime2(ros::Time::now());
  bool success = graph_manager->optimize();
  ROS_INFO_STREAM("View voxels + Optimization time: " << (ros::Time::now() - StartTime2) << " s");
  //if (!success || (ros::Time::now() - StartTime2).toSec() > config.graph_search_time)
  if (eval_running)
    octree_manager->savePlannerData(eval_current_segment);
    
  if (!success)
  {
    //ROS_INFO_STREAM("Time limit exceeded for optimization");
    ROS_INFO_STREAM("Optimization process failed");
    return false;
  }

  if (config.visualize_graph)
  { 
    graph_manager->visualizeOptimizedGraph(config.visualize_expanded, config.visualize_unexpanded);
  }

  return true;
}

bool ViewMotionPlanner::executePath()
{
  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> trajectories = graph_manager->getNextTrajectories(config.execution_cost_limit);
  std::optional<Vertex> next_start_vertex = graph_manager->getCurrentStartVertex();
  bool moved = false;
  ROS_INFO_STREAM("Executing " << trajectories.size() << " trajectories");
  if (trajectories.size() == 0)
  {
    ROS_WARN_STREAM("No trajectories; resetting graph");
    graph_manager->clear();
    next_start_vertex = initCameraPoseGraph();
    if (!next_start_vertex)
    {
      ROS_ERROR("Camera pose not initialized");
    }
    return false;
  }
  for (const auto &[next_vertex, traj, cost] : trajectories)
  {
    ViewposePtr vp = graph_manager->getGraph()[next_vertex];
    if (!traj)
    {
      ROS_WARN_STREAM("No trajectory found");
      graph_manager->clear();
      next_start_vertex = initCameraPoseGraph();
      if (!next_start_vertex)
      {
        ROS_ERROR("Camera pose not initialized");
        return false;
      }
      graph_manager->connectNeighbors(*next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
      break;
    }
    else
    {
      if (moved)
      {
        if (!robot_manager->isValidTrajectory(traj))
        {
          ROS_WARN_STREAM("Planned trajectory no longer valid");
          break;
        }
      }
      if (config.fix_trajectories_on_execution)
      {
        graph_manager->fixTrajectoryStartPoint(traj, robot_manager->getCurrentState());
      }
      bool success = robot_manager->executeTrajectory(traj);
      moved = true;
      graph_manager->markAsVisited(*next_start_vertex);
      octree_manager->waitForPointcloudWithRoi();
      if (eval_running)
      {
        octree_manager->saveEvaluatorData(cost, traj->getDuration(), eval_current_segment, trolley_remote.getPosition(), trolley_remote.getHeight());
      }
      if (!success)
      {
        ROS_WARN_STREAM("Failed to execute trajectory, setting new start pose");
        graph_manager->clear();
        next_start_vertex = initCameraPoseGraph();
        if (!next_start_vertex)
        {
          ROS_ERROR("Camera pose not initialized");
          return false;
        }
        graph_manager->connectNeighbors(*next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
        break;
      }
      else
      {
        octree_manager->updatePastViewposesList(vp);
        next_start_vertex = next_vertex;
      }
    }
  }
  if (moved)
  {
    graph_manager->cleanupAfterMove();
    graph_manager->initStartPose(*next_start_vertex);
  }
  return moved;
}


bool ViewMotionPlanner::executeOptimizedPath()
{
  // Get the vector of vertex, trajectory and cost in the order in which the viewposes are to be visited
  ros::Time StartTime1(ros::Time::now());
  std::vector<std::tuple<Vertex, robot_trajectory::RobotTrajectoryPtr, double>> trajectories = graph_manager->getOptimizedTrajectories(config.execution_cost_limit);
  ROS_INFO_STREAM("Optimized trajectory is obtained in : " << (ros::Time::now() - StartTime1) << " s");

  // Get current start vertex. Only done once in the beginning to get the start vertex for the graph
  std::optional<Vertex> next_start_vertex = graph_manager->getCurrentStartVertex();

  if (next_start_vertex.has_value()) { // Check if optional contains a value
    graph_manager->remove_visited_vertices(next_start_vertex.value()); // Extract the value from the optional
  }

  bool moved = false;
  ROS_INFO_STREAM("Executing " << trajectories.size() << " trajectories");

  // if no trajectories are found, reset the graph completely
  // current camera pose is added to the graph as a vertex
  if (trajectories.size() == 0) // very rarely can happen
  {
    ROS_WARN_STREAM("No trajectories; resetting graph");
    // graph_manager->clear();
    // next_start_vertex = initCameraPoseGraph();
    // if (!next_start_vertex)
    // {
    //   ROS_ERROR("Camera pose not initialized");
    // }
    return false;

    // the graph will be built once this is returned
  }

  //graph_manager->fixTrajectoryStartPoint(std::get<1>(trajectories[0]), robot_manager->getCurrentState());

  boost::thread octomap_update_thread;

  // start time of execution
  ros::Time start_time = ros::Time::now();
  // Iterate through the vector of trajectories
  for (const auto &[next_vertex, traj, cost] : trajectories)
  {
    // Get the current vertex of the robot
    ViewposePtr vp = graph_manager->getGraph()[next_vertex];

    // If no trajectory is found from the current vertex, reset the graph
    if (!traj)
    {
      ROS_WARN_STREAM("No trajectory found");
      // graph_manager->clear();
      // // current camera pose is added to the graph as a vertex
      // next_start_vertex = initCameraPoseGraph();
      // if (!next_start_vertex)
      // {
      //   ROS_ERROR("Camera pose not initialized");
      //   if (octomap_update_thread.joinable())
      //   {
      //     octomap_update_thread.join();
      //   }
      //   return false;
      // }
      // graph_manager->connectNeighbors(*next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
      break;
    }
    else
    {
      // if previously moved
      if (moved)
      {
        // and the current trajectory is not valid, break the loop
        if (!robot_manager->isValidTrajectory(traj))
        {
          ROS_WARN_STREAM("Planned trajectory no longer valid");
          break;
        }
      }

      // If the current pose is not exactly same as the expected pose, fix the startpoint for next trajectory execution
      if (config.fix_trajectories_on_execution)
      {
        ROS_WARN_STREAM("Fixing the trajectory start point");
        graph_manager->fixTrajectoryStartPoint(traj, robot_manager->getCurrentState());
      }

      // Execute the trajectory
      bool success = robot_manager->executeTrajectory(traj);
      // if (!success)
      // {
      //   ROS_WARN_STREAM("Retrying to fix the problem");
      //   graph_manager->fixTrajectoryStartPoint(std::get<1>(trajectories[0]), robot_manager->getCurrentState());
      //   success = robot_manager->executeTrajectory(traj);
      // }
      // moved = true; // set to true if the robot has moved atleast once
      // Mark the current vertex as visited using graph[v]->visited = true
      graph_manager->markAsVisited(*next_start_vertex);
      // Pointcloud is updated with the new viewpose
      // DEFAULT OPTION
      // octree_manager->waitForPointcloudWithRoi();

      // OPTION 1
      octree_manager->threadedWaitForPointcloudWithRoi(octomap_update_thread);
      //option2
      //octree_manager->waitForPointcloudWithRoi();

      // If the evaluator is running, save the data
      if (eval_running)
      {
        // Check if the evaluation is affected by thread
        // if (octomap_update_thread.joinable())
        // {
        //   octomap_update_thread.join();
        // }
        octree_manager->saveEvaluatorData(cost, traj->getDuration(), eval_current_segment, trolley_remote.getPosition(), trolley_remote.getHeight());
      }

      // If the trajectory is not executed successfully, reset the graph as before
      if (!success)
      {
        ROS_WARN_STREAM("Failed to execute trajectory, setting new start pose");
        // graph_manager->clear();
        // next_start_vertex = initCameraPoseGraph();
        // if (!next_start_vertex)
        // {
        //   ROS_ERROR("Camera pose not initialized");
        //   if (octomap_update_thread.joinable())
        //   {
        //     octomap_update_thread.join();
        //   }
        //   return false;
        // }
        // graph_manager->connectNeighbors(*next_start_vertex, static_cast<size_t>(config.max_start_state_connect_count), config.max_start_state_connect_dist);
        if (octomap_update_thread.joinable())
        {
          octomap_update_thread.join();
        }
        if (moved)
          break; // if moved atleast once, it will execute final 2 code blocks and return true.
        else
          return false; // if not moved, then return false
      }
      // If the trajectory is successful, the viewpose is updated in the past viewposes list
      else
      {
        moved = true; // set to true if the robot has moved atleast once
        // Remove the visited viewpose from the id_vertex and vertex_id map
        graph_manager->remove_visited_vertices(next_vertex);
        octree_manager->updatePastViewposesList(vp); // past_viewposes_ is updated. But not used anywhere
        // here we can also add this pose to the list of visited poses so that it is not revisited
        next_start_vertex = next_vertex;
      }
    }
    // Before going to next iteration, check if the time exceeded the limit.
    if ((ros::Time::now() - start_time).toSec() > config.execution_time_limit)
    {
      ROS_WARN_STREAM("Execution time limit exceeded");
      break;
    }
  }
  // If the robot has moved atleast once,
  if (moved)
  {
    // Now the entire trajectory is executed, so cleanup priorityQueue and expanded_vertices which can accomodate new values in next iteration
    graph_manager->cleanupAfterMove();
    // Now the whole graph is updated with the new start vertex. So collision check will be performed for previously created edges.
    graph_manager->initStartPose(*next_start_vertex);
    // Make sure the lastly executed pose is the next starting one
    graph_manager->add_current_vertex(*next_start_vertex);
    ROS_ERROR_STREAM("MOVEMENT IS COMPLETE");

    ROS_ERROR_STREAM("Current start vertex is " << graph_manager->getCurrentStartVertex());
    ROS_ERROR_STREAM("The lastly executed vertex is  " << *next_start_vertex);

  }

  if (octomap_update_thread.joinable())
  {
    octomap_update_thread.join();
  }

  return moved;
}

void ViewMotionPlanner::pathSearcherThread(EvalEpisodeEndParam ep, double duration)
{
  /*bool start_connected = */buildGraph();

  if (config.mode < Vmp_PLAN && config.insert_scan_if_not_moved)
    octree_manager->waitForPointcloudWithRoi();

  while (ros::ok() && config.mode >= Vmp_PLAN)
  {
    if (!searchPath() || config.mode < Vmp_PLAN_AND_EXECUTE)
    {
      if (config.insert_scan_if_not_moved)
        octree_manager->waitForPointcloudWithRoi();

      break;
    }

    bool moved = executePath();

    if (!moved && config.insert_scan_if_not_moved)
      octree_manager->waitForPointcloudWithRoi();

    if ( (ep == EvalEpisodeEndParam::TIME && octree_manager->getEvalPassedTime() > duration) ||
         (ep == EvalEpisodeEndParam::PLAN_DURATION && octree_manager->getEvalAccPlanDuration() > duration) ||
         (ep == EvalEpisodeEndParam::PLAN_LENGTH && octree_manager->getEvalAccPlanLength() > duration)
       )
    {
      break;
    }

    resumeGraphBuilderThreads();
    if (config.pause_graph_building_on_search)
    {
      ros::Duration(config.graph_building_time).sleep();
    }
  }
  pauseGraphBuilderThreads();
}

void ViewMotionPlanner::pathSearcherThread(const ros::Time &end_time)
{
  // publishWorkspaceMarker();
  // publishSamplingRegionMarker();
  if (eval_running)
  {
    octree_manager->startPlanning();
  }
  // Update the targets before starting to build the graph
  ros::Time updateTargetStartTime(ros::Time::now());
  octree_manager->updateTargets();
  ROS_INFO_STREAM("Updating targets took " << (ros::Time::now() - updateTargetStartTime) << " s");
  // Select which mode to be used - superellipsoid or inflation tree
  if (config.enable_superellipsoid_roi_sampling)
    updateMissingSurface(); // Update the missing surface to generate the potential ROI samples
  else
    updateInflatedTreeTargets(); // Update the inflationTree to generate the potential ROI samples
  
  // The current viewpose is added as a vertex to the graph and is connected to its neighbors
  bool start_connected = buildGraph();

  if (!start_connected)
  {
    ROS_ERROR("Program exited due to invalid start state which is the home pose");
  }

  if (config.mode < Vmp_PLAN && config.insert_scan_if_not_moved)
    octree_manager->waitForPointcloudWithRoi();

  while (ros::ok() && config.mode >= Vmp_PLAN && ros::Time::now() < end_time)
  {
    // Create the Gurobi graph, add constraints, optimize the graph and generate the optimized trajectories
    if (!optimizePath())
    {
      ROS_ERROR_STREAM("OPTIMIZATION FAILED");
      if (config.insert_scan_if_not_moved)
        octree_manager->waitForPointcloudWithRoi();

      // // Option 1 - continue building the graph and retry
      // resumeGraphBuilderThreads();
      // if (config.pause_graph_building_on_search)
      //   ros::Duration(config.graph_building_time).sleep();


      // Option 2 - Reset the graph and try again
      graph_manager->clear();
      start_connected = buildGraph();
      ROS_ERROR_STREAM("COMPLETED BUILDING GRAPH");


      // // Option 3 - Move to home pose, reset the graph and try again
      // try
      // {
      //   graph_manager->clear();
      //   start_connected = buildGraph();
      // }
      // catch(...)
      // {
      //   ROS_ERROR("Error in building the graph. So moving to home pose");

      //   if(!trolley_current_flipped)
      //     robot_manager->moveToNamedPose("home");
      //   else
      //     robot_manager->moveToNamedPose("home_flipped");
      //   graph_manager->clear();
      //   start_connected = buildGraph();
      // }

      continue;
    }
    
    if (config.mode < Vmp_PLAN_AND_EXECUTE)
      break;

    if (eval_running)
    {
      octree_manager->savePlanningData(eval_current_segment);
      octree_manager->startExecuting();
    }

    //bool moved = executePath();
    bool moved = executeOptimizedPath();

    // reset the model for the next iteration
    graph_optimizer->clear();

    if (!moved && config.insert_scan_if_not_moved)
      octree_manager->waitForPointcloudWithRoi();

    // Update the targets based on the previous executions
    updateTargetStartTime =ros::Time::now();
    octree_manager->updateTargets();
    ROS_INFO_STREAM("Updating targets took " << (ros::Time::now() - updateTargetStartTime) << " s");
    // Select which mode to be used
    if (config.enable_superellipsoid_roi_sampling)
      updateMissingSurface(); // Update the missing surface to generate the potential ROI samples
    else
      updateInflatedTreeTargets(); // Update the inflationTree to generate the potential ROI samples

    if (eval_running)
    {
      octree_manager->saveExecutionData(moved);
    }

    if (moved) // if successfully moved, then continue filling the graph and move to next iteration
    {
      if (eval_running)
      {
        octree_manager->startPlanning();
      }
      // 1. without resetting the graph
      resumeGraphBuilderThreads();
      if (config.pause_graph_building_on_search)
        ros::Duration(config.graph_building_time).sleep();
      // 2. with resetting the graph
      // graph_manager->clear();
      // start_connected = buildGraph();
    }
    else // if the executeOptimizedPath returns false if trajectory is 0
    {
      // // Option 1 - continue building the graph and retry
      // resumeGraphBuilderThreads();
      // if (config.pause_graph_building_on_search)
      //   ros::Duration(config.graph_building_time).sleep();


      // Option 2 - Reset the graph and try again
      // Move to home pose and reset the graph
      if(!trolley_current_flipped)
        robot_manager->moveToNamedPose("home");
      else
        robot_manager->moveToNamedPose("home_flipped");
      graph_manager->clear();

      if (eval_running)
      {
        octree_manager->startPlanning();
      }

      start_connected = buildGraph();


      // // Option 3 - Move to home pose, reset the graph and try again
      // try
      // {
      //   graph_manager->clear();
      //   start_connected = buildGraph();
      // }
      // catch(...)
      // {
      //   ROS_ERROR("Error in building the graph. So moving to home pose");

      //   if(!trolley_current_flipped)
      //     robot_manager->moveToNamedPose("home");
      //   else
      //     robot_manager->moveToNamedPose("home_flipped");
      //   graph_manager->clear();
      //   start_connected = buildGraph();
      // }
    }

  }
  pauseGraphBuilderThreads();

  if (eval_running)
  {
    octree_manager->savePlanningData(eval_current_segment);
    octree_manager->startExecuting();
    octree_manager->saveExecutionData(false);
  }

  // to make sure evaluator capture the last updated octomap information
  if (eval_running)
  {
    int last_segment; // get index of last segment
    if (config.trolley_flip_workspace) // double row
      last_segment = 2*config.trolley_num_segments*config.trolley_num_vertical_segments - 1;
    else // single row
      last_segment = config.trolley_num_segments*config.trolley_num_vertical_segments - 1;

    if (eval_current_segment==last_segment) // if the current segment is the final segment
      octree_manager->saveEvaluatorData(0, 0, eval_current_segment, trolley_remote.getPosition(), trolley_remote.getHeight());
  }
}

void ViewMotionPlanner::initGraphBuilderThreads()
{
  graph_builder_threads.reserve(NUM_GRAPH_BUILDER_THREADS);

  for (size_t i = 0; i < NUM_GRAPH_BUILDER_THREADS; i++)
  {
    graph_builder_threads.push_back(boost::move(boost::thread(boost::bind(&ViewMotionPlanner::graphBuilderThread, this))));
  }
}

void ViewMotionPlanner::pauseGraphBuilderThreads()
{
  graph_builder_condition.pause();
  graph_builder_condition.waitForPause();
}

void ViewMotionPlanner::resumeGraphBuilderThreads()
{
  graph_builder_condition.resume();
}

void ViewMotionPlanner::exploreNamedPoses()
{
  std::vector<std::string> pose_list;
  // A boolean to check flipped or not and choose the namedposes accordingly.
  if(!trolley_current_flipped)
  {
    //nh_.param<std::vector<std::string>>("view_motion_planner/pose_list", pose_list, {"explpose1", "explpose2", "explpose3", "explpose4", "explpose5", "home"});
    nh_.param<std::vector<std::string>>("view_motion_planner/pose_list", pose_list, {"LeftBottom", "RightBottom", "LeftTop", "RightTop", "home"});
  }
  else
  {
    //nh_.param<std::vector<std::string>>("view_motion_planner/pose_list", pose_list, {"explpose1_flipped", "explpose2_flipped", "explpose3_flipped", "explpose4_flipped", "explpose5_flipped", "home_flipped"});
    nh_.param<std::vector<std::string>>("view_motion_planner/pose_list", pose_list, {"LeftBottomFlipped", "RightBottomFlipped", "LeftTopFlipped", "RightTopFlipped", "home_flipped"});
  }

  boost::thread octomap_update_thread;
  for (const std::string &pose : pose_list)
  {
    robot_manager->moveToNamedPose(pose);
    ROS_INFO_STREAM("Moved to pose: " << pose);

    octree_manager->threadedWaitForPointcloudWithRoi(octomap_update_thread);
  }
  if (octomap_update_thread.joinable())
  {
    octomap_update_thread.join();
  }
  
  // // DEFAULT OPTION
  // for (const std::string &pose : pose_list)
  // {
  //   robot_manager->moveToNamedPose(pose);
  //   octree_manager->waitForPointcloudWithRoi();
  //   ROS_INFO_STREAM("Moved to pose: " << pose);
  // }
}

bool ViewMotionPlanner::startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                    bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist,
                    bool with_trolley)
{
  if (!eval_initialized || eval_running || eval_start) // Evaluation not initialized, alread in progress or about to start
    return false;

  eval_num_episodes = start_index + numEvals;
  eval_current_episode = start_index;
  eval_episode_duration = episodeDuration;
  eval_epend_param = episodeEndParam;
  eval_with_trolley = with_trolley;
  eval_current_segment = 0;

  bool success = octree_manager->startEvaluator(numEvals, episodeEndParam, episodeDuration, start_index,
    randomize_plants, min, max, min_dist, with_trolley);

  if (!success) return false;

  eval_start = true;
  return true;
}

void ViewMotionPlanner::flipWsAndSr()
{
  config.ws_min_y = -(config.ws_min_y - 0.3);
  config.ws_max_y = -(config.ws_max_y - 0.3);
  std::swap(config.ws_min_y, config.ws_max_y);
  config.sr_min_y = -(config.sr_min_y - 0.3);
  config.sr_max_y = -(config.sr_max_y - 0.3);
  std::swap(config.sr_min_y, config.sr_max_y);

  updateConfig();

  robot_manager->flipHomePose();

  publishWorkspaceMarker();
  publishSamplingRegionMarker();
}

bool ViewMotionPlanner::trolleyGoNextSegment()
{
  robot_manager->moveToHomePose(); // move arm to home pose before moving to next segment

  if (config.trolley_flip_workspace && !trolley_current_flipped)
  {
    flipWsAndSr();
    robot_manager->moveToHomePose();
    trolley_current_flipped = true;
    eval_current_segment++;
    ROS_INFO_STREAM("Flipping workspace");
    return true;
  }

  if (config.trolley_num_vertical_segments > 1 && trolley_current_vertical_segment < config.trolley_num_vertical_segments - 1)
  {
    trolley_current_vertical_segment++;
    ROS_INFO_STREAM("Lifting trolley");
    trolley_remote.liftTo(469.0 + trolley_current_vertical_segment * config.trolley_lift_dist);
    trolley_remote.waitForReady();
    ros::Duration(5).sleep(); // wait for transform to update
    // Lifting up sampling region to minimize the overlap between the crop levels
    config.sr_min_z += 0.4;
    config.sr_max_z += 0.4;
    updateConfig();
    if (config.trolley_flip_workspace && trolley_current_flipped)
    {
      flipWsAndSr();
      trolley_current_flipped = false;
    }
    robot_manager->moveToHomePose();
    eval_current_segment++;
    ROS_INFO_STREAM("Lifting workspace");
    return true;
  }

  if (config.trolley_num_segments > 1 && trolley_current_segment < config.trolley_num_segments - 1)
  {
    // Lifting down sampling region to minimize the overlap between the crop levels
    config.sr_min_z -= 0.4 * trolley_current_vertical_segment;
    config.sr_max_z -= 0.4 * trolley_current_vertical_segment;
    updateConfig();
    trolley_current_vertical_segment = 0;
    trolley_current_segment++;
    ROS_INFO_STREAM("Moving trolley");
    trolley_remote.moveTo(static_cast<float>(trolley_current_segment * config.trolley_move_length));
    trolley_remote.waitForReady();
    trolley_remote.liftTo(469.0);
    trolley_remote.waitForReady();
    ros::Duration(10).sleep(); // wait for transform to update. 10 seconds is required for the trolley to stabilize
    if (config.trolley_flip_workspace && trolley_current_flipped)
    {
      flipWsAndSr();
      trolley_current_flipped = false;
    }
    robot_manager->moveToHomePose();
    eval_current_segment++;
    ROS_INFO_STREAM("Moving workspace");
    return true;
  }

  // End of the glasshouse has reached. Save the point cloud for any post experiment analysis.
  octree_manager->saveFinalPcd();
  return false;
}

/*main path planning and execution loop*/
void ViewMotionPlanner::plannerLoop()
{    
  ROS_INFO_STREAM("PLANNER LOOP CALLED");

  // update map with the current observation
  octree_manager->waitForPointcloudWithRoi();

  // project 63 raycast lines from a sampled viewpose in observationPoses - not used
  pose_visualize_thread = boost::move(boost::thread(boost::bind(&ViewMotionPlanner::poseVisualizeThread, this)));
  pose_visualizer_condition.resume();

  // initialize the graph builder threads - 4 threads
  initGraphBuilderThreads();

  /*if (evaluation_mode)
  {
    ROS_INFO_STREAM("EVALUATION MODE ACTIVATED");
    config.mode = Vmp_PLAN_AND_EXECUTE;
    updateConfig();
    octree_manager->startEvaluator();
    for (size_t current_episode=0; ros::ok() && current_episode < eval_num_episodes; current_episode++)
    {
      pathSearcherThread(eval_epend_param, eval_episode_duration);
      pauseGraphBuilderThreads();
      robot_manager->moveToHomePose();
      octree_manager->resetOctomap();
      graph_manager->clear();
      graph_optimizer->clear();
      octree_manager->waitForPointcloudWithRoi();
      octree_manager->resetEvaluator();
    }
  }*/
  ROS_INFO_STREAM("ENTER MAIN LOOP");
  for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    if (eval_start)
    {
      ROS_INFO_STREAM("STARTING_EVALUATION");
      assert(!eval_running);
      if (eval_with_trolley)
      {
        config.trolley_segment_end_param = static_cast<int>(eval_epend_param);
        config.trolley_time_per_segment = static_cast<int>(eval_episode_duration);
        config.mode = Vmp_PLAN_WITH_TROLLEY;
      }
      else
      {
        config.mode = Vmp_PLAN_AND_EXECUTE;
      }

      updateConfig();
      eval_running = true;
      eval_start = false;
    }

    if (config.mode == Vmp_EXPLORE_NAMED_POSES)
    {
      ROS_INFO_STREAM("Explore named poses");
      exploreNamedPoses();
      config.mode = Vmp_IDLE;
      updateConfig();
    }
    else if (config.mode == Vmp_PLAN_WITH_TROLLEY)
    {
      // storage for past 1000 viewposes(past_viewposes_) - not used
      octree_manager->clearPastViewposesList();

      // Execute the preplanned poses at the 4 corners
      if (config.trolley_plan_named_poses)
      {
        ROS_INFO_STREAM("Explore named poses for segment");
        exploreNamedPoses();
      }
      ROS_INFO_STREAM("Planning new segment");
      // Clear all the iteratively updated datastructures
      graph_manager->clear();

      switch(config.trolley_segment_end_param)
      {
      case Vmp_TIME:
        pathSearcherThread(ros::Time::now() + ros::Duration(config.trolley_time_per_segment));
        break;
      case Vmp_PLAN_DURATION:
        pathSearcherThread(EvalEpisodeEndParam::PLAN_DURATION, (1 + eval_current_segment) * config.trolley_time_per_segment);
        break;
      case Vmp_PLAN_LENGTH:
        pathSearcherThread(EvalEpisodeEndParam::PLAN_LENGTH,  (1 + eval_current_segment) * config.trolley_time_per_segment);
        break;
      default:
        ROS_ERROR_STREAM("Invalid trolley segment end param: " << config.trolley_segment_end_param);
      }

      const std::string TROLLEY_TREE_PREFIX = "trolleyTree_segment";
      octree_manager->saveOctomap(TROLLEY_TREE_PREFIX + std::to_string(trolley_current_segment), true);
      graph_manager->clear();
      ROS_INFO_STREAM("Moving to home pose");
      robot_manager->moveToHomePose();
      if (!trolleyGoNextSegment()) // end reached, go to idle
      {
        if (eval_current_episode + 1 >= eval_num_episodes)
        {
          config.mode = Vmp_IDLE;
          updateConfig();
          continue;
        }
        robot_manager->moveToHomePose();
        trolley_remote.moveTo(0);
        trolley_remote.waitForReady();
        trolley_remote.liftTo(469.0);
        trolley_remote.waitForReady();
        ros::Duration(5).sleep(); // wait for transform to update

        // reset current segments
        trolley_current_segment = 0;
        trolley_current_vertical_segment = 0;
        eval_current_segment = 0;
        if (config.trolley_flip_workspace && trolley_current_flipped)
        {
          flipWsAndSr();
          trolley_current_flipped = false;
        }

        robot_manager->moveToHomePose();
        octree_manager->resetOctomap();
        graph_manager->clear();
        octree_manager->waitForPointcloudWithRoi();
        octree_manager->resetEvaluator();
        eval_current_episode++;
      }
      octree_manager->waitForPointcloudWithRoi();
      /*ROS_INFO_STREAM("Moving up");
      double h = trolley_remote.getHeight();
      ROS_INFO_STREAM("Height: " << h);
      const double LIFT = 500;
      static double sign = 1;
      trolley_remote.liftTo(sign * LIFT);
      ROS_INFO_STREAM("Lifting trolley");
      trolley_remote.waitForReady();
      ROS_INFO_STREAM("Trolley Ready");
      config.mode = Vmp_IDLE;
      sign *= -1;
      updateConfig();*/
    }
    else if (config.mode >= Vmp_BUILD_GRAPH)
    {
      ROS_INFO_STREAM("PLANNER BUILD GRAPH AND PLAN");
      if (eval_running)
      {
        pathSearcherThread(eval_epend_param, eval_episode_duration);
        pauseGraphBuilderThreads();
        robot_manager->moveToHomePose();
        octree_manager->resetOctomap();
        graph_manager->clear();
        octree_manager->waitForPointcloudWithRoi();
        octree_manager->resetEvaluator();
        eval_current_episode++;
        if (eval_current_episode >= eval_num_episodes)
        {
          eval_running = false;
          config.mode = Vmp_IDLE;
          updateConfig();
        }
      }
      else
      {

        pathSearcherThread();
      }
    }
    else if (config.mode == Vmp_MAP_ONLY)
    {
      ROS_INFO_STREAM("PLANNER MAPPING");
      octree_manager->waitForPointcloudWithRoi();
    }
    else
    {
      //ROS_INFO_STREAM("PLANNER IDLING");
    }
  }

  /*for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }*/
}

/*bool ViewMotionPlanner::plannerLoopOnce()
{
  std::vector<ViewposePtr> newPoses = octree_manager->sampleObservationPoses();
  observationPoseMtx.lock();
  observationPoses = newPoses;
  observationPoseMtx.unlock();
  octree_manager->publishObservationPoints(newPoses);
  //generateViewposeGraph();
  return false;
}*/

} // namespace view_motion_planner
