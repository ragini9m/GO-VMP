#include "view_motion_planner/octree_manager.h"
#include <ros/topic.h>
#include <octomap_msgs/conversions.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/counting_range.hpp>
#include <execution>
#include <rvp_evaluation/octree_provider_interfaces/provided_tree_interface.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <tf2_eigen/tf2_eigen.h>

#define RAYCAST_PARALLEL_LOOP

namespace view_motion_planner
{

OctreeManager::OctreeManager(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer,
                             const std::string &map_frame, const std::string &ws_frame, const std::string &pose_frame,
                             double tree_resolution, std::default_random_engine &random_engine,
                             std::shared_ptr<RobotManager> robot_manager, size_t num_sphere_vecs, bool update_planning_tree, bool initialize_evaluator) :
  nh(nh), robot_manager(robot_manager), random_engine(random_engine), tfBuffer(tfBuffer),
  planningTree(new octomap_vpp::RoiOcTree(tree_resolution)),
  observationRegions(new octomap_vpp::WorkspaceOcTree(tree_resolution)), gtLoader(new rvp_evaluation::GtOctreeLoader(tree_resolution)),
  evaluator(nullptr), tree_mtx(own_mtx), map_frame(map_frame), ws_frame(ws_frame), pose_frame(pose_frame), old_rois(0),
  sphere_vecs(getFibonacciSphereVectors(num_sphere_vecs)),
  update_planning_tree(update_planning_tree)
{
  // publishes the planningTree octomap
  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1, true);
  // publishes the inflated observation region - not used
  observationRegionsPub = nh.advertise<octomap_msgs::Octomap>("observation_regions", 1);
  // publishes the x,y,z coordinates of the keys in the observation region - not used
  observatonPointsPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("observation_points", 1);
  // publishes the current target voxels as RED - ROI, GREEN - Exploration, BLUE - Border
  targetPub = nh.advertise<visualization_msgs::Marker>("targets", 1, true);
  // publishes the number of observed voxels / total voxels in the sampling region cube - not used
  coveragePub = nh.advertise<std_msgs::Float64>("samplingRegionCoverage", 1, true);

#ifdef DBG_TARGETS_VPS
  dbg_target_sample_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("dbg_target_sample", 1);
  dbg_target_cloud.header.frame_id = map_frame;
  dbg_viewpoint_sample_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("dbg_viewpoint_sample", 1);
  dbg_viewpoint_cloud.header.frame_id = ws_frame;
#endif

  resetMoveitOctomapClient = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
  resetVoxbloxMapClient = nh.serviceClient<std_srvs::Empty>("/voxblox_node/clear_map");

  //past_viewposes_.reserve(1000);

  if (initialize_evaluator)
  {
    ros::NodeHandle nh_eval("evaluator");
    std::shared_ptr<rvp_evaluation::ProvidedTreeInterface> interface(new rvp_evaluation::ProvidedTreeInterface(planningTree, tree_mtx));
    evaluator.reset(new rvp_evaluation::Evaluator(interface, nh, nh_eval, true, false, gtLoader));
    external_cluster_evaluator.reset(new rvp_evaluation::ExternalClusterEvaluator(gtLoader));
    eval_trial_num = 0;
    evaluator->saveGtAsColoredCloud();
  }
}

/*update the map with the new pointcloud, update logodds, update the targets and publish the map*/
void OctreeManager::registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg)
{
  geometry_msgs::TransformStamped pcFrameTf;
  bool cloud_in_map_frame = (msg->cloud.header.frame_id == map_frame);
  if (cloud_in_map_frame)
  {
    //ROS_INFO_STREAM("Incoming cloud already in target frame");
    //pcFrameTf.header = roi.cloud.header;
    pcFrameTf.transform = msg->transform;
  }
  else
  {
    //ROS_INFO_STREAM("Convert incoming cloud (" << roi.cloud.header.frame_id << ") to map frame (" << map_frame << "), assuming no transform in incoming data");
    try
    {
      pcFrameTf = tfBuffer.lookupTransform(map_frame, msg->cloud.header.frame_id, msg->cloud.header.stamp);
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to map frame in registerRoiOCL: " << e.what());
      return;
    }
  }

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);

  // inlierCloud - stores the pointcloud data of the ROI
  // outlierCloud - stores the pointcloud data of the non-ROI
  // fullCloud - stores the pointcloud data of the full pointcloud
  octomap::Pointcloud inlierCloud, outlierCloud, fullCloud;
  if (cloud_in_map_frame)
    octomap_vpp::pointCloud2ToOctomapByIndices(msg->cloud, msg->roi_indices, inlierCloud, outlierCloud, fullCloud);
  else
    octomap_vpp::pointCloud2ToOctomapByIndices(msg->cloud, msg->roi_indices, pcFrameTf.transform, inlierCloud, outlierCloud, fullCloud);

  tree_mtx.lock();
  ros::Time insertStartTime(ros::Time::now());
  // fullCloud is used to update the map with occupancy information - occupied and free cells
  planningTree->insertPointCloud(fullCloud, scan_orig);
  // ROI and non-ROI keys are obtained and the logOdds for ROI are increased using prob_hit_log and the logOdds for non-ROI are decreased using prob_miss_log
  planningTree->insertRegionScan(inlierCloud, outlierCloud);
  ROS_INFO_STREAM("Inserting took " << (ros::Time::now() - insertStartTime) << " s");
  //ros::Time updateTargetStartTime(ros::Time::now());
  // current_roi_targets, current_expl_targets, current_border_targets lists are updated based on the current state of the map.
  // these voxels are published as different colors in rviz - RED - ROI, GREEN - Exploration, BLUE - Border
  // DOUBT - chances of duplication as we are using vector instead of set
  //updateTargets();
  //ROS_INFO_STREAM("Updating targets took " << (ros::Time::now() - updateTargetStartTime) << " s");
  ROS_INFO_STREAM("ROI targets: " << current_roi_targets.size() << ", Expl. targets: " << current_expl_targets.size());
  tree_mtx.unlock();
  // publishes the current planningTree octomap using octomapPub
  publishMap();
  //publishCoverage();
}

void OctreeManager::waitForPointcloudWithRoi()
{
  if (!update_planning_tree)
  {
    boost::unique_lock lock(tree_mtx);
    updateTargets();
    return;
  }

  // waits for the processed pointcloud from the Gazebo along with the ROI indices based on the color segmentation
  pointcloud_roi_msgs::PointcloudWithRoiConstPtr msg = ros::topic::waitForMessage<pointcloud_roi_msgs::PointcloudWithRoi>("/detect_roi/results", nh, ros::Duration(2.0));
  if (!msg)
  {
    ROS_ERROR_STREAM("Pointcloud message not received");
    return;
  }
  // ONCE THE MESSAGE IS RECEIVED, THE FUNCTION registerPointcloudWithRoi CAN BE RUNNED ON A SEPERATE THREAD TO PREVENT BLOCKING
  // IN THE NEXT ITERATION, CHECK IF THE PREVIOUS THREAD IS RUNNING. IF NO, WAIT FOR IT TO FINISH
  // update the map with the new pointcloud, update logOdds value, update the targets and publish the map, colored target voxels
  registerPointcloudWithRoi(msg);
}

void OctreeManager::threadedWaitForPointcloudWithRoi(boost::thread& parallel_map_update_thread)
{
  pointcloud_roi_msgs::PointcloudWithRoiConstPtr msg = ros::topic::waitForMessage<pointcloud_roi_msgs::PointcloudWithRoi>("/detect_roi/results", nh, ros::Duration(2.0));
  if (!msg)
  {
    ROS_ERROR_STREAM("Pointcloud message not received");
    return;
  }
  // run the below function as a thread
  if (parallel_map_update_thread.joinable())
  {
    parallel_map_update_thread.join();
  }
  parallel_map_update_thread = boost::thread(boost::bind(&OctreeManager::registerPointcloudWithRoi, this, msg));
}


std::vector<ViewposePtr> OctreeManager::sampleObservationPoses(double sensorRange)
{
  ros::Time startTime(ros::Time::now());
  std::vector<ViewposePtr> observationPoses;
  boost::mutex obsVpMtx;

  boost::unique_lock lock(tree_mtx);
  octomap::KeySet roiKeys = planningTree->getRoiKeys();

  #ifdef RAYCAST_PARALLEL_LOOP
  auto loop_policy = std::execution::par;
  #else
  auto loop_policy = std::execution::seq;
  #endif

  std::for_each(loop_policy, roiKeys.begin(), roiKeys.end(), [&](const octomap::OcTreeKey &roiKey)
  {
    octomap::point3d origin = planningTree->keyToCoord(roiKey);
    if (!isInSamplingRegion(transform(origin, map_frame, ws_frame))) // ROI not in sampling region
      return;

    std::for_each(loop_policy, sphere_vecs.begin(), sphere_vecs.end(), [&](const octomap::point3d &direction)
    {
      octomap::point3d end = origin + direction * sensorRange;
      octomap::KeyRay ray;
      planningTree->computeRayKeys(origin, end, ray);
      auto rayIt = ray.begin();
      rayIt++; // ignore origin cell (will always be occupied)
      bool intersects = false;
      for (auto rayEnd = ray.end(); rayIt != rayEnd; rayIt++)
      {
        octomap_vpp::RoiOcTreeNode *node = planningTree->search(*rayIt);
        if (node && planningTree->isNodeOccupied(node))
        {
          intersects = true;
          break;
        }
      }
      if (!intersects)
      {
        if (!isInWorkspace(transform(end, map_frame, ws_frame))) // Point not in workspace region
          return;

        ViewposePtr vp(new Viewpose());

        vp->pose.position = octomap::pointOctomapToMsg(end);
        vp->pose.orientation = tf2::toMsg(getQuatInDir(-direction));

        vp->state = robot_manager->getPoseRobotState(transform(vp->pose, map_frame, pose_frame));
        if (vp->state == nullptr)
          return;

        /* std::vector<double> joint_values = robot_manager->getPoseJointValues(transform(pose, map_frame, pose_frame));
        if (joint_values.empty())
          return; */

        obsVpMtx.lock();
        observationPoses.push_back(vp);
        obsVpMtx.unlock();
      }
    });
  });

  ROS_INFO_STREAM("Generating observation poses took " << (ros::Time::now() - startTime));
  return observationPoses;
}

std::vector<octomap::point3d> OctreeManager::getTargetVoxels()
{
  return current_filtered_targets;
}

void OctreeManager::updateFilteredTargets(octomap::KeySet filtered_keys)
{
  current_filtered_targets.clear();
  current_filtered_targets.reserve(filtered_keys.size());

  for (const auto& key : filtered_keys) {
      octomap::point3d point = inflatedTree->keyToCoord(key);
      current_filtered_targets.push_back(point);
  }
}

void OctreeManager::updateFilteredTargets(std::vector<octomap::point3d> filtered_points)
{
  current_filtered_targets.clear();
  current_filtered_targets = std::move(filtered_points);
}

void OctreeManager::updateFilteredTargetsKeys(octomap::KeySet filtered_keys)
{
  potential_roi_voxels.clear();
  potential_roi_voxels = filtered_keys;
}

octomap::KeySet OctreeManager::getFilteredTargetsKeys()
{
  return potential_roi_voxels;
}

octomap::KeySet OctreeManager::getRoiTargetsKeys()
{
  return current_roi_keys;
}

// lock tree mutex before calling function
void OctreeManager::updateRoiTargets()
{
  std::vector<octomap::point3d> new_roi_targets;
  octomap::KeySet new_roi_keys;

  octomap::KeySet roi = planningTree->getRoiKeys();
  octomap::KeySet freeNeighbours;
  for (const octomap::OcTreeKey &key : roi)
  {
    if (!isInSamplingRegion(transform(planningTree->keyToCoord(key), map_frame, ws_frame))) // ROI not in sampling region
      continue;

    planningTree->getNeighborsInState(key, freeNeighbours, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::FREE_NONROI, octomap_vpp::NB_18);
  }
  for (const octomap::OcTreeKey &key : freeNeighbours)
  {
    if (planningTree->hasNeighborInState(key, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::UNKNOWN, octomap_vpp::NB_18))
    {
      new_roi_targets.push_back(planningTree->keyToCoord(key));
      new_roi_keys.insert(key);
    }
  }

  current_roi_targets = new_roi_targets;
  current_roi_keys = new_roi_keys;
}

// lock tree mutex before calling function
void OctreeManager::updateExplTargets()
{
  std::vector<octomap::point3d> new_expl_targets;
  std::vector<octomap::point3d> new_border_targets;
  octomap::point3d srMin_tf = transform(octomap::point3d(config.sr_min_x, config.sr_min_y, config.sr_min_z), ws_frame, map_frame);
  octomap::point3d srMax_tf = transform(octomap::point3d(config.sr_max_x, config.sr_max_y, config.sr_max_z), ws_frame, map_frame);
  for (unsigned int i = 0; i < 3; i++)
  {
    if (srMin_tf(i) > srMax_tf(i))
      std::swap(srMin_tf(i), srMax_tf(i));
  }
  for (auto it = planningTree->begin_leafs_bbx(srMin_tf, srMax_tf), end = planningTree->end_leafs_bbx(); it != end; it++)
  {
    //if (!isInSamplingRegion(transform(it.getCoordinate(), map_frame, ws_frame)))
    //  continue;

    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (planningTree->hasNeighborInState(it.getKey(), octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::UNKNOWN, octomap_vpp::NB_6))
      {
        if (planningTree->hasNeighborInState(it.getKey(), octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::OCCUPIED_ROI, octomap_vpp::NB_6))
          new_expl_targets.push_back(it.getCoordinate());
        else
          new_border_targets.push_back(it.getCoordinate());
      }
    }
  }

  current_expl_targets = std::move(new_expl_targets);
  current_border_targets = std::move(new_border_targets);
}

// lock tree mutex before calling function
void OctreeManager::updateTargets()
{
  boost::unique_lock lock(target_vector_mtx);
  updateRoiTargets();
  updateExplTargets();
  publishTargets();
}

bool OctreeManager::getRandomFilteredTarget(octomap::point3d &target)
{
  std::uniform_int_distribution<size_t> distribution(0, current_filtered_targets.size() - 1);
  target = current_filtered_targets[distribution(random_engine)];
  return true;
}

bool OctreeManager::getRandomRoiTarget(octomap::point3d &target)
{
  if (current_roi_targets.empty())
    return false;

  std::uniform_int_distribution<size_t> distribution(0, current_roi_targets.size() - 1);
  target = current_roi_targets[distribution(random_engine)];
  return true;
}

bool OctreeManager::getRandomExplTarget(octomap::point3d &target)
{
  if (current_expl_targets.empty())
    return false;

  std::uniform_int_distribution<size_t> distribution(0, current_expl_targets.size() - 1);
  target = current_expl_targets[distribution(random_engine)];
  return true;
}

bool OctreeManager::getRandomBorderTarget(octomap::point3d &target)
{
  if (current_border_targets.empty())
    return false;

  std::uniform_int_distribution<size_t> distribution(0, current_border_targets.size() - 1);
  target = current_border_targets[distribution(random_engine)];
  return true;
}

ViewposePtr OctreeManager::sampleRandomViewPose(TargetType type)
{
  octomap::point3d origin;
  bool sample_target_success = false;
  target_vector_mtx.lock();
  if (type == TARGET_ROI) // can consist of both TARGET ROI AND TARGET FILTER
  {
    std::uniform_real_distribution<double> target_select_dist(0.0, 1.0);
    double target_selector = target_select_dist(random_engine);
    if (!current_filtered_targets.empty() && target_selector <= config.filtered_target_percentage)
    {
      sample_target_success = getRandomFilteredTarget(origin);
      if (!sample_target_success)
        sample_target_success = getRandomRoiTarget(origin);
      else
        type = TARGET_FILTER;
    }
    else
    {
      sample_target_success = getRandomRoiTarget(origin);
    }
    if (!sample_target_success)
      type = TARGET_OCC;
  }
  if (type == TARGET_OCC)
  {
    sample_target_success = getRandomExplTarget(origin);
    if (!sample_target_success)
      type = TARGET_BORDER;
  }
  if (type == TARGET_BORDER)
  {
    sample_target_success = getRandomBorderTarget(origin);
  }
  if (!sample_target_success)
  {
    target_vector_mtx.unlock();
    return nullptr;
  }
  target_vector_mtx.unlock();
  octomap::point3d end;
  if (config.vp_select_type == Vmp_RANGE)
  {
    bool found_vp = false;
    for (size_t i=0; i < 100; i++)
    {
      end = sampleRandomViewpoint(origin, config.sensor_min_range, config.sensor_max_range, random_engine);
      if (isInWorkspace(transform(end, map_frame, ws_frame))) // Point in workspace region
      {
        found_vp = true;
        break;
      }
    }
    if (!found_vp)
    {
      return nullptr;
    }
  }
  // else if (config.enable_superellipsoid_roi_sampling)
  // {
  //   bool found_vp = false;
  //   octomap::OcTreeKey origin_key = planningTree->coordToKey(origin); // get the key of the origin
  //   // octomath::Vector3 x_axis;
  //   // if (config.sr_max_y>config.sr_min_y)
  //   //   x_axis = octomath::Vector3(1,0,0); //left to right
  //   // else
  //   //   x_axis = octomath::Vector3(-1,0,0); //right to left

  //   // look through the 9 neighbours randomly in each sample
  //   std::vector<int> rand_order(9);
  //   std::iota(rand_order.begin(), rand_order.end(), 0);
  //   std::shuffle(rand_order.begin(), rand_order.end(), random_engine);

  //   for (size_t i = 0; i < 9; i++)
  //   {
  //     // Get the key of the neighbour.
  //     octomap::OcTreeKey neighbour_key;

  //     if (config.sr_max_y>config.sr_min_y) // check right 9 voxels in case of sampling region is on left side
  //       neighbour_key = octomap::OcTreeKey(origin_key[0] + octomap_vpp::nb9LutRight[rand_order[i]][0], origin_key[1] + octomap_vpp::nb9LutRight[rand_order[i]][1], origin_key[2] + octomap_vpp::nb9LutRight[rand_order[i]][2]);
  //     else // check left 9 voxels in case of sampling region is on right side
  //       neighbour_key = octomap::OcTreeKey(origin_key[0] + octomap_vpp::nb9LutLeft[rand_order[i]][0], origin_key[1] + octomap_vpp::nb9LutLeft[rand_order[i]][1], origin_key[2] + octomap_vpp::nb9LutLeft[rand_order[i]][2]);

  //     // Check for free or unknown neighbour
  //     octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key);
  //     if (node == NULL || node->getLogOdds() <= 0)
  //     {
  //       // Generate a direction vector from origin to neighbour and normalize it
  //       octomath::Vector3 dir_vec = (planningTree->keyToCoord(neighbour_key) - origin).normalize();

  //       // Below code is not needed as we check only the 9 nieghbours in the direction of FOV
  //       // // Check if the angle beween x-axis(origin towards camera) and the direction vector is within 160 degrees field of view (80 degrees on either side)
  //       // double dot_product = x_axis.dot(dir_vec);
  //       // double angle_degrees = std::acos(dot_product) * 180.0 / M_PI;
  //       // // check if the angle is within the FOV - 160 degrees is set as the FOV. (80 degrees on either side)
  //       // bool is_fov = (angle_degrees <= (160 / 2.0));

  //       // if (is_fov)
  //       // {
  //       //   // Generate an end point at a random distance between sensor_min_range to sensor_max_range in the direction of the neighbour_key
  //       //   std::uniform_real_distribution<double> dist_distribution(config.sensor_min_range, config.sensor_max_range);
  //       //   end = origin + dir_vec * dist_distribution(random_engine);

  //       //   if (isInWorkspace(transform(end, map_frame, ws_frame))) // Check if the viewPoint in workspace region
  //       //   {
  //       //     found_vp = true;
  //       //     break;
  //       //   }
  //       // }

  //       // Generate an end point at a random distance between sensor_min_range to sensor_max_range in the direction of the neighbour_key
  //       std::uniform_real_distribution<double> dist_distribution(config.sensor_min_range, config.sensor_max_range);
  //       end = origin + dir_vec * dist_distribution(random_engine);

  //       if (isInWorkspace(transform(end, map_frame, ws_frame))) // Check if the viewPoint in workspace region
  //       {
  //         found_vp = true;
  //         break;
  //       }
  //     }
  //   }
  //   if (!found_vp)
  //   {
  //     return nullptr;
  //   }
  // }
  else
  {
    bool found_vp = false;
    for (size_t i=0; i < 100; i++)
    {
      octomap::point3d end_ws = sampleRandomWorkspacePoint(); // uses min/max coordinates of workspace
      end = transform(end_ws, ws_frame, map_frame);
      if (config.vp_select_type == Vmp_WORKSPACE_RANGE)
      {
        double dist = end.distance(origin);
        if (dist < config.sensor_min_range || dist > config.sensor_max_range)
          continue;
      }
      //if (isInWorkspace(end_ws)) // Point in workspace region
      //{
      found_vp = true;
      break;
      //}
    }
    if (!found_vp)
    {
      return nullptr;
    }
  }
  octomap::KeyRay ray;

  boost::unique_lock lock(tree_mtx);
  planningTree->computeRayKeys(origin, end, ray);
  auto rayIt = ray.begin();
  for (auto rayEnd = ray.end(); rayIt != rayEnd; rayIt++)
  {
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(*rayIt);
    if (node && planningTree->isNodeOccupied(node))
    {
      return nullptr;
    }
  }

  ViewposePtr vp(new Viewpose());

  vp->pose.position = octomap::pointOctomapToMsg(end);
  vp->pose.orientation = tf2::toMsg(getQuatInDir((origin - end).normalize()));
  vp->origin = origin;
  vp->dir_vec = (end - origin).normalize();
  /*if(isViewpointSimilarToPastViewpoints(past_viewposes_, vp))
  {
    return nullptr;
  }*/

  vp->state = robot_manager->getPoseRobotState(vp->pose);

  // if (vp->state)
  // {
  //   const Eigen::Affine3d& end_effector_state = vp->state->getGlobalLinkTransform("camera_link");
  //   // Convert Eigen::Affine3d to geometry_msgs::Pose
  //   geometry_msgs::Pose pose;
  //   pose.position.x = end_effector_state.translation().x();
  //   pose.position.y = end_effector_state.translation().y();
  //   pose.position.z = end_effector_state.translation().z();

  //   Eigen::Quaterniond quat(end_effector_state.rotation());
  //   pose.orientation.x = quat.x();
  //   pose.orientation.y = quat.y();
  //   pose.orientation.z = quat.z();
  //   pose.orientation.w = quat.w();

  //   tf2::Quaternion tf2_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  //   double roll, pitch, yaw;
  //   tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

  //   //ROS_INFO_STREAM("Transformed Pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", " << roll << ", " << pitch << ", " << yaw);
  //   ROS_INFO_STREAM("Transformed Pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w);
  //   ROS_INFO_STREAM("Original Pose: " << vp->pose.position.x << ", " << vp->pose.position.y << ", " << vp->pose.position.z << ", " << vp->pose.orientation.x << ", " << vp->pose.orientation.y << ", " << vp->pose.orientation.z << ", " << vp->pose.orientation.w);
  // }

  // // Option 1 -Use the computed state as the vp->pose
  // if (vp->state) // if state is valid
  // {
  //   const Eigen::Affine3d& end_effector_state = vp->state->getGlobalLinkTransform("camera_link");

  //   // correct the postion
  //   vp->pose.position.x = end_effector_state.translation().x();
  //   vp->pose.position.y = end_effector_state.translation().y();
  //   vp->pose.position.z = end_effector_state.translation().z();

  //   // correct the orientation
  //   Eigen::Quaterniond quaternion(end_effector_state.rotation());
  //   vp->pose.orientation = tf2::toMsg(quaternion);

  //   // correct the origin which is used for visualization and fov check
  //   vp->origin = octomap::point3d(vp->pose.position.x - vp->dir_vec.x(), vp->pose.position.y - vp->dir_vec.y(), vp->pose.position.z - vp->dir_vec.z());
  // }

  // Option 2 - change the position(vp->pose.position) a little bit forward and backward along the direction vector

  vp->type = type;

#ifdef DBG_TARGETS_VPS
  dbg_target_cloud.push_back(octomap_vpp::octomapPointToPcl<pcl::PointXYZ>(origin));
  dbg_viewpoint_cloud.push_back(octomap_vpp::octomapPointToPcl<pcl::PointXYZ>(transform(end, map_frame, ws_frame)));

  static ros::Time dbg_last_print_time = ros::Time::now();
  static size_t dbg_valid_states = 0;
  static size_t dbg_invalid_states = 0;
  ros::Time dbg_cur_time = ros::Time::now();
  if ((dbg_cur_time - dbg_last_print_time).toSec() > 1.0)
  {
    ROS_INFO_STREAM("States valid: " << dbg_valid_states << "; invalid: " << dbg_invalid_states);
    dbg_target_cloud.header.stamp = pcl_conversions::toPCL(dbg_cur_time);
    dbg_target_sample_pub.publish(dbg_target_cloud);
    dbg_viewpoint_cloud.header.stamp = pcl_conversions::toPCL(dbg_cur_time);
    dbg_viewpoint_sample_pub.publish(dbg_viewpoint_cloud);
    dbg_last_print_time = dbg_cur_time;
  }
#endif

  if (vp->state == nullptr)
  {
#ifdef DBG_TARGETS_VPS
    dbg_invalid_states++;
#endif
    return nullptr;
  }
#ifdef DBG_TARGETS_VPS
  dbg_valid_states++;
#endif
  return vp;
}

void OctreeManager::setMissingShapes(const superellipsoid_msgs::MissingEllipsoidSurfaceParams& msg)
{
  missing_shapes.clear(); // store the missing shapes as MissingShape
  missing_surface_targets.clear(); // store the missing surface points as pcl::PointCloud

  // Convert centers_optimized to octomap::point3d
  pcl::PointCloud<pcl::PointXYZ> centers_optimized_cloud;
  pcl::fromROSMsg(msg.centers_optimized, centers_optimized_cloud);

  // // Convert centers_prior to octomap::point3d
  // pcl::PointCloud<pcl::PointXYZ> centers_prior_cloud;
  // pcl::fromROSMsg(msg.centers_prior, centers_prior_cloud);

  // iterate through each cluster
  for (size_t i = 0; i < msg.missing_surfaces.size(); i++)
  {
    MissingShape ellipsoid;

    // Add centers_optimized
    if (!centers_optimized_cloud.empty()) {
      auto& point = centers_optimized_cloud.points[i]; // get the centroid
      ellipsoid.completed_shape_centre = octomap::point3d(point.x, point.y, point.z); // store the centroid
    }

    // Check if the fruit cluster is in the sampling region. else continue for loop
    if (!isInSamplingRegion(transform(ellipsoid.completed_shape_centre, map_frame, ws_frame)))
      continue;
    else
      ROS_WARN_STREAM("Completed shape " << i << " has " << ellipsoid.completed_shape_centre << " as centre");

    // // Add centers_prior
    // if (!centers_prior_cloud.empty()) {
    //   auto& point = centers_prior_cloud.points[i];
    //   ellipsoid.missing_shape_centre = octomap::point3d(point.x, point.y, point.z);
    //   ROS_WARN_STREAM("Missing shape " << i << " has " << ellipsoid.missing_shape_centre << " as centre");
    // }

    // Maintain the missing surface cloud as pcl::PointCloud - to access normals
    pcl::fromROSMsg(msg.missing_surfaces[i], ellipsoid.missing_surface_pcl_cloud);

    if (config.inflate_missing_surface)
    {
      // Inflate the missing surface cloud
      for (size_t i = 0; i < ellipsoid.missing_surface_pcl_cloud.points.size(); ++i)
      {
        // Move the point in the opposite direction of its normal vector
        ellipsoid.missing_surface_pcl_cloud.points[i].x -= ellipsoid.missing_surface_pcl_cloud.points[i].normal_x * config.inflation_distance;
        ellipsoid.missing_surface_pcl_cloud.points[i].y -= ellipsoid.missing_surface_pcl_cloud.points[i].normal_y * config.inflation_distance;
        ellipsoid.missing_surface_pcl_cloud.points[i].z -= ellipsoid.missing_surface_pcl_cloud.points[i].normal_z * config.inflation_distance;
      }
    }
    ellipsoid.num_missing_points = ellipsoid.missing_surface_pcl_cloud.size();
    ROS_WARN_STREAM("Missing shape " << i << " has " << ellipsoid.missing_surface_pcl_cloud.size() << " missing points");

    // All the missing points are stored here which will be further filtered out
    missing_surface_targets += ellipsoid.missing_surface_pcl_cloud;

    missing_shapes.push_back(ellipsoid);

    completed_surface_percentage = msg.mapped_surface;
  }
}

std::pair<octomap::KeySet, octomap::KeySet> OctreeManager::filterMissingShapes()
{
  // 1. Convert the points to octomap::point3d
  std::vector<octomap::point3d> octomap_points;
  octomap_points.reserve(missing_surface_targets.points.size());

  for (const auto& point : missing_surface_targets.points)
  {
    octomap::point3d octo_point(point.x, point.y, point.z);
    octomap_points.push_back(octo_point);
  }

  // 2. Reducing the number of voxels to be used as samples by filtering

  std::vector<octomap::point3d> filtered_points; // store the filtered points
  filtered_points.reserve(octomap_points.size());

  // 2.1 Filter the missing surface points having a free voxel in the neighbourhood
  if (config.select_voxels_with_free_neigh && octomap_points.size() > 10)
  {
    for (const auto& point : octomap_points)
    {
      // check if there is a free voxel in neighbourhood of the missing surface point in the planning tree
      if (planningTree->hasNeighborInState(planningTree->coordToKey(point), octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::FREE_NONROI, octomap_vpp::NB_18))
        filtered_points.push_back(point);
    }
  }

  // 2.2 Filter the missing surface points that are in the visible surface contour of the sampling space
  else if (config.select_voxels_on_front && octomap_points.size() > 10)
  {
    std::unordered_set<octomap::point3d> unseen_points(octomap_points.begin(), octomap_points.end()); // iterate and keep the unseen points within this variable

    // Sample some points in the workspace, cast rays in planningTree and check for occlusion
    // if the missing surface point is not occluded in atleast 1 view, then add it to the filtered_points
    for (int ray = 0; ray < config.number_of_visibility_check_points; ray++)
    {
      // Sample a point in the workspace
      octomap::point3d cam_point = transform(sampleRandomWorkspacePoint(), ws_frame, map_frame);

      // Check for occlusion with the planning tree
      octomap::KeyRay ray1;

      for (auto it = unseen_points.begin(); it != unseen_points.end(); )
      {
        bool ret1 = planningTree->computeRayKeys(*it, cam_point, ray1);
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
          // Search for occlusion in the planning tree
          octomap_vpp::RoiOcTreeNode *node = planningTree->search(*rayIt);
          // if occlusion, then continue
          if (node && planningTree->isNodeOccupied(node))
          {
            occlusion = true;
            break;
          }
        }
        if (occlusion) // if occluded view, continue to next point
        {
          ++it;
          continue;
        }
        else // if not occluded in atleast one workspace point, then add it to the filtered_points
        {
          filtered_points.push_back(*it); // add the point to the filtered_points
          it = unseen_points.erase(it); // point is already seen, so remove it from the unseen_points to be not used in next iteration
        }
      }
    }
  }

  // 3. To visualize the results
  octomap::KeySet all_voxels;
  octomap::KeySet selected_voxels;
  octomap::KeySet not_selected_voxels;

  for (const auto& point : octomap_points) {
    octomap::OcTreeKey key = planningTree->coordToKey(point);
    all_voxels.insert(key);
  }

  for (const auto& point : filtered_points) {
    octomap::OcTreeKey key = planningTree->coordToKey(point);
    selected_voxels.insert(key);
  }

  for (const auto& key : all_voxels) {
    if (selected_voxels.find(key) == selected_voxels.end()) {
      not_selected_voxels.insert(key);
    }
  }

  // 3. If filtering process is done, then set that as the target points
  if (filtered_points.size() > 10) // make sure atleast 10 points are selected for sampling. If not enough, then use all the octomap_points
  {
    octomap_points.clear();
    octomap_points = std::move(filtered_points);
  }

  // 4. Add to targets that will be used for sampling
  updateFilteredTargetsKeys(selected_voxels); // update keys
  updateFilteredTargets(octomap_points); // update points


  return std::make_pair(selected_voxels, not_selected_voxels);
}

/*
std::shared_ptr<octomap_vpp::WorkspaceOcTree> OctreeManager::computeObservationRegions(double inflation_radius)
{
  struct KeyWithPrio
  {
    KeyWithPrio(const octomap::OcTreeKey &key, float prio) : key(key), prio(prio) {}

    octomap::OcTreeKey key;
    float prio;

    bool operator< (const KeyWithPrio &rhs) const
    {
      return prio < rhs.prio;
    }
  };

  typedef boost::heap::fibonacci_heap<KeyWithPrio> ValueHeap;
  typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ValueHeap::handle_type, octomap::OcTreeKey::KeyHash> KeyHandleMap;

  ValueHeap nodeVals;
  KeyHandleMap openKeys;
  octomap::KeySet processedKeys;

  octomap::KeySet observationPoints = sampleObservationPoints();
  publishObservationPoints(observationPoints);

  return observationRegions; // DEBUG: no inflation for now

  for (const octomap::OcTreeKey &key : observationPoints)
  {
    ValueHeap::handle_type handle = nodeVals.push(KeyWithPrio(key, 1.0));
    openKeys[key] = handle;
  }

  float stepReduction = 1.0 / inflation_radius * observationRegions->getResolution();

  ROS_INFO("Start inflation");

  ros::Time startTime(ros::Time::now());
  ros::Time lastPrintTime(startTime);

  while (!nodeVals.empty())
  {
    const KeyWithPrio &maxEl = nodeVals.top();
    octomap::OcTreeKey curKey = maxEl.key;
    float curVal = maxEl.prio;
    nodeVals.pop();
    openKeys.erase(curKey);
    observationRegions->updateNode(curKey, curVal, false);
    processedKeys.insert(curKey);

    ros::Time curTime(ros::Time::now());
    if (curTime - lastPrintTime > ros::Duration(1)) // print every second
    {
      ROS_INFO_STREAM("Estimated progress: " << ((1.0 - curVal) * 100.0) << "%");
      lastPrintTime = curTime;
      if (!ros::ok())
        break;
    }

    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        for (int k = -1; k <= 1; k++)
        {
          int coordDiff = std::abs(i) + std::abs(j) + std::abs(k);
          float newNeighbourVal = 0;
          if (coordDiff == 0) continue; // only process neighbours, not the node itself
          else if (coordDiff == 1) newNeighbourVal = curVal - stepReduction;
          else if (coordDiff == 2) newNeighbourVal = curVal - stepReduction * sqrt(2);
          else newNeighbourVal = curVal - stepReduction * sqrt(3); // coordDiff == 3

          if (newNeighbourVal <= 0) continue; // new value is out of influence radius

          octomap::OcTreeKey neighbourKey(curKey[0] + i, curKey[1] + j, curKey[2] + k);
          if (processedKeys.find(neighbourKey) != processedKeys.end()) continue; // already processed keys can be ignored

          auto it = openKeys.find(neighbourKey);
          if (it != openKeys.end()) // key already known, check if update needed
          {
            ValueHeap::handle_type handle = it->second;
            if (newNeighbourVal > (*handle).prio) // only update if new value would be higher
            {
              (*handle).prio = newNeighbourVal;
              nodeVals.increase(handle);
            }
          }
          else // otherwise, enter new key to map
          {
            ValueHeap::handle_type handle = nodeVals.push(KeyWithPrio(neighbourKey, newNeighbourVal));
            openKeys[neighbourKey] = handle;
          }
        }
      }
    }
  }
  observationRegions->updateInnerVals();
  ros::Time endTime(ros::Time::now());
  ROS_INFO_STREAM("Inflation done, took " << (endTime - startTime).toSec() << "s");
  return observationRegions;
}
*/

std::string OctreeManager::saveOctomap(const std::string &name, bool name_is_prefix)
{
  std::stringstream fName;
  fName << name;
  if (name_is_prefix)
  {
    const boost::posix_time::ptime curDateTime = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const timeFacet = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    fName.imbue(std::locale(fName.getloc(), timeFacet));
    fName << "_" << curDateTime;
  }
  fName << ".ot";
  tree_mtx.lock();
  bool result = planningTree->write(fName.str());
  tree_mtx.unlock();
  return result ? fName.str() : "";
}

void OctreeManager::moveOctomap(octomap_vpp::RoiOcTree* &tree, const geometry_msgs::Transform &offset)
{
  bool translate_tree = (offset.translation.x != 0 || offset.translation.y != 0 || offset.translation.z != 0);
  bool rotate_tree =  (offset.rotation.x != 0 || offset.rotation.y != 0 || offset.rotation.z != 0);

  if (!translate_tree && !rotate_tree)
    return;

  octomap_vpp::RoiOcTree *moved_tree(new octomap_vpp::RoiOcTree(tree->getResolution()));

  if (!rotate_tree) // no rotation needed, simply offset keys
  {
    const octomap::OcTreeKey ZERO_KEY = tree->coordToKey(0, 0, 0);
    const octomap::OcTreeKey OFFSET_KEY = tree->coordToKey(offset.translation.x, offset.translation.y, offset.translation.z);
    auto addOffset = [&ZERO_KEY, &OFFSET_KEY](const octomap::OcTreeKey &key)
    {
      return octomap::OcTreeKey(key[0] - ZERO_KEY[0] + OFFSET_KEY[0], key[1] - ZERO_KEY[1] + OFFSET_KEY[1], key[2] - ZERO_KEY[2] + OFFSET_KEY[2]);
    };
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; it++)
    {
      octomap::OcTreeKey insertKey = addOffset(it.getKey());
      octomap_vpp::RoiOcTreeNode *node = moved_tree->setNodeValue(insertKey, it->getLogOdds(), true);
      node->setRoiLogOdds(it->getRoiLogOdds());
    }
  }
  else // fully transform coordinate
  {
    octomap::pose6d transform = octomap_vpp::transformToOctomath(offset);
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; it++)
    {
      octomap::point3d insertPoint = transform.transform(it.getCoordinate());
      octomap_vpp::RoiOcTreeNode *node = moved_tree->setNodeValue(insertPoint, it->getLogOdds(), true);
      node->setRoiLogOdds(it->getRoiLogOdds());
    }
  }

  delete tree;
  tree = moved_tree;
}

int OctreeManager::loadOctomap(const std::string &filename, const geometry_msgs::Transform &offset)
{
  octomap_vpp::RoiOcTree *map = nullptr;
  octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(filename);
  if (!tree)
    return -1;

  map = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
  if(!map)
  {
    delete tree;
    return -2;
  }

  moveOctomap(map, offset);

  tree_mtx.lock();
  planningTree.reset(map);
  planningTree->computeRoiKeys();
  updateTargets();
  tree_mtx.unlock();
  publishMap();
  return 0;
}

void OctreeManager::resetOctomap()
{
  tree_mtx.lock();
  planningTree->clear();
  planningTree->clearRoiKeys();
  old_rois = 0;
  tree_mtx.unlock();
  encountered_keys.clear();

  if (!resetMoveitOctomapClient.call(emptySrv))
  {
    ROS_ERROR("Failed to reset moveit octomap");
  }
  if (!resetVoxbloxMapClient.call(emptySrv))
  {
    ROS_WARN("Failed to reset voxblox octomap");
  }

  publishMap();
}

void OctreeManager::randomizePlants(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist)
{
  if (evaluator)
    evaluator->randomizePlantPositions(octomap_vpp::pointToOctomath(min), octomap_vpp::pointToOctomath(max), min_dist);
  else
    gtLoader->randomizePlantPositions(octomap_vpp::pointToOctomath(min), octomap_vpp::pointToOctomath(max), min_dist);
}

void OctreeManager::publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(*planningTree, map_msg);
  tree_mtx.unlock();
  if (msg_generated)
  {
    octomapPub.publish(map_msg);
  }
}

void OctreeManager::publishObservationRegions()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  bool msg_generated = octomap_msgs::fullMapToMsg(*observationRegions, map_msg);
  if (msg_generated)
  {
    observationRegionsPub.publish(map_msg);
  }
}

void OctreeManager::publishObservationPoints(const octomap::KeySet &keys)
{
  //octomap::point3d (octomap_vpp::WorkspaceOcTree::*keyToCoord)(const octomap::OcTreeKey &key) const = &octomap_vpp::WorkspaceOcTree::keyToCoord;
  auto keyToCoord = [this](const octomap::OcTreeKey &key) {return observationRegions->keyToCoord(key);};
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc = octomap_vpp::octomapKeysetToPcl<pcl::PointXYZ>(keys, keyToCoord);
  pc->header.frame_id = map_frame;
  pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp);
  observatonPointsPub.publish(pc);
}

void OctreeManager::publishObservationPoints(const std::vector<ViewposePtr> &vps)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
  pc->reserve(vps.size());
  for (const ViewposePtr &vp : vps)
  {
    pc->push_back(pcl::PointXYZ(vp->pose.position.x, vp->pose.position.y, vp->pose.position.z));
  }
  pc->header.frame_id = map_frame;
  pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp);
  observatonPointsPub.publish(pc);
}

void OctreeManager::publishTargets()
{
  static const std_msgs::ColorRGBA COLOR_RED = []{std_msgs::ColorRGBA c; c.r = 1.f; c.g = 0.f; c.b = 0.f; c.a = 1.f; return c; } ();
  static const std_msgs::ColorRGBA COLOR_GREEN = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 1.f; c.b = 0.f; c.a = 1.f; return c; } ();
  static const std_msgs::ColorRGBA COLOR_BLUE = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 0.f; c.b = 1.f; c.a = 1.f; return c; } ();

  const size_t NUM_P = current_roi_targets.size() + current_expl_targets.size() + current_border_targets.size();
  if (NUM_P == 0)
  {
    ROS_WARN("No targets");
    return;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = map_frame;
  m.header.stamp = ros::Time();
  m.ns = "targets";
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.005;
  m.scale.y = 0.005;
  m.color.a = 1.0;
  m.points.reserve(NUM_P);
  m.colors.reserve(NUM_P);
  for (octomap::point3d &p : current_roi_targets)
  {
    m.points.push_back(octomap::pointOctomapToMsg(p));
    m.colors.push_back(COLOR_RED);
  }
  for (octomap::point3d &p : current_expl_targets)
  {
    m.points.push_back(octomap::pointOctomapToMsg(p));
    m.colors.push_back(COLOR_GREEN);
  }
  for (octomap::point3d &p : current_border_targets)
  {
    m.points.push_back(octomap::pointOctomapToMsg(p));
    m.colors.push_back(COLOR_BLUE);
  }
  targetPub.publish(m);
}

void OctreeManager::publishCoverage()
{
  octomap::point3d min = transform(octomap::point3d(config.ws_min_x, config.ws_min_y, config.ws_min_z), ws_frame, map_frame);
  octomap::point3d max = transform(octomap::point3d(config.ws_max_x, config.ws_max_y, config.ws_max_z), ws_frame, map_frame);
  for (size_t i = 0; i < 3; i++)
  {
    if (min(i) > max(i))
      std::swap(min(i), max(i));
  }
  std_msgs::Float64 msg;
  static octomap_vpp::RoiOcTree::CoverageInfo last_ci;
  auto ci = planningTree->getVolumeCoverage(min, max);
  ROS_ERROR_STREAM("Cov: " << ci.covered_cells << ", " << ci.total_cells);
  ROS_ERROR_STREAM("New cells: " << (ci.covered_cells - last_ci.covered_cells));
  msg.data = static_cast<double>(ci.covered_cells) / static_cast<double>(ci.total_cells);
  coveragePub.publish(msg);
  last_ci = ci;
}

bool OctreeManager::startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                                   bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist,
                                   bool with_trolley)
{
  eval_trial_num = 0;
  setEvaluatorStartParams();
  return true;
}

void OctreeManager::setEvaluatorStartParams()
{
  std::string file_index_str = std::to_string(eval_trial_num);
  // eval_resultsFile = std::ofstream("planner_results_" + file_index_str + ".csv");
  // eval_resultsFileOld = std::ofstream("planner_results_old" + file_index_str + ".csv");
  eval_externalClusterFile = std::ofstream("planner_results_ec" + file_index_str + ".csv");
  eval_planningtimeFile = std::ofstream("planning_results" + file_index_str + ".csv");
  eval_planning_durationFile = std::ofstream("planning_duration" + file_index_str + ".csv");
  eval_volume_accuracyFile = std::ofstream("volume_accuracy" + file_index_str + ".csv");
  eval_surface_coverageFile = std::ofstream("surface_coverage" + file_index_str + ".csv");
  eval_roi_coverageFile = std::ofstream("roi_coverage" + file_index_str + ".csv");
  // eval_fruitCellPercFile = std::ofstream("results_fruit_cells_" + file_index_str + ".csv");
  // eval_volumeAccuracyFile = std::ofstream("results_volume_accuracy_" + file_index_str + ".csv");
  // eval_distanceFile = std::ofstream("results_distances_" + file_index_str + ".csv");
  // eval_resultsFile << "Time (s),Plan duration (s),Plan Length,";
  // evaluator->writeHeader(eval_resultsFile) << ",Step,Segment,Trolley Position,Trolley Height" << std::endl;
  // eval_resultsFileOld << "Time (s),Plan duration (s),Plan Length,";
  // evaluator->writeHeaderOld(eval_resultsFileOld) << ",Step,Segment,Trolley Position,Trolley Height" << std::endl;
  eval_externalClusterFile << "Time (s),Plan duration (s),Plan Length,";
  external_cluster_evaluator->writeHeader(eval_externalClusterFile)<< ",Step,Segment,Trolley Position,Trolley Height" << std::endl;
  eval_planningtimeFile << "Time (s),Segment" << std::endl;
  eval_planning_durationFile << "Segment,Plan Duration(s),Execution Duration(s)" << std::endl;
  eval_volume_accuracyFile << "Time (s),Volume Accuracy Percentage of each fruit" << std::endl;
  eval_surface_coverageFile << "Time (s),Surface Coverage Percentage of each fruit" << std::endl;
  eval_roi_coverageFile << "Time (s),ROI Coverage Percentage of each fruit" << std::endl;
  eval_plannerStartTime = ros::Time::now();
  eval_passedTime = 0;
  eval_accumulatedPlanDuration = 0;
  eval_accumulatedPlanLength = 0;
}

template<typename T>
std::ostream& writeVector(std::ostream &os, double passed_time, const std::vector<T> &vec)
{
  os << passed_time << ",";
  for (size_t i = 0; i < vec.size(); i++)
  {
    os << vec[i];
    if (i < vec.size() - 1)
      os << ",";
  }
  return os;
}

bool OctreeManager::saveEvaluatorData(double plan_length, double traj_duration, size_t segment, double trolley_pos, double trolley_height)
{
  ros::Time currentTime = ros::Time::now();

  eval_passedTime = (currentTime - eval_plannerStartTime).toSec();

  eval_accumulatedPlanDuration += traj_duration;
  eval_accumulatedPlanLength += plan_length;

  // rvp_evaluation::EvaluationParameters res = evaluator->processDetectedRois(true, eval_trial_num, static_cast<size_t>(eval_passedTime));
  // rvp_evaluation::EvaluationParametersOld resOld = evaluator->processDetectedRoisOld();

  // eval_resultsFile << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  // evaluator->writeParams(eval_resultsFile, res) << "," << eval_lastStep << "," << segment << "," << trolley_pos << "," << trolley_height << std::endl;

  // eval_resultsFileOld << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  // evaluator->writeParamsOld(eval_resultsFileOld, resOld) << "," << eval_lastStep << "," << segment << "," << trolley_pos << "," << trolley_height << std::endl;

  eval_externalClusterFile << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  external_cluster_evaluator->writeParams(eval_externalClusterFile, external_cluster_evaluator->getCurrentParams()) << "," << eval_lastStep << "," << segment << "," << trolley_pos << "," << trolley_height << std::endl;

  // writeVector(eval_fruitCellPercFile, eval_passedTime, res.fruit_cell_percentages) << std::endl;
  // writeVector(eval_volumeAccuracyFile, eval_passedTime, res.volume_accuracies) << std::endl;
  // writeVector(eval_distanceFile, eval_passedTime, res.distances) << std::endl;

  rvp_evaluation::EvaluationParameters res = evaluator->processVolumeRois();
  writeVector(eval_volume_accuracyFile, eval_passedTime, res.volume_accuracies) << std::endl; // HULL VOLUME
  writeVector(eval_roi_coverageFile, eval_passedTime, res.fruit_cell_percentages) << std::endl; // ROI overlap


  if (!config.enable_superellipsoid_roi_sampling)
  {
    superellipsoid_msgs::MissingEllipsoidSurfaceParamsConstPtr msg = ros::topic::waitForMessage<superellipsoid_msgs::MissingEllipsoidSurfaceParams>("/capsicum_superellipsoid_detector/vmp_missing_surfaces", nh, ros::Duration(2.0));
    completed_surface_percentage = msg->mapped_surface;
  }

  writeVector(eval_surface_coverageFile, eval_passedTime, completed_surface_percentage) << std::endl;

  return true;
}

void OctreeManager::savePlannerData(size_t segment)
{
  eval_planningtimeFile << (ros::Time::now() - eval_plannerStartTime).toSec() << "," << segment << std::endl;
}

void OctreeManager::savePlanningData(size_t segment)
{
  eval_planning_durationFile << segment << "," << (ros::Time::now() - eval_planningStartTime).toSec() << ",";
}

void OctreeManager::saveExecutionData(bool moved)
{
  if (moved)
  {
    eval_planning_durationFile << (ros::Time::now() - eval_executionStartTime).toSec() << ",True" << std::endl;
  }
  else
  {
    eval_planning_durationFile << (ros::Time::now() - eval_executionStartTime).toSec() << ",False" << std::endl;
  }
}

void OctreeManager::startPlanning()
{
  eval_planningStartTime = ros::Time::now();
}

void OctreeManager::startExecuting()
{
  eval_executionStartTime = ros::Time::now();
}


bool OctreeManager::resetEvaluator()
{
  eval_resultsFile.close();
  eval_resultsFileOld.close();
  eval_externalClusterFile.close();
  eval_planningtimeFile.close();
  eval_planning_durationFile.close();
  eval_volume_accuracyFile.close();
  eval_surface_coverageFile.close();
  eval_roi_coverageFile.close();
  eval_fruitCellPercFile.close();
  eval_volumeAccuracyFile.close();
  eval_distanceFile.close();
  eval_trial_num++;
  setEvaluatorStartParams();
  return true;
}

} // namespace view_motion_planner
