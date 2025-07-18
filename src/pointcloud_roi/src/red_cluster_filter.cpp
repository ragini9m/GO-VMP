#include "pointcloud_roi/red_cluster_filter.h"
#include "pointcloud_roi/utils.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>


namespace pointcloud_roi
{

RedClusterFilter::RedClusterFilter(ros::NodeHandle &nhp)
{
  transform_pointcloud = nhp.param<bool>("transform_pointcloud", true);
  target_frame = nhp.param<std::string>("map_frame", "world");
  //bool transform_to_map_frame = nhp.param<bool>("publish_separate_clouds", false);
  //bool publish_combined_cloud = nhp.param<bool>("publish_combined_cloud", true);
  //bool publish_separate_clouds = nhp.param<bool>("publish_separate_clouds", false);

  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nhp));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nhp, "input", 1));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, target_frame, 1000, nhp));
  transform_filter->registerCallback(&RedClusterFilter::filter, this);
  pc_roi_pub = nhp.advertise<pointcloud_roi_msgs::PointcloudWithRoi>("results", 1);
  roi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("roi_cloud", 1);
  nonroi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("nonroi_cloud", 1);
  // for debugging the clipping parameters for the world
  full_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("filtered_live_cloud", 1);

  dynrec_server.reset(new dynamic_reconfigure::Server<pointcloud_roi::ClipWorldConfig>(nhp));
  dynrec_server->setCallback(boost::bind(&RedClusterFilter::reconfigureCallback, this, _1, _2));

}

pcl::IndicesConstPtr RedClusterFilter::filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (output == nullptr)
    output.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  //inRange((30, 25, 15), (80, 255,255))
  //pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GE, 0)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LE, 35)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::GE, 50)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::GE, 70)));
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LT, -20)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GT, 35)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::LT, 30)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::LT, 30)));

  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(true);
  condrem.setInputCloud(input);
  condrem.setCondition(color_cond);
  condrem.filter(*output);
  return condrem.getRemovedIndices();
}

void RedClusterFilter::filter(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc, *pcl_cloud);

  pcl::IndicesConstPtr redIndices = filterRed(pcl_cloud);

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tf_buffer->lookupTransform(target_frame, pc->header.frame_id, pc->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  ROS_INFO_STREAM("Transform for time " << pc->header.stamp << " successful");

  static Eigen::Isometry3d lastTfEigen;
  Eigen::Isometry3d tfEigen = tf2::transformToEigen(pcFrameTf);

  if (tfEigen.isApprox(lastTfEigen, 1e-2)) // Publish separate clouds only if not moved
  {
    const auto [inlier_cloud, outlier_cloud] = separateCloudByIndices<pcl::PointXYZRGB>(pcl_cloud, redIndices);
    roi_only_pub.publish(*inlier_cloud);
    //nonroi_only_pub.publish(*outlier_cloud);
  }
  lastTfEigen = tfEigen;

  if (transform_pointcloud)
  {
    pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, tfEigen.matrix());
    pcl_cloud->header.frame_id = target_frame;
  }

  // ////***** CROPPING ******////
  // ////***** Filter out points outside a certain distance in the Y and Z directions *****////
  // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, config.world_x_min_crop)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, config.world_x_max_crop)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, config.world_y_min_crop)));
  // //range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, config.world_y_max_crop)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, config.world_z_min_crop)));
  // //range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, config.world_z_max_crop)));

  // pcl::ConditionalRemoval<pcl::PointXYZRGB> range_removal;
  // range_removal.setCondition(range_cond);
  // range_removal.setInputCloud(pcl_cloud);
  // range_removal.filter(*pcl_cloud);

  // full_pub.publish(*pcl_cloud);
  // //***** Filter out points outside a certain distance in the Y and Z directions *****////
  // ////***** CROPPING ******////

  ////***** CLIPPING ******////
  for (auto& point : pcl_cloud->points)
  {
    if (point.x < config.world_x_min_crop) point.x = config.world_x_min_crop;
    if (point.x > config.world_x_max_crop) point.x = config.world_x_max_crop;
    if (point.y < config.world_y_min_crop) point.y = config.world_y_min_crop;
    if (point.y > config.world_y_max_crop) point.y = config.world_y_max_crop;
    if (point.z < config.world_z_min_crop) point.z = config.world_z_min_crop;
    if (point.z > config.world_z_max_crop) point.z = config.world_z_max_crop;
  }
  full_pub.publish(*pcl_cloud);
  ////***** CLIPPING ******////


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_ds(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (pcl_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  //vg.setSaveLeafLayout(true);
  vg.filter (*pcl_cloud_ds);

  redIndices = filterRed(pcl_cloud_ds);
  pcl::IndicesPtr redIndicesRo(new std::vector<int>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> ro;
  ro.setInputCloud(pcl_cloud_ds);
  ro.setIndices(redIndices);
  ro.filter(*redIndicesRo);

  pointcloud_roi_msgs::PointcloudWithRoi res;
  pcl::toROSMsg(*pcl_cloud_ds, res.cloud);
  res.cloud.header.stamp = pc->header.stamp;
  res.roi_indices.assign(redIndicesRo->begin(), redIndicesRo->end());
  if (transform_pointcloud)
  {
    res.cloud.header.frame_id = target_frame;
    res.transform = pcFrameTf.transform;
  }
  else
  {
    res.cloud.header.frame_id = pc->header.frame_id;
    res.transform.rotation.w = 1;
  }

  pc_roi_pub.publish(res);
}

void RedClusterFilter::reconfigureCallback(pointcloud_roi::ClipWorldConfig &new_config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure callback called, level " << level);

  // Adjust sensor range if necessary
  if (level & (1 << 20)) // minimum sensor range
  {
    if (new_config.world_x_max_crop < new_config.world_x_min_crop)
      new_config.world_x_max_crop = new_config.world_x_min_crop;
  }
  else if (level & (1 << 21)) // maximum sensor range
  {
    if (new_config.world_x_min_crop > new_config.world_x_max_crop)
      new_config.world_x_min_crop = new_config.world_x_max_crop;
  }

  this->config = new_config;
}

} // namespace pointcloud_roi
