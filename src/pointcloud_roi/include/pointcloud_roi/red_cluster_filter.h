#ifndef RED_CLUSTER_FILTER_H
#define RED_CLUSTER_FILTER_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <pointcloud_roi_msgs/ClipWorldConfig.h>

namespace pointcloud_roi
{

class RedClusterFilter
{
public:
  RedClusterFilter(ros::NodeHandle &nhp);

  void filter(const sensor_msgs::PointCloud2ConstPtr &pc);


private:
  bool transform_pointcloud;
  std::string target_frame;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;

  pointcloud_roi::ClipWorldConfig config;
  std::unique_ptr<dynamic_reconfigure::Server<pointcloud_roi::ClipWorldConfig>> dynrec_server;

  ros::Publisher pc_roi_pub;
  ros::Publisher roi_only_pub;
  ros::Publisher nonroi_only_pub;
  // for debugging the clipping parameters for the world
  ros::Publisher full_pub;
  
  void reconfigureCallback(pointcloud_roi::ClipWorldConfig &config, uint32_t level);

  pcl::IndicesConstPtr filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output = nullptr);
};

} // namespace pointcloud_roi

#endif // RED_CLUSTER_FILTER_H
