#include <ros/ros.h>
#include "pointcloud_roi/red_cluster_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_roi");

  ros::NodeHandle nhp("~");

  pointcloud_roi::RedClusterFilter filter(nhp);

  ros::spin();
}
