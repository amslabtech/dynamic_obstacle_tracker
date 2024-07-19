/**
 * @file dynamic_obstacle_tracker.h
 * @brief Dynamic obstacle tracker class
 * @author amsl
 * @copyright Copyright (c) 2024
 */

#ifndef DYNAMIC_OBSTACLE_TRACKER_DYNAMIC_OBSTACLE_TRACKER_H
#define DYNAMIC_OBSTACLE_TRACKER_DYNAMIC_OBSTACLE_TRACKER_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * @struct Params
 * @brief Parameters
 */
struct Params
{
  std::string global_frame;
  float min_dist_th;
};

/**
 * @class DynamicObstacleTracker
 * @brief Dynamic obstacle tracker class
 */
class DynamicObstacleTracker
{
public:
  /**
   * @brief Constructor
   */
  DynamicObstacleTracker(void);

private:
  /**
   * @brief Callback function for point cloud
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  /**
   * @brief Clustering point cloud
   * @param cloud Point cloud
   * @return Pose array of dynamic obstacles
   */
  geometry_msgs::PoseArray clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /**
   * @brief Track dynamic obstacles
   * @param poses Pose array of dynamic obstacles
   */
  void track(geometry_msgs::PoseArray poses);

  /**
   * @brief Visualize trajectories
   * @param paths Paths
   */
  void visualize_trajectories(const std::vector<nav_msgs::Path> &paths);

  /**
   * @brief Create marker message
   * @param id Marker ID
   * @param scale Width of marker
   * @param color Color of marker
   * @param path Path
   * @param header Header
   */
  visualization_msgs::Marker
  create_marker_msg(const int id, const double scale, const std_msgs::ColorRGBA color, const nav_msgs::Path &path);

  Params params_;
  std::vector<nav_msgs::Path> paths_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher paths_pub_;
  ros::Publisher poses_pub_;
  ros::Subscriber cloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif  // DYNAMIC_OBSTACLE_TRACKER_DYNAMIC_OBSTACLE_TRACKER_H
