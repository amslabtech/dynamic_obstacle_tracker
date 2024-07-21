/**
 * @file dynamic_obstacle_tracker.cpp
 * @brief Dynamic obstacle tracker class
 * @author amsl
 * @copyright Copyright (c) 2024
 */

#include <string>
#include <vector>

#include "dynamic_obstacle_tracker/dynamic_obstacle_tracker.h"

DynamicObstacleTracker::DynamicObstacleTracker(void) : private_nh_("~"), tf_listener_(tf_buffer_)
{
  private_nh_.param<std::string>("global_frame", param_.global_frame, std::string("map"));
  private_nh_.param<float>("min_dist_th", param_.min_dist_th, 1.0);

  poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/dynamic_obstacle/poses", 1);
  paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacle/trajectories", 1);
  cloud_sub_ = nh_.subscribe("/cloud/dynamic", 1, &DynamicObstacleTracker::cloud_callback, this);
}

void DynamicObstacleTracker::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // transform pointcloud to global frame
  geometry_msgs::TransformStamped transform_stamped;
  while (ros::ok())
  {
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(param_.global_frame, cloud_msg->header.frame_id, ros::Time(0));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.5).sleep();
    }
  }
  const Eigen::Matrix4f transform = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  sensor_msgs::PointCloud2 cloud_transformed;
  pcl_ros::transformPointCloud(transform, *cloud_msg, cloud_transformed);
  cloud_transformed.header.frame_id = param_.global_frame;

  // clustering and tracking
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_transformed, *pcl_cloud);
  track(clustering(pcl_cloud));
}

geometry_msgs::PoseArray DynamicObstacleTracker::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // === clustering ===
  // kd-tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  // execute clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
  ece.setClusterTolerance(0.5);
  ece.setMinClusterSize(10);
  ece.setMaxClusterSize(cloud->points.size());
  ece.setSearchMethod(tree);
  ece.setInputCloud(cloud);
  ece.extract(cluster_indices);

  // === extract cluster ===
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::ExtractIndices<pcl::PointXYZ> ei;
  ei.setInputCloud(cloud);
  ei.setNegative(false);
  for (size_t i = 0; i < cluster_indices.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr clustered_indices(new pcl::PointIndices);
    *clustered_indices = cluster_indices[i];
    ei.setIndices(clustered_indices);
    ei.filter(*cloud_clustered);
    clusters.push_back(cloud_clustered);
  }

  // === publish ===
  geometry_msgs::PoseArray pose_array;
  pose_array.header = pcl_conversions::fromPCL(cloud->header);
  for (size_t i = 0; i < clusters.size(); i++)
  {
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*clusters[i], xyz_centroid);
    geometry_msgs::Pose pose;
    pose.position.x = xyz_centroid[0];
    pose.position.y = xyz_centroid[1];
    pose.position.z = xyz_centroid[2];
    pose_array.poses.push_back(pose);
  }
  poses_pub_.publish(pose_array);

  return pose_array;
}

void DynamicObstacleTracker::track(geometry_msgs::PoseArray poses)
{
  if (paths_.empty())  // initialize
  {
    for (size_t i = 0; i < poses.poses.size(); i++)
    {
      nav_msgs::Path path;
      path.header = poses.header;
      geometry_msgs::PoseStamped pose;
      pose.pose = poses.poses[i];
      path.poses.push_back(pose);
      paths_.push_back(path);
    }
  }
  else  // update
  {
    int index_check_array[poses.poses.size()] = {0};
    std::vector<int> new_index;
    for (size_t i = 0; i < poses.poses.size(); i++)
    {
      // search nearest path
      float min_dist = 1e6;
      int min_idx = -1;
      for (size_t j = 0; j < paths_.size(); j++)
      {
        const float dist = sqrt(
            pow(poses.poses[i].position.x - paths_[j].poses.back().pose.position.x, 2) +
            pow(poses.poses[i].position.y - paths_[j].poses.back().pose.position.y, 2));
        if (dist < min_dist)
        {
          min_dist = dist;
          min_idx = j;
        }
      }

      if (min_dist < param_.min_dist_th && index_check_array[min_idx] == 0)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose = poses.poses[i];
        paths_[min_idx].poses.push_back(pose);
        index_check_array[min_idx] = 1;
      }
      else if (min_dist >= param_.min_dist_th)
      {
        new_index.push_back(i);
      }
    }

    // remove path
    for (size_t i = 0; i < paths_.size(); i++)
      if (index_check_array[i] == 0)
        paths_.erase(paths_.begin() + i);

    // add new path
    if (!new_index.empty())
    {
      for (size_t i = 0; i < new_index.size(); i++)
      {
        nav_msgs::Path path;
        path.header = poses.header;
        geometry_msgs::PoseStamped pose;
        pose.pose = poses.poses[new_index[i]];
        path.poses.push_back(pose);
        paths_.push_back(path);
      }
    }

    visualize_trajectories(paths_);
  }
}

void DynamicObstacleTracker::visualize_trajectories(const std::vector<nav_msgs::Path> &paths)
{
  visualization_msgs::MarkerArray v_trajectories;
  for (size_t i = 0; i < paths.size(); i++)
  {
    if (paths[i].poses.size() < 2)
      continue;

    std_msgs::ColorRGBA color;
    color.g = 1.0;
    visualization_msgs::Marker v_trajectory = create_marker_msg(i, 0.1, color, paths[i]);
    v_trajectories.markers.push_back(v_trajectory);
  }

  if (!v_trajectories.markers.empty())
    paths_pub_.publish(v_trajectories);
}

visualization_msgs::Marker DynamicObstacleTracker::create_marker_msg(
    const int id, const double scale, const std_msgs::ColorRGBA color, const nav_msgs::Path &path)
{
  visualization_msgs::Marker marker;
  marker.header = path.header;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = scale;
  marker.color = color;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(0.1);

  geometry_msgs::Point p;
  for (const auto &pose : path.poses)
  {
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    marker.points.push_back(p);
  }

  return marker;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_obstacle_tracker");
  DynamicObstacleTracker dot;
  ros::spin();

  return 0;
}
