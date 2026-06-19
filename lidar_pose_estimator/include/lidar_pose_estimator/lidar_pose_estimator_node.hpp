#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

namespace lidar_pose_estimator
{

class LidarPoseEstimatorNode : public rclcpp::Node
{
public:
  explicit LidarPoseEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  void saveMap(const std::string & path);

private:
  //Callbacks
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  //Helpers
  void accumulateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in_map);
  void publishAll(const rclcpp::Time & stamp);   // odometry + pose + TF + path
  void publishMap();
  void logTum(const rclcpp::Time & stamp);

  //Parameters
  std::string lidar_topic_;
  std::string map_pcd_path_;
  std::string tum_path_;

  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_epsilon_;
  int    ndt_max_iter_;
  double ndt_fitness_threshold_;
  double voxel_leaf_size_;
  double map_voxel_leaf_size_;
  double max_scan_range_;
  double min_scan_range_;
  int    ndt_max_points_;         
  int    map_publish_every_;
  int    map_min_points_;         
  int    map_revoxel_every_;      

  //State
  bool initialized_{false};

  Eigen::Isometry3d T_map_lidar_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_delta_{Eigen::Isometry3d::Identity()};

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;

  nav_msgs::msg::Path lidar_path_msg_;
  std::ofstream       tum_file_;

  int frame_count_{0};
  int consecutive_failures_{0};

  //ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             lidar_path_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}
