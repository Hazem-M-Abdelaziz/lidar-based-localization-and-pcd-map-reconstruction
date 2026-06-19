#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <deque>
#include <fstream>
#include <optional>
#include <string>

namespace lidar_pose_estimator
{

class EvaluationNode : public rclcpp::Node
{
public:
  explicit EvaluationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  void shutdown();

private:
  //Callbacks
  void mapCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void enuOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  //Service
  void saveTrigger(
    const std_srvs::srv::Trigger::Request::SharedPtr  req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);

  //Helpers
  bool            saveMapToDisk();
  void            tryInitOrigins(const nav_msgs::msg::Odometry & enu_msg);
  Eigen::Vector3d enuToMap(const geometry_msgs::msg::Point & p) const;
  void            computeAndPublishError(const rclcpp::Time & stamp);

  //Parameters
  std::string map_cloud_topic_;
  std::string lidar_odom_topic_;
  std::string enu_odom_topic_;
  std::string output_pcd_path_;
  std::string output_tum_path_;
  std::string output_error_csv_path_;
  double      error_window_sec_;
  double      enu_yaw_offset_deg_; 

  //Frame alignment state
  bool              origins_set_{false};

  // Full transform from ENU world frame → LiDAR map frame applied to every incoming ENU position, so both paths share the same frame
  Eigen::Isometry3d T_enu_to_map_{Eigen::Isometry3d::Identity()};

  //Latest poses
  std::optional<Eigen::Vector3d>    latest_lidar_pos_;
  std::optional<Eigen::Quaterniond> latest_lidar_rot_;
  std::optional<Eigen::Vector3d>    latest_enu_pos_in_map_;

  //Captured at the first synchronised moment for auto-alignment
  std::optional<Eigen::Vector3d>    first_lidar_pos_;
  std::optional<Eigen::Quaterniond> first_lidar_rot_;

  //Path accumulators
  nav_msgs::msg::Path lidar_path_msg_;
  nav_msgs::msg::Path enu_path_msg_;

  //Rolling RMSE
  struct ErrorEntry { double t; double error_m; };
  std::deque<ErrorEntry> error_history_;

  //Map cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_map_;
  bool map_received_{false};

  //File streams
  std::ofstream tum_file_;
  std::ofstream error_csv_;

  int frame_count_{0};

  //ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       lidar_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       enu_odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    lidar_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    enu_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
};

}
