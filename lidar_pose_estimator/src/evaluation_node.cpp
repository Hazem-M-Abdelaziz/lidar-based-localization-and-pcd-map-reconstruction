//Including required 
#include "lidar_pose_estimator/evaluation_node.hpp"
#include <iomanip>
#include <cmath>

namespace lidar_pose_estimator
{

static constexpr double DEG2RAD = M_PI / 180.0;

EvaluationNode::EvaluationNode(const rclcpp::NodeOptions & options)
: Node("lidar_evaluation", options)
  //Node parameters and values.
{
  map_cloud_topic_       = declare_parameter<std::string>("map_cloud_topic",       "/map_cloud");
  lidar_odom_topic_      = declare_parameter<std::string>("lidar_odom_topic",      "/lidar_odom");
  enu_odom_topic_        = declare_parameter<std::string>("enu_odom_topic",        "/fixposition/odometry_enu");
  output_pcd_path_       = declare_parameter<std::string>("output_pcd_path",       "/tmp/lidar_map_eval.pcd");
  output_tum_path_       = declare_parameter<std::string>("output_tum_path",       "/tmp/lidar_tum_eval.txt");
  output_error_csv_path_ = declare_parameter<std::string>("output_error_csv_path", "/tmp/pose_error.csv");
  error_window_sec_      = declare_parameter<double>     ("error_window_sec",      5.0);
  enu_yaw_offset_deg_    = declare_parameter<double>     ("enu_yaw_offset_deg",    0.0);
  //object to hold points for latest map while iterating.
  latest_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  //Frame id for both lidar generated path and enu map.
  lidar_path_msg_.header.frame_id = "map";
  enu_path_msg_.header.frame_id   = "map";

  //TUM file (Deliverable)
  tum_file_.open(output_tum_path_);
  if (tum_file_.is_open()) {
    tum_file_ << "# TUM format: timestamp tx ty tz qx qy qz qw\n";
    tum_file_ << "# Source: " << lidar_odom_topic_ << "\n";
  } else {
    RCLCPP_WARN(get_logger(), "Cannot open TUM file '%s'", output_tum_path_.c_str());
  }

  //Error CSV file as evaluation data to be furtherly visualized if needed.
  error_csv_.open(output_error_csv_path_);
  if (error_csv_.is_open()) {
    error_csv_ << "frame,stamp,lidar_x,lidar_y,lidar_z,"
                  "enu_x,enu_y,enu_z,error_m,rmse_m\n";
  } else {
    RCLCPP_WARN(get_logger(), "Cannot open CSV file '%s'", output_error_csv_path_.c_str());
  }

  //Subscribers
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    map_cloud_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&EvaluationNode::mapCloudCallback, this, std::placeholders::_1));

  lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    lidar_odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&EvaluationNode::lidarOdomCallback, this, std::placeholders::_1));

  enu_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    enu_odom_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
    std::bind(&EvaluationNode::enuOdomCallback, this, std::placeholders::_1));

  //Publishers
  lidar_path_pub_ = create_publisher<nav_msgs::msg::Path>("/lidar_path", 10);
  enu_path_pub_   = create_publisher<nav_msgs::msg::Path>("/enu_path",   10);
  error_pub_      = create_publisher<std_msgs::msg::Float64>("/pose_error", 10);

  //Service
  save_srv_ = create_service<std_srvs::srv::Trigger>(
    "/save_map",
    std::bind(&EvaluationNode::saveTrigger, this,
              std::placeholders::_1, std::placeholders::_2));

  //Info to be logged
  RCLCPP_INFO(get_logger(),
    "EvaluationNode ready.\n"
    "  lidar odom topic -> %s\n"
    "  ENU odom topic   -> %s\n"
    "  PCD output       -> %s\n"
    "  TUM output       -> %s\n"
    "  error CSV        -> %s\n"
    "  yaw offset       -> %.1f deg  (0 = auto-align)\n"
    "Waiting for first LiDAR + ENU messages to auto-align frames...",
    lidar_odom_topic_.c_str(), enu_odom_topic_.c_str(),
    output_pcd_path_.c_str(), output_tum_path_.c_str(),
    output_error_csv_path_.c_str(), enu_yaw_offset_deg_);
}

//map_cloud callback
void EvaluationNode::mapCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, *latest_map_);
  map_received_ = true;
}

//lidar_odom callback
void EvaluationNode::lidarOdomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;

  latest_lidar_pos_ = Eigen::Vector3d(p.x, p.y, p.z);
  latest_lidar_rot_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);

  // Cache first LiDAR pose for auto-alignment
  if (!first_lidar_pos_.has_value()) {
    first_lidar_pos_ = *latest_lidar_pos_;
    first_lidar_rot_ = *latest_lidar_rot_;
  }

  //Appending to LiDAR path
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = msg->header.stamp;
    ps.header.frame_id = "map";
    ps.pose            = msg->pose.pose;
    lidar_path_msg_.header.stamp = msg->header.stamp;
    lidar_path_msg_.poses.push_back(ps);
    lidar_path_pub_->publish(lidar_path_msg_);
  }

  //TUM file writing (line to be added)
  if (tum_file_.is_open()) {
    tum_file_
      << std::fixed << std::setprecision(9) << stamp.seconds() << " "
      << std::setprecision(6)
      << p.x << " " << p.y << " " << p.z << " "
      << q.x << " " << q.y << " " << q.z << " " << q.w << "\n";
    tum_file_.flush();
  }

  computeAndPublishError(stamp);
  frame_count_++;
}

//fixposition/odometry_enu callback
void EvaluationNode::enuOdomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tryInitOrigins(*msg);
  if (!origins_set_) return;

  const Eigen::Vector3d pos_in_map = enuToMap(msg->pose.pose.position);
  latest_enu_pos_in_map_ = pos_in_map;

  //Append to ENU path
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp    = msg->header.stamp;
  ps.header.frame_id = "map";
  ps.pose.position.x = pos_in_map.x();
  ps.pose.position.y = pos_in_map.y();
  ps.pose.position.z = pos_in_map.z();
  ps.pose.orientation = msg->pose.pose.orientation;

  enu_path_msg_.header.stamp = msg->header.stamp;
  enu_path_msg_.poses.push_back(ps);
  enu_path_pub_->publish(enu_path_msg_);
}

/* ══════════════════════════════════════════════════════════════════════════
   Auto allignment strategy
   ++++++++++++++++++++++++
   1. Capture the first ENU pose (position + orientation in world ENU frame).
   2. Wait until the first LiDAR odometry pose is also available.
   3. Build T_enu_to_map_ such that:
        - ENU origin → LiDAR origin (translation alignment)
        - ENU heading → LiDAR heading (rotation alignment via yaw difference)
   4. If enu_yaw_offset_deg_ != 0, override the rotation with a manual value (and this is only applied according to settings).
   ══════════════════════════════════════════════════════════════════════════ */
void EvaluationNode::tryInitOrigins(const nav_msgs::msg::Odometry & enu_msg)
{
  if (origins_set_) return;
  if (!first_lidar_pos_.has_value()) return;  // wait for first LiDAR pose

  const auto & ep = enu_msg.pose.pose.position;
  const auto & eq = enu_msg.pose.pose.orientation;

  const Eigen::Vector3d    enu_pos(ep.x, ep.y, ep.z);
  const Eigen::Quaterniond enu_rot(eq.w, eq.x, eq.y, eq.z);

  //translation — map ENU origin to LiDAR map origin

  Eigen::Quaterniond R_enu_to_map;

  if (std::abs(enu_yaw_offset_deg_) > 1e-3) {
    // user-specified yaw offset
    const double yaw_rad = enu_yaw_offset_deg_ * DEG2RAD;
    R_enu_to_map = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
    RCLCPP_INFO(get_logger(),
      "Frame alignment: MANUAL yaw offset = %.1f deg", enu_yaw_offset_deg_);
  } else {
    //if no user-specified yaw offset align ENU heading to LiDAR heading
    const double enu_yaw = std::atan2(
      2.0 * (enu_rot.w() * enu_rot.z() + enu_rot.x() * enu_rot.y()),
      1.0 - 2.0 * (enu_rot.y() * enu_rot.y() + enu_rot.z() * enu_rot.z()));

    //Extract yaw from first LiDAR quaternion (heading in map frame)
    const Eigen::Quaterniond & lr = *first_lidar_rot_;
    const double lidar_yaw = std::atan2(
      2.0 * (lr.w() * lr.z() + lr.x() * lr.y()),
      1.0 - 2.0 * (lr.y() * lr.y() + lr.z() * lr.z()));

    //Rotation to apply to ENU vectors to align them with the LiDAR map frame
    const double delta_yaw = lidar_yaw - enu_yaw;
    R_enu_to_map = Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ());

    RCLCPP_INFO(get_logger(),
      "Frame alignment: AUTO\n"
      "  ENU yaw   = %.3f rad (%.1f deg)\n"
      "  LiDAR yaw = %.3f rad (%.1f deg)\n"
      "  delta yaw = %.3f rad (%.1f deg)",
      enu_yaw, enu_yaw / DEG2RAD,
      lidar_yaw, lidar_yaw / DEG2RAD,
      delta_yaw, delta_yaw / DEG2RAD);
  }

  // ── Build T_enu_to_map_ ──
  // Maps a point p_enu (in ENU world frame) to p_map (in LiDAR map frame):
  T_enu_to_map_.setIdentity();
  T_enu_to_map_.linear()      = R_enu_to_map.toRotationMatrix();
  T_enu_to_map_.translation() = -(R_enu_to_map * enu_pos);

  origins_set_ = true;

  RCLCPP_INFO(get_logger(),
    "Origins set. ENU→map translation offset: [%.3f, %.3f, %.3f]",
    T_enu_to_map_.translation().x(),
    T_enu_to_map_.translation().y(),
    T_enu_to_map_.translation().z());
}

//enuToMap — apply T_enu_to_map_ to an ENU world position
Eigen::Vector3d EvaluationNode::enuToMap(
  const geometry_msgs::msg::Point & p) const
{
  return T_enu_to_map_ * Eigen::Vector3d(p.x, p.y, p.z);
}

//computeAndPublishError
void EvaluationNode::computeAndPublishError(const rclcpp::Time & stamp)
{
  if (!latest_lidar_pos_.has_value() || !latest_enu_pos_in_map_.has_value()) return;

  const Eigen::Vector3d & lp = *latest_lidar_pos_;
  const Eigen::Vector3d & ep = *latest_enu_pos_in_map_;
  const double error_m = (lp - ep).norm();
  const double t       = stamp.seconds();

  //Publish scalar error
  std_msgs::msg::Float64 err_msg;
  err_msg.data = error_m;
  error_pub_->publish(err_msg);

  //Rolling RMSE 
  error_history_.push_back({t, error_m});
  while (!error_history_.empty() &&
         (t - error_history_.front().t) > error_window_sec_) {
    error_history_.pop_front();
  }
  double rmse = 0.0;
  if (!error_history_.empty()) {
    double sum_sq = 0.0;
    for (const auto & e : error_history_) sum_sq += e.error_m * e.error_m;
    rmse = std::sqrt(sum_sq / static_cast<double>(error_history_.size()));
  }

  //Writing error to csv
  if (error_csv_.is_open()) {
    error_csv_
      << frame_count_ << ","
      << std::fixed << std::setprecision(9) << t << ","
      << std::setprecision(6)
      << lp.x() << "," << lp.y() << "," << lp.z() << ","
      << ep.x() << "," << ep.y() << "," << ep.z() << ","
      << error_m << "," << rmse << "\n";
    error_csv_.flush();
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "[Frame %4d]  LiDAR [%.2f %.2f %.2f]  ENU [%.2f %.2f %.2f]  "
    "err=%.3f m  RMSE(%.0fs)=%.3f m",
    frame_count_,
    lp.x(), lp.y(), lp.z(),
    ep.x(), ep.y(), ep.z(),
    error_m, error_window_sec_, rmse);
}

//save_map service
void EvaluationNode::saveTrigger(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  res->success = saveMapToDisk();
  res->message = res->success ?
    "Map saved to " + output_pcd_path_ : "Map empty or not received.";
}


//saveMapToDisk (Saving map locally)
bool EvaluationNode::saveMapToDisk()
{
  if (!map_received_ || latest_map_->empty()) {
    RCLCPP_WARN(get_logger(), "No map data to save.");
    return false;
  }
  if (pcl::io::savePCDFileBinary(output_pcd_path_, *latest_map_) == 0) {
    RCLCPP_INFO(get_logger(), "PCD saved to '%s'  (%zu pts)",
      output_pcd_path_.c_str(), latest_map_->size());
    return true;
  }
  RCLCPP_ERROR(get_logger(), "Failed to save PCD to '%s'", output_pcd_path_.c_str());
  return false;
}


//Eval node shutdown

void EvaluationNode::shutdown()
{
  RCLCPP_INFO(get_logger(), "Shutdown — saving outputs...");
  saveMapToDisk();
  if (tum_file_.is_open())  { tum_file_.close();
    RCLCPP_INFO(get_logger(), "TUM closed: '%s'", output_tum_path_.c_str()); }
  if (error_csv_.is_open()) { error_csv_.close();
    RCLCPP_INFO(get_logger(), "CSV closed: '%s'", output_error_csv_path_.c_str()); }
}

}  


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_pose_estimator::EvaluationNode>();
  rclcpp::spin(node);
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
