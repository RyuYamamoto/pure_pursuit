#include "pure_pursuit/pure_pursuit.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit")
{
  period_ = this->declare_parameter("period", 0.01);
  target_vel_ = this->declare_parameter("target_vel", 3.0);

  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", rclcpp::QoS{1}.transient_local(),
    std::bind(&PurePursuit::setPath, this, std::placeholders::_1));

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 * period_)),
    std::bind(&PurePursuit::timerCallback, this));
}

void PurePursuit::setPath(const nav_msgs::msg::Path & msg)
{
  if (!init_path_) {
    init_path_ = true;
    ref_path_.header.frame_id = msg.header.frame_id;
    ref_path_.header.stamp = msg.header.stamp;
    ref_path_.poses = msg.poses;
  }

  for (auto pose : ref_path_.poses) {
    ref_x_.emplace_back(pose.pose.position.x);
    ref_y_.emplace_back(pose.pose.position.y);
  }
}

void PurePursuit::timerCallback()
{
  if (!init_path_) {
    RCLCPP_ERROR(get_logger(), "Not Found Path.");
    return;
  }
  double look_ahead;
  const double ak = pidVelocityControl(target_vel_, current_vel_);
  const double steering_angle = calcPurePursuit(current_pose_, look_ahead);
  current_pose_ = steeringControl(current_pose_, ak, steering_angle, look_ahead);
  publishMarker(current_pose_);
  publishTF(current_pose_);
  publishVelocity(current_vel_, 0.0);
}

// ロボットの姿勢を出力
void PurePursuit::publishMarker(geometry_msgs::msg::Pose pose) const
{
  visualization_msgs::msg::Marker marker;

  marker.id = 0;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();

  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.ns = "robot";
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker_publisher_->publish(marker);
}

void PurePursuit::publishVelocity(double v, double w) const
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = v;
  vel.angular.z = w;
  velocity_publisher_->publish(vel);
}

// tfを出力
void PurePursuit::publishTF(geometry_msgs::msg::Pose pose) const
{
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = rclcpp::Clock().now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation.x = pose.orientation.x;
  transformStamped.transform.rotation.y = pose.orientation.y;
  transformStamped.transform.rotation.z = pose.orientation.z;
  transformStamped.transform.rotation.w = pose.orientation.w;

  broadcaster_->sendTransform(transformStamped);
}

// pure pursuitによるステアリング角の決定
double PurePursuit::calcPurePursuit(geometry_msgs::msg::Pose pose, double & look_ahead)
{
  // double look_ahead;
  std::size_t index = planTargetPoint(pose, look_ahead);
  const double yaw = quaternionToYaw(pose.orientation);
  const double alpha =
    std::atan2(ref_y_[index] - pose.position.y, ref_x_[index] - pose.position.x) - yaw;
  const double steering_angle =
    std::atan2(2.0 * look_ahead * std::sin(alpha) / (0.3 + 0.1 * current_vel_), 1.0);
  return steering_angle;
}

// look ahead distanceの更新とターゲット位置を取得
std::size_t PurePursuit::planTargetPoint(geometry_msgs::msg::Pose pose, double & look_ahead)
{
  std::vector<double> dx, dy;

  for (std::size_t index = 0; index < ref_x_.size(); index++) {
    dx.push_back(pose.position.x - ref_x_[index]);
    dy.push_back(pose.position.y - ref_y_[index]);
  }
  std::vector<double> distance;
  for (std::size_t index = 0; index < dx.size(); index++) {
    const double dist = std::abs(std::sqrt(dx[index] * dx[index] + dy[index] * dy[index]));
    distance.push_back(dist);
  }
  std::vector<double>::iterator iter = std::min_element(distance.begin(), distance.end());
  std::size_t index = std::distance(distance.begin(), iter);
  const double look_ahead_filter = 0.1 * current_vel_ + 0.3;
  look_ahead = 0.0;
  // look ahead distanceの更新
  while (look_ahead_filter > look_ahead && (index + 1) < ref_x_.size()) {
    const double d_x = ref_x_[index + 1] - ref_x_[index];
    const double d_y = ref_y_[index + 1] - ref_y_[index];
    look_ahead += std::sqrt(d_x * d_x + d_y * d_y);
    index += 1;
  }
  return index;
}

// 現在位置と速度、ステアンリング各からt+1後のロボットの位置を計算する
geometry_msgs::msg::Pose PurePursuit::steeringControl(
  geometry_msgs::msg::Pose pose, double vel, double angle, double look_ahead)
{
  geometry_msgs::msg::Pose update_pose = pose;
  // 現在姿勢からyaw軸の姿勢を取得
  double yaw = quaternionToYaw(pose.orientation);
  update_pose.position.x = pose.position.x + current_vel_ * std::cos(yaw) * 0.01;
  update_pose.position.y = pose.position.y + current_vel_ * std::sin(yaw) * 0.01;
  yaw += (current_vel_ / look_ahead) * std::tan(angle) * (0.01);
  current_vel_ += vel * 0.01;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  update_pose.orientation.w = quat.w();
  update_pose.orientation.x = quat.x();
  update_pose.orientation.y = quat.y();
  update_pose.orientation.z = quat.z();
  return update_pose;
}
