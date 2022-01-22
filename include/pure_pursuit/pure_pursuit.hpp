#ifndef _PURE_PURSUIT_H_
#define _PURE_PURSUIT_H_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

// TODO  シミュレーションのパターンを2通り考える
// 1. ステアリングを切ったあとの位置を出力する場合(tfのみを出力)
// 2. 速度を出力する場合(gazebo及び実機で検証))
// 3. 停止制御の実装
// 4. 旋回方向の速度の計算
class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit();
  ~PurePursuit() {}

private:
  void timerCallback();
  void setPath(const nav_msgs::msg::Path& msg);
  void publishMarker(geometry_msgs::msg::Pose pose) const;
  void publishTF(geometry_msgs::msg::Pose pose) const;
  void publishVelocity(double v, double w) const;
  double calcPurePursuit(geometry_msgs::msg::Pose pose, double & look_ahead);
  std::size_t planTargetPoint(geometry_msgs::msg::Pose pose, double & look_ahead);
  geometry_msgs::msg::Pose steeringControl(
    geometry_msgs::msg::Pose pose, double vel, double angle, double look_ahead);
  inline double pidVelocityControl(double target_vel, double current_vel)
  {
    return 1 * (target_vel - current_vel);
  }
  double quaternionToYaw(geometry_msgs::msg::Quaternion quat)
  {
    double roll, pitch, yaw;
    tf2::Quaternion tf_quat;
    tf_quat.setW(quat.w);
    tf_quat.setX(quat.x);
    tf_quat.setY(quat.y);
    tf_quat.setZ(quat.z);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Path ref_path_;

  std::vector<double> ref_x_, ref_y_;
  double current_vel_{0.0};
  double target_vel_{0.0};
  bool init_path_{false};
  double period_;
};

#endif
