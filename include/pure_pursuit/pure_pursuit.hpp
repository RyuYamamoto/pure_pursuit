#ifndef _PURE_PURSUIT_H_
#define _PURE_PURSUIT_H_

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

// TODO  シミュレーションのパターンを2通り考える
// 1. ステアリングを切ったあとの位置を出力する場合(tfのみを出力)
// 2. 速度を出力する場合(gazebo及び実機で検証))
class PurePursuit
{
public:
    PurePursuit(ros::NodeHandle nh);
    ~PurePursuit(){}
    void run();
private:
    void _set_path(const nav_msgs::PathConstPtr &msg);
    void _publish_marker(geometry_msgs::Pose pose) const;
    void _publish_tf(geometry_msgs::Pose pose) const;
    double _calc_pure_pursuit(geometry_msgs::Pose pose, double &look_ahead);
    std::size_t _plan_target_point(geometry_msgs::Pose pose, double &look_ahead);
    geometry_msgs::Pose _steering_control(geometry_msgs::Pose pose, double vel, double angle, double look_ahead);
    inline double _pid_vel_control(double target_vel, double current_vel)
    {
        return 1*(target_vel-current_vel);  
    }
    double _geometry_quat_to_rpy(geometry_msgs::Quaternion quat)
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
    
    ros::NodeHandle _nh;
    ros::Subscriber _sub_path;
    ros::Publisher _pub_vel;
    ros::Publisher _pub_marker;

    nav_msgs::Path _ref_path;

    std::vector<double> _ref_x, _ref_y;
    double _current_vel;
    bool _init_path;

    std::string _output_vel;
    std::string _input_path;
};

#endif