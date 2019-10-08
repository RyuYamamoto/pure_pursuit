#include <pure_pursuit/pure_pursuit.hpp>

PurePursuit::PurePursuit(ros::NodeHandle nh) 
    : _nh(nh),
      _current_vel(0.0),
      _init_path(false)
{
    ros::NodeHandle _pnh("~");

    _pnh.param<std::string>("output_vel", _output_vel, "/cmd_vel");
    _pnh.param<std::string>("input_path", _input_path, "/path");

    _pub_vel = _nh.advertise<geometry_msgs::Twist>(_output_vel, 10);
    _pub_marker = _nh.advertise<visualization_msgs::Marker>("/robot_marker", 10);
    _sub_path = _nh.subscribe(_input_path, 10, &PurePursuit::_set_path, this);
}

void PurePursuit::_set_path(const nav_msgs::PathConstPtr &msg)
{
    if (!_init_path)
    {
        _init_path = true;
        _ref_path.header.frame_id = msg->header.frame_id;
        _ref_path.header.stamp = msg->header.stamp;
        _ref_path.poses = msg->poses;
        run();
    }
}

void PurePursuit::run()
{
    {
        ros::Rate rate(10);
        // 目標速度
        double target_vel = 7.0;

        // スプライン補間されたパスを格納する
        for(std::size_t index=0; index<_ref_path.poses.size();index++)
        {
            ROS_INFO("%f %f", _ref_path.poses[index].pose.position.x, _ref_path.poses[index].pose.position.y);
            _ref_x.push_back(_ref_path.poses[index].pose.position.x);
            _ref_y.push_back(_ref_path.poses[index].pose.position.y);
        }
        
        geometry_msgs::Pose robot_pose;
        robot_pose.position.x = 0.0;
        robot_pose.position.y = 0.0;
        robot_pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        robot_pose.orientation.w = q.w();
        robot_pose.orientation.x = q.x();
        robot_pose.orientation.y = q.y();
        robot_pose.orientation.z = q.z();

        // メイン処理
        while(1)
        {
            double look_ahead;
            // P制御による速度決定
            double ak = _pid_vel_control(target_vel, _current_vel);
            ROS_INFO("current_vel: %f", _current_vel);
            double steering_angle = _calc_pure_pursuit(robot_pose, look_ahead);
            ROS_INFO("look ahead: %f", look_ahead);
            robot_pose = _steering_control(robot_pose, ak, steering_angle, look_ahead);
            _publish_marker(robot_pose);
            _publish_tf(robot_pose);
            //ros::spinOnce();
            rate.sleep();
        }
    }
}

// ロボットの姿勢を出力
void PurePursuit::_publish_marker(geometry_msgs::Pose pose) const
{
    visualization_msgs::Marker marker;

    marker.id = 0;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.scale.x = 1.0;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.ns = "robot";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    _pub_marker.publish(marker);
}

// tfを出力
void PurePursuit::_publish_tf(geometry_msgs::Pose pose) const 
{
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "robot";
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;
    transformStamped.transform.rotation.x = pose.orientation.x;
    transformStamped.transform.rotation.y = pose.orientation.y;
    transformStamped.transform.rotation.z = pose.orientation.z;
    transformStamped.transform.rotation.w = pose.orientation.w;

    br.sendTransform(transformStamped);
}

// pure pursuitによるステアリング角の決定
double PurePursuit::_calc_pure_pursuit(geometry_msgs::Pose pose, double &look_ahead)
{
    try
    {
        //double look_ahead;
        std::size_t index = _plan_target_point(pose, look_ahead);
        ROS_INFO("%d",index);
        double yaw = _geometry_quat_to_rpy(pose.orientation);
        double alpha = std::atan2(_ref_y[index]-pose.position.y, _ref_x[index]-pose.position.x)-yaw;
        double steering_angle = std::atan2(2.0*look_ahead*std::sin(alpha)/(3.0+0.1*_current_vel), 1.0);
        return steering_angle;
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }   
}

// look ahead distanceの更新とターゲット位置を取得
std::size_t PurePursuit::_plan_target_point(geometry_msgs::Pose pose, double &look_ahead)
{
    std::vector<double> dx, dy;

    for(std::size_t index=0;index<_ref_x.size(); index++)
    {
        dx.push_back(pose.position.x - _ref_x[index]);
        dy.push_back(pose.position.y - _ref_y[index]);
    }
    std::vector<double> distance;
    for(std::size_t index=0;index<dx.size(); index++)
    {
        double dist = std::abs(std::sqrt(dx[index]*dx[index] + dy[index]*dy[index]));
        distance.push_back(dist);
    }
    std::vector<double>::iterator iter = std::min_element(distance.begin(), distance.end());
    std::size_t index = std::distance(distance.begin(), iter);
    double look_ahead_filter = 0.1 * _current_vel + 0.3;
    look_ahead = 0.0;
    // look ahead distanceの更新
    while(look_ahead_filter > look_ahead && (index+1) < _ref_x.size())
    {
        double d_x = _ref_x[index+1] - _ref_x[index];
        double d_y = _ref_y[index+1] - _ref_y[index];
        look_ahead += std::sqrt(d_x*d_x + d_y*d_y);
        index += 1;
    }
    return index;
}

// 現在位置と速度、ステアンリング各からt+1後のロボットの位置を計算する
geometry_msgs::Pose PurePursuit::_steering_control(geometry_msgs::Pose pose, double vel, double angle, double look_ahead)
{
    geometry_msgs::Pose update_pose=pose;
    // 現在姿勢からyaw軸の姿勢を取得
    double yaw = _geometry_quat_to_rpy(pose.orientation);
    update_pose.position.x = pose.position.x + _current_vel*std::cos(yaw)*0.01;
    update_pose.position.y = pose.position.y + _current_vel*std::sin(yaw)*0.01;
    yaw += (_current_vel/look_ahead)*std::tan(angle)*0.01;
    _current_vel += vel*0.01;
    tf2::Quaternion quat;
    quat.setRPY(0.0,0.0,yaw);
    update_pose.orientation.w = quat.w();
    update_pose.orientation.x = quat.x();
    update_pose.orientation.y = quat.y();
    update_pose.orientation.z = quat.z();
    return update_pose;
}