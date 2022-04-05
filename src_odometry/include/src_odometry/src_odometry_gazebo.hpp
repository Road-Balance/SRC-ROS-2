#pragma once


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include "src_odometry/odometry.hpp"

using String = std_msgs::msg::String;
using Float64 = std_msgs::msg::Float64;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using JointState = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;

class SRCOdometry: public rclcpp::Node {
public:
    SRCOdometry();

    void update(const rclcpp::Time& time, const rclcpp::Duration& period);
    void starting();
    void stopping(const rclcpp::Time& /*time*/);

    void jointstateCallback(const JointState::SharedPtr msg);
    void steeringAngleSubCallback(const Float64::SharedPtr msg);
    void cmdvelSubCallback(const Twist::SharedPtr msg);
    void imuSubCallback(const Imu::SharedPtr msg);

    void odom_update(const rclcpp::Time &time);
    void publish_odom_topic(const rclcpp::Time &time);
    void timer_cb();

private:
    ackermann_steering_controller::Odometry odometry_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<Float64>::SharedPtr imu_heading_pub_;

    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<Float64>::SharedPtr steering_angle_sub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool verbose_;

    /// Odometry related:
    // rclcpp::Duration publish_period_;
    rclcpp::Time last_state_publish_time_;
    bool open_loop_;
    bool has_imu_heading_;
    
    /// Velocity command related:
    struct Commands
    {
      double lin;
      double ang;
      rclcpp::Time stamp;

      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    Commands command_struct_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_h_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_h_multiplier_;
    double wheel_radius_multiplier_;
    double steer_pos_multiplier_;

    // open loop variables
    double linear_x, angular_z;
    
    // Mean of two rear wheel pose (radian)
    double rear_wheel_pos;
    double left_rear_wheel_joint, right_rear_wheel_joint;

    // 
    double heading_angle;

    // Front wheel steering pose (radian)
    float front_hinge_pos;

    int velocity_rolling_window_size_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Whether to allow multiple publishers on cmd_vel topic or not:
    bool allow_multiple_cmd_vel_publishers_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Number of steer joints:
    size_t steer_joints_size_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    
};