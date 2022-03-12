#pragma once


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/msg/odometry.hpp"
#include "src_odometry/odometry.hpp"
#include "geometry_msg/msg/Twist.hpp"

using Twist = geometry_msg::msg::Twist;

class SRCOdometry {
public:
    SRCOdometry();

    void update(const rclcpp::Time& time, const rclcpp::Duration& period);
    void starting(const rclcpp::Time& time);
    void stopping(const rclcpp::Time& /*time*/);

private:
    std::string name_;

    /// Odometry related:
    rclcpp::Duration publish_period_;
    rclcpp::Time last_state_publish_time_;
    bool open_loop_;

    struct Commands
    {
      double lin;
      double ang;
      ros::Time stamp;

      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };

    rclcpp::Subscription<Twist>::SharedPtr twist_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    rclcpp::Publisher<>
    Odometry odometry_;

    
};