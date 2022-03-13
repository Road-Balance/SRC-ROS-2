#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "src_odometry/odometry.hpp"

using Odometry = nav_msgs::msg::Odometry;
using JointState = sensor_msgs::msg::JointState;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class AckermannOdometry : public rclcpp::Node { 

private:
    ackermann_steering_controller::Odometry odometry_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr float_arr_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// Odometry related:
    // rclcpp::Duration publish_period_;
    rclcpp::Time last_state_publish_time_;
    bool open_loop_;
    
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

public:
    AckermannOdometry(): Node("ackermann_odometry"){
        // Setup Parameters
        this->declare_parameter("publish_rate", 50);
        auto publish_rate = this->get_parameter("publish_rate").as_int();
        auto interval = std::chrono::duration<double>(1.0 / publish_rate);
        pub_timer_ = this->create_wall_timer(interval, std::bind(&AckermannOdometry::timer_cb, this));

        this->declare_parameter("open_loop", false);
        open_loop_ = this->get_parameter("open_loop").as_bool();

        this->declare_parameter("wheel_separation_h_multiplier", 1.0);
        wheel_separation_h_multiplier_ = this->get_parameter("wheel_separation_h_multiplier").as_double();

        this->declare_parameter("wheel_radius_multiplier", 0.0);
        wheel_radius_multiplier_ = this->get_parameter("wheel_radius_multiplier").as_double();
        
        this->declare_parameter("steer_pos_multiplier", 1.0);
        steer_pos_multiplier_ = this->get_parameter("steer_pos_multiplier").as_double();

        this->declare_parameter("velocity_rolling_window_size", 10);
        velocity_rolling_window_size_ = this->get_parameter("velocity_rolling_window_size").as_int();

        odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);

        this->declare_parameter("base_frame_id", "base_link");
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();

        this->declare_parameter("odom_frame_id", "odom");
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();

        this->declare_parameter("enable_odom_tf", true);
        enable_odom_tf_ = this->get_parameter("enable_odom_tf").as_bool();

        const double ws_h = wheel_separation_h_multiplier_ * wheel_separation_h_;
        const double wr = wheel_radius_multiplier_ * wheel_radius_;
        odometry_.setWheelParams(ws_h, wr);

        RCLCPP_INFO(this->get_logger(), "Ackermann Odometry Node created");

        odom_pub_ = this->create_publisher<Odometry>("odom", 10);
     
        joint_state_sub_ = this->create_subscription<JointState>("joint_states", 10,
            std::bind(&AckermannOdometry::joint_state_cb, this, std::placeholders::_1));

        float_arr_sub_ = this->create_subscription<Float64MultiArray>("steering_angle_middle", 10,
            std::bind(&AckermannOdometry::float_arr_cb, this, std::placeholders::_1));
    }

    void joint_state_cb(const JointState::SharedPtr msg){
        std::cout << msg->name[0].c_str() << std::endl;
    }

    void float_arr_cb(const Float64MultiArray::SharedPtr msg){
        std::cout << msg->data[0] << std::endl;
    }

    void timer_cb(){

    }
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannOdometry>());
    rclcpp::shutdown();

    return 0;
}
