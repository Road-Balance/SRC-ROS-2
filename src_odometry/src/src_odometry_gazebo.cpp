#include <memory>
#include <boost/range/combine.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include "src_odometry/odometry.hpp"

using String = std_msgs::msg::String;
using Float64 = std_msgs::msg::Float64;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using JointState = sensor_msgs::msg::JointState;

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

class AckermannOdometry : public rclcpp::Node { 

private:
    ackermann_steering_controller::Odometry odometry_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<Float64>::SharedPtr steering_angle_sub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool verbose_;

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

    // open loop variables
    double linear_x, angular_z;
    
    // Mean of two rear wheel pose (radian)
    double rear_wheel_pos;
    double left_rear_wheel_joint, right_rear_wheel_joint;

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

public:
    AckermannOdometry(): Node("ackermann_odometry"){
        // Setup Parameters
        this->declare_parameter("verbose", false);
        verbose_ = this->get_parameter("verbose").as_bool();

        this->declare_parameter("publish_rate", 50);
        auto publish_rate = this->get_parameter("publish_rate").as_int();
        auto interval = std::chrono::duration<double>(1.0 / publish_rate);
        pub_timer_ = this->create_wall_timer(interval, std::bind(&AckermannOdometry::timer_cb, this));

        this->declare_parameter("open_loop", false);
        open_loop_ = this->get_parameter("open_loop").as_bool();

        // this->declare_parameter("wheel_separation_h_", 0.245);
        this->declare_parameter("wheel_separation_h_", 0.325);
        wheel_separation_h_ = this->get_parameter("wheel_separation_h_").as_double();

        this->declare_parameter("wheel_separation_h_multiplier", 1.0);
        wheel_separation_h_multiplier_ = this->get_parameter("wheel_separation_h_multiplier").as_double();

        this->declare_parameter("wheel_radius_", 0.05);
        wheel_radius_ = this->get_parameter("wheel_radius_").as_double();
        
        this->declare_parameter("wheel_radius_multiplier", 1.0);
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

        odom_pub_ = this->create_publisher<Odometry>("odom", rclcpp::QoS(1));
     
        joint_state_sub_ = this->create_subscription<JointState>("joint_states", 10,
            std::bind(&AckermannOdometry::joint_state_cb, this, std::placeholders::_1));

        steering_angle_sub_ = this->create_subscription<Float64>("steering_angle_middle", 10,
            std::bind(&AckermannOdometry::steering_angle_sub, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<Twist>("cmd_vel", 10,
            std::bind(&AckermannOdometry::cmd_vel_sub, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        starting();
    }

    void starting(){

        rclcpp::Time time = this->get_clock()->now();
        
        last_state_publish_time_ = time;

        odometry_.init(time);
    }

    void joint_state_cb(const JointState::SharedPtr msg){

        for(auto i=0; i < 6; i++){
            if(msg->position[i] == 0)
                break;

            if(strcmp(msg->name[i].c_str(), "left_rear_wheel_joint") == 0)
                left_rear_wheel_joint =  msg->position[i];
            if(strcmp(msg->name[i].c_str(), "right_rear_wheel_joint") == 0)
                right_rear_wheel_joint =  msg->position[i];
        }

        rear_wheel_pos = (left_rear_wheel_joint + right_rear_wheel_joint) / 2;

        if (verbose_)
            RCLCPP_INFO(this->get_logger(), "Rear Wheel Pose : %lf", rear_wheel_pos);
    }

    void steering_angle_sub(const Float64::SharedPtr msg){

        front_hinge_pos = msg->data;

        if (verbose_)
            RCLCPP_INFO(this->get_logger(), "Front Hinge Pose : %f", front_hinge_pos);
    }

    void cmd_vel_sub(const Twist::SharedPtr msg){
        linear_x = msg->linear.x;
        angular_z = msg->angular.z;
    }

    void odom_update(const rclcpp::Time &time){
        // COMPUTE AND PUBLISH ODOMETRY
        // TODO : open_loop implement & comparison
        if (open_loop_){
            odometry_.updateOpenLoop(linear_x, angular_z, time);
        }
        else{
            if (std::isnan(rear_wheel_pos) || std::isnan(front_hinge_pos))
                return;

            // Estimate linear and angular velocity using joint information
            front_hinge_pos *= steer_pos_multiplier_;
            odometry_.update(rear_wheel_pos, front_hinge_pos, time);
        }
    }

    void publish_odom_topic(const rclcpp::Time &time){
        // Publish odometry message
        // Compute and store orientation info
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, odometry_.getHeading());
        // const geometry_msgs::Quaternion orientation(
        //         createQuaternionMsgFromYaw(odometry_.getHeading()));

        // Populate odom message and publish
        auto odom = std::make_unique<nav_msgs::msg::Odometry>();
        // auto now = get_clock()->now();

        odom->header.frame_id = odom_frame_id_;
        odom->child_frame_id = base_frame_id_;
        // odom->header.stamp = now;
        odom->header.stamp = time;
        odom->pose.pose.position.x = odometry_.getX();  
        odom->pose.pose.position.y = odometry_.getY(); 
        odom->pose.pose.orientation.x = q.x();
        odom->pose.pose.orientation.y = q.y();
        odom->pose.pose.orientation.z = q.z();
        odom->pose.pose.orientation.w = q.w();
        odom->pose.covariance.fill(0.0);
        odom->pose.covariance[0] = 1e-3;
        odom->pose.covariance[7] = 1e-3;
        odom->pose.covariance[14] = 1e6; 
        odom->pose.covariance[21] = 1e6; 
        odom->pose.covariance[28] = 1e6;
        odom->pose.covariance[35] = 1e-3;

        odom->twist.twist.linear.x = odometry_.getLinear();  
        odom->twist.twist.angular.z = odometry_.getAngular();      
        odom->twist.covariance.fill(0.0);
        odom->twist.covariance[0] = 1e-3;
        odom->twist.covariance[7] = 1e-3;
        odom->twist.covariance[14] = 1e6;
        odom->twist.covariance[21] = 1e6;
        odom->twist.covariance[28] = 1e6;
        odom->twist.covariance[35] = 1e3;

        if(enable_odom_tf_){
            // publish TF
            geometry_msgs::msg::TransformStamped odom_tf;
            // odom_tf.header.stamp = now; 
            odom_tf.header.stamp = time;
            odom_tf.header.frame_id = odom_frame_id_;
            odom_tf.child_frame_id = base_frame_id_;
            odom_tf.transform.translation.x = odometry_.getX();
            odom_tf.transform.translation.y = odometry_.getY();
            odom_tf.transform.translation.z = 0;
            odom_tf.transform.rotation = odom->pose.pose.orientation;
            tf_broadcaster_->sendTransform(odom_tf);
        }

        odom_pub_->publish(std::move(odom));
    }

    void publish_odom_tf(){

    }

    void timer_cb(){
        rclcpp::Time cur_time = this->get_clock()->now();

        odom_update(cur_time);
        publish_odom_topic(cur_time);
    }
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannOdometry>());
    rclcpp::shutdown();

    return 0;
}
