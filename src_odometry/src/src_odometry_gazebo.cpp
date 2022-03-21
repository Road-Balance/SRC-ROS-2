#include <memory>
#include <boost/range/combine.hpp>

#include "src_odometry/src_odometry_gazebo.hpp"
#include "src_odometry/odometry.hpp"

SRCOdometry::SRCOdometry(): Node("ackermann_odometry"){
    // Setup Parameters
    this->declare_parameter("verbose", false);
    verbose_ = this->get_parameter("verbose").as_bool();

    this->declare_parameter("publish_rate", 50);
    auto publish_rate = this->get_parameter("publish_rate").as_int();
    auto interval = std::chrono::duration<double>(1.0 / publish_rate);
    pub_timer_ = this->create_wall_timer(interval, std::bind(&SRCOdometry::timer_cb, this));

    this->declare_parameter("open_loop", false);
    open_loop_ = this->get_parameter("open_loop").as_bool();

    this->declare_parameter("wheel_separation_h_", 0.325);
    wheel_separation_h_ = this->get_parameter("wheel_separation_h_").as_double();

    this->declare_parameter("wheel_separation_h_multiplier", 1.1);
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
        std::bind(&SRCOdometry::joint_state_cb, this, std::placeholders::_1));

    steering_angle_sub_ = this->create_subscription<Float64>("steering_angle_middle", 10,
        std::bind(&SRCOdometry::steering_angle_sub, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<Twist>("cmd_vel", 10,
        std::bind(&SRCOdometry::cmd_vel_sub, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    this->starting();
}


void SRCOdometry::starting(){

    rclcpp::Time time = this->get_clock()->now();
    
    last_state_publish_time_ = time;

    odometry_.init(time);
}

void SRCOdometry::joint_state_cb(const JointState::SharedPtr msg){

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

void SRCOdometry::steering_angle_sub(const Float64::SharedPtr msg){

    front_hinge_pos = msg->data;

    std::cout << "front_hinge_pos : " << front_hinge_pos << std::endl;

    if (verbose_)
        RCLCPP_INFO(this->get_logger(), "Front Hinge Pose : %f", front_hinge_pos);
}

void SRCOdometry::cmd_vel_sub(const Twist::SharedPtr msg){
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;
}

void SRCOdometry::odom_update(const rclcpp::Time &time){
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

void SRCOdometry::publish_odom_topic(const rclcpp::Time &time){
    // Publish odometry message
    // Compute and store orientation info
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odometry_.getHeading());

    // Populate odom message and publish
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();

    odom->header.frame_id = odom_frame_id_;
    odom->child_frame_id = base_frame_id_;
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

void SRCOdometry::timer_cb(){
    rclcpp::Time cur_time = this->get_clock()->now();

    odom_update(cur_time);
    publish_odom_topic(cur_time);
}

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SRCOdometry>());
    rclcpp::shutdown();

    return 0;
}
