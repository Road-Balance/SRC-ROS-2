#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int64.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

typedef sensor_msgs::msg::Imu Imu;
typedef std_msgs::msg::Int64 Int64;

typedef nav_msgs::msg::Odometry Odometry;
typedef tf2_ros::TransformBroadcaster TransformBroadcaster; 

class SRCOdom : public rclcpp::Node {
private: 

  /// Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  /// Current velocity:
  double linear_;  //   [m/s]
  double angular_; // [rad/s]

  /// Previous wheel position/state [rad]:
  double rear_wheel_old_pos_;

  int64_t encoder_pos;
  int64_t prev_encoder_pos;

  rclcpp::Time prev_time;

  tf2::Quaternion q_;

  Imu imu_msg_;
  Int64 encoder_msg_;

  Odometry odom_msg_;
  std::unique_ptr<TransformBroadcaster> broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Int64>::SharedPtr encoder_sub_;

  /// Parameters 
  
  // Subscribe topic names 
  std::string imu_topic_name_;
  std::string encoder_topic_name_;
  
  // TF parameters
  bool publish_tf_;
  bool verbose_;
  int update_rate_;
  std::string base_frame_id_;
  std::string odom_frame_id_;

  // Dynamics Parameters
  double wheel_radius_;
  uint encoder_resolution_;

public:
  SRCOdom(): Node("ackermann_odometry") {

    this->declare_parameter("imu_topic_name", "imu/data");
    rclcpp::Parameter imu_topic_name = this->get_parameter("imu_topic_name");
    imu_topic_name_ = imu_topic_name.as_string();

    this->declare_parameter("encoder_topic_name", "/encoder_value");
    rclcpp::Parameter encoder_topic_name = this->get_parameter("encoder_topic_name");
    encoder_topic_name_ = encoder_topic_name.as_string();

    this->declare_parameter("publish_tf", true);
    rclcpp::Parameter publish_tf = this->get_parameter("publish_tf");
    publish_tf_ = publish_tf.as_bool();

    this->declare_parameter("update_rate", 50);
    rclcpp::Parameter update_rate = this->get_parameter("update_rate");
    update_rate_ = update_rate.as_int();

    this->declare_parameter("base_frame_id", "base_link");
    rclcpp::Parameter base_frame_id = this->get_parameter("base_frame_id");
    base_frame_id_ = base_frame_id.as_string();

    this->declare_parameter("odom_frame_id", "odom");
    rclcpp::Parameter odom_frame_id = this->get_parameter("odom_frame_id");
    odom_frame_id_ = odom_frame_id.as_string();

    this->declare_parameter("wheel_radius", 0.0508);
    rclcpp::Parameter wheel_radius = this->get_parameter("wheel_radius");
    wheel_radius_ = wheel_radius.as_double();

    this->declare_parameter("encoder_resolution", 150);
    rclcpp::Parameter encoder_resolution = this->get_parameter("encoder_resolution");
    encoder_resolution_ = encoder_resolution.as_int();

    this->declare_parameter("verbose", false);
    rclcpp::Parameter verbose = this->get_parameter("verbose");
    verbose_ = verbose.as_bool();

    odom_pub_ = this->create_publisher<Odometry>("odom", rclcpp::QoS(1));
    
    imu_sub_ = this->create_subscription<Imu>("imu/data", 10,
        std::bind(&SRCOdom::imu_sub_cb, this, std::placeholders::_1));

    encoder_sub_ = this->create_subscription<Int64>("encoder_value", rclcpp::SensorDataQoS(),
        std::bind(&SRCOdom::encoder_value_cb, this, std::placeholders::_1));

    auto interval = (1000 / update_rate_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&SRCOdom::timer_cb, this));
    
    broadcaster_ = std::make_unique<TransformBroadcaster>(this);
    prev_time = this->get_clock()->now();

    q_[0] = 0.0;
    q_[1] = 0.0;
    q_[2] = 0.0;
    q_[3] = 1.0;
  }

  void imu_sub_cb(const Imu::SharedPtr msg){

    q_[0] = msg->orientation.x;
    q_[1] = msg->orientation.y;
    q_[2] = msg->orientation.z;
    q_[3] = msg->orientation.w;

    tf2::Matrix3x3 m(q_);
    
    double roll, pitch;
    m.getRPY(roll, pitch, heading_);

    if (verbose_)
      RCLCPP_INFO(get_logger(), "yaw : %f", heading_);
  }

  bool update_odom(int64_t encoder_pos, double heading, const rclcpp::Time &cur_time){

    auto rear_encoder_diff = (encoder_pos - prev_encoder_pos);
    auto rear_wheel_diff = rear_encoder_diff * (2 * M_PI * wheel_radius_ / encoder_resolution_);

    if(verbose_)
      RCLCPP_INFO(get_logger(), "rear_encoder_diff : %f / rear_wheel_diff : %f", 
        rear_encoder_diff, rear_wheel_diff);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (cur_time - prev_time).seconds();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    // wheel_difference = cur_pose - prev_pose
    auto linear_vel_ = rear_wheel_diff / dt;

    if(verbose_)
      RCLCPP_INFO(get_logger(), "dt : %f / linear_vel_ : %f", 
        dt, linear_vel_);

    x_       += rear_wheel_diff * cos(heading);
    y_       += rear_wheel_diff * sin(heading);
    heading_  = heading;

    prev_encoder_pos = encoder_pos;
    prev_time = cur_time;

    return true;
  }

  void encoder_value_cb(const Int64::SharedPtr msg){
    encoder_pos = msg->data;
  }

  void timer_cb(){
    auto time = this->get_clock()->now();
    update_odom(encoder_pos, this->heading_, time);

    odom_msg_.header.stamp = time;
    odom_msg_.header.frame_id = odom_frame_id_;
    odom_msg_.child_frame_id = base_frame_id_;
    
    odom_msg_.pose.pose.position.x = x_;  
    odom_msg_.pose.pose.position.y = y_;
    // odom_msg_.pose.pose.orientation = q_;
    odom_msg_.pose.pose.orientation.x = q_[0];
    odom_msg_.pose.pose.orientation.y = q_[1];
    odom_msg_.pose.pose.orientation.z = q_[2];
    odom_msg_.pose.pose.orientation.w = q_[3];
    odom_msg_.pose.covariance.fill(0.0);
    odom_msg_.pose.covariance[0] = 1e-3;
    odom_msg_.pose.covariance[7] = 1e-3;
    odom_msg_.pose.covariance[14] = 1e6;
    odom_msg_.pose.covariance[21] = 1e6; 
    odom_msg_.pose.covariance[28] = 1e6;
    odom_msg_.pose.covariance[35] = 1e-3;

    // odom_msg_.twist.twist.linear.x = odometry_.getLinear();  
    // odom_msg_.twist.twist.angular.z = odometry_.getAngular();      
    odom_msg_.twist.covariance.fill(0.0);
    odom_msg_.twist.covariance[0] = 1e-3;
    odom_msg_.twist.covariance[7] = 1e-3;
    odom_msg_.twist.covariance[14] = 1e6;
    odom_msg_.twist.covariance[21] = 1e6;
    odom_msg_.twist.covariance[28] = 1e6;
    odom_msg_.twist.covariance[35] = 1e3;
    
    if(publish_tf_){
      // publish TF
      geometry_msgs::msg::TransformStamped odom_tf;

      odom_tf.header.stamp = time;
      odom_tf.header.frame_id = odom_frame_id_;
      odom_tf.child_frame_id = base_frame_id_;

      odom_tf.transform.translation.x = odom_msg_.pose.pose.position.x;
      odom_tf.transform.translation.y = odom_msg_.pose.pose.position.y;
      odom_tf.transform.translation.z = 0;
      odom_tf.transform.rotation = odom_msg_.pose.pose.orientation;

      broadcaster_->sendTransform(odom_tf);
    }

    odom_pub_->publish(odom_msg_);
  }

  ~SRCOdom(){

  }
};

int main(int argc, char **argv){

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SRCOdom>());

  rclcpp::shutdown();

  return 0;
}