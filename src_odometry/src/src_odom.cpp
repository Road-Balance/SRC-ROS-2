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
  double front_hinge_pos;

  int64_t encoder_pos;

  rclcpp::Time timestamp_;

  Imu imu_msg_;
  Int64 encoder_msg_;

  Odometry odom_msg_;
  std::unique_ptr<TransformBroadcaster> broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Int64>::SharedPtr encoder_sub_;

public:
  SRCOdom(): Node("ackermann_odometry") {
    // imu topic name
    // odom topic name
    // publish_tf
    // base_frame_id
    // odom_frame_id
    // update_rate

    odom_pub_ = this->create_publisher<Odometry>("odom", rclcpp::QoS(1));
    
    imu_sub_ = this->create_subscription<Imu>("imu/data", 10,
        std::bind(&SRCOdom::imu_sub_cb, this, std::placeholders::_1));

    encoder_sub_ = this->create_subscription<Int64>("encoder_value", rclcpp::SensorDataQoS(),
        std::bind(&SRCOdom::encoder_value_cb, this, std::placeholders::_1));

    // auto interval = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SRCOdom::timer_cb, this));
    timestamp_ = this->get_clock()->now();
  }

  void imu_sub_cb(const Imu::SharedPtr msg){
    std::cout << "imu_sub_cb" << std::endl;

    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
    );

    tf2::Matrix3x3 m(q);
    
    double roll, pitch;
    m.getRPY(roll, pitch, front_hinge_pos);

    std::cout << "yaw : " << front_hinge_pos << std::endl;
  }

  void encoder_value_cb(const Int64::SharedPtr msg){
    std::cout << "encoder_value_cb" << std::endl;
    encoder_pos = msg->data;
  }

  void timer_cb(){

    rclcpp::Time cur_time = this->get_clock()->now();

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (cur_time - timestamp_).seconds();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    // wheel_difference = cur_pose - prev_pose
    auto v = wheel_difference / dt;

    x_       += v * cos(encoder_pos);
    y_       += v * sin(encoder_pos);
    heading_ += angular;

    timestamp_ = cur_time;
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