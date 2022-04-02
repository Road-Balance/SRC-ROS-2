
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <src_control_message/msg/src_msg.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

using Twist = geometry_msgs::msg::Twist;
using SRCMsg = src_control_message::msg::SRCMsg;
using Float32 = std_msgs::msg::Float32;
using Imu = sensor_msgs::msg::Imu;

class CmdToSRC : public rclcpp::Node {
private:
  rclcpp::Publisher<SRCMsg>::SharedPtr src_msg_pub_;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<Float32>::SharedPtr accel_vel_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  rclcpp::Time prev_time;

  // TODO: src_msg params & yaml file (for accel, daccel, etc...)
  SRCMsg src_msg_;
  uint steering_offset_ = 20;

  float accel_;
  float deaccel;
  unsigned int scale;

public:
  CmdToSRC() : Node("cmd_vel_to_src_msg") {
    RCLCPP_INFO(get_logger(), "Cmd_Vel to SRC_Msg Node Created");

    src_msg_pub_ = create_publisher<SRCMsg>("src_control", 10);

    imu_sub_ = create_subscription<Imu>(
      "imu/data", 10,
      std::bind(&CmdToSRC::imu_cb, this, std::placeholders::_1)
    );

    cmd_vel_sub_ = create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&CmdToSRC::cmd_vel_cb, this, std::placeholders::_1)
    );
    
    accel_vel_sub_ = create_subscription<Float32>(
      "accel_vel", 10,
      std::bind(&CmdToSRC::accel_vel_cb, this, std::placeholders::_1)
    );

    pub_timer_ = this->create_wall_timer(
      20ms, std::bind(&CmdToSRC::timer_callback, this)
    );

    // paramter
    this->declare_parameter("accel_scale", 5.0);
    accel_ = this->get_parameter("accel_scale").as_double();

    this->declare_parameter("deaccel_scale", 5.0);
    deaccel = this->get_parameter("deaccel_scale").as_double();

    this->declare_parameter("scale", 9);
    scale = this->get_parameter("scale").as_int();

    src_msg_.speed = 0.0;
    src_msg_.steering = 0;
    src_msg_.light = false;
    src_msg_.direction = true;
    src_msg_.lcd_msg = "";
    src_msg_.accel = accel_;
    src_msg_.deaccel = deaccel;
    src_msg_.scale = scale;

    prev_time = this->get_clock()->now();
  }

  void imu_cb(const Imu::SharedPtr msg){

    auto current_time = this->get_clock()->now();
    const double  dt = (current_time - prev_time).seconds();
    RCLCPP_INFO(get_logger(), "dt : %f", dt); // dt 0.02 ok
    
    prev_time = current_time;

    tf2::Quaternion q_;

    q_[0] = msg->orientation.x;
    q_[1] = msg->orientation.y;
    q_[2] = msg->orientation.z;
    q_[3] = msg->orientation.w;

    tf2::Matrix3x3 m(q_);
    
    double roll, pitch;
    double heading_;

    m.getRPY(roll, pitch, heading_);

    RCLCPP_INFO(get_logger(), "yaw : %f", heading_);
  }

  void cmd_vel_cb(const Twist::SharedPtr msg) {
    src_msg_.speed = msg->linear.x;

    // TODO: Find Zero point
    src_msg_.steering = -((msg->angular.z)/2 * steering_offset_) + steering_offset_;
    // TODO: Light On Off mode
    
    src_msg_.direction = src_msg_.speed >= 0 ? 1:0;
  }

  void accel_vel_cb(const Float32::SharedPtr msg){
    src_msg_.accel = msg->data;
    src_msg_.deaccel = msg->data;
  }

  void timer_callback() {
    src_msg_pub_->publish(src_msg_);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto cmd_to_src = std::make_shared<CmdToSRC>();

  rclcpp::spin(cmd_to_src);
  rclcpp::shutdown();

  return 0;
}