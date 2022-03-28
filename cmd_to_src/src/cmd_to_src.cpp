
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "src_control_message/msg/src_msg.hpp"

using namespace std::chrono_literals;

using Twist = geometry_msgs::msg::Twist;
using SRCMsg = src_control_message::msg::SRCMsg;
using Float32 = std_msgs::msg::Float32;

class CmdToSRC : public rclcpp::Node {
private:
  rclcpp::Publisher<SRCMsg>::SharedPtr src_pub;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<Float32>::SharedPtr accel_vel_sub;
  rclcpp::TimerBase::SharedPtr pub_timer;

  // TODO: src_msg params & yaml file (for accel, daccel, etc...)
  SRCMsg src_msg;
  uint steering_offset = 20;

  float accel;
  float deaccel;
  unsigned int scale;

public:
  CmdToSRC() : Node("cmd_vel_to_src_msg") {
    RCLCPP_INFO(get_logger(), "Cmd_Vel to SRC_Msg Node Created");

    src_pub = create_publisher<SRCMsg>("src_control", 10);

    cmd_vel_sub = create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&CmdToSRC::cmd_vel_cb, this, std::placeholders::_1)
    );
    
    accel_vel_sub = create_subscription<Float32>(
      "accel_vel", 10,
      std::bind(&CmdToSRC::accel_vel_cb, this, std::placeholders::_1)
    );

    pub_timer = this->create_wall_timer(
      20ms, std::bind(&CmdToSRC::timer_callback, this)
    );

    // paramter
    this->declare_parameter("accel_scale", 5.0);
    accel = this->get_parameter("accel_scale").as_double();

    this->declare_parameter("deaccel_scale", 5.0);
    deaccel = this->get_parameter("deaccel_scale").as_double();

    this->declare_parameter("scale", 9);
    scale = this->get_parameter("scale").as_int();

    src_msg.speed = 0.0;
    src_msg.steering = 0;
    src_msg.light = false;
    src_msg.direction = true;
    src_msg.lcd_msg = "";
    src_msg.accel = accel;
    src_msg.deaccel = deaccel;
    src_msg.scale = scale;
  }

  void cmd_vel_cb(const Twist::SharedPtr msg) {
    src_msg.speed = msg->linear.x;

    // TODO: Find Zero point
    src_msg.steering = -((msg->angular.z)/2 * steering_offset) + steering_offset;
    // TODO: Light On Off mode
    
    src_msg.direction = src_msg.speed >= 0 ? 1:0;
  }

  void accel_vel_cb(const Float32::SharedPtr msg){
    src_msg.accel = msg->data;
    src_msg.deaccel = msg->data;
  }

  void timer_callback() {
    src_pub->publish(src_msg);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto cmd_to_src = std::make_shared<CmdToSRC>();

  rclcpp::spin(cmd_to_src);
  rclcpp::shutdown();

  return 0;
}