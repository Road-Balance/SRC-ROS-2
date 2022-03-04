
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "src_control_message/msg/src_msg.hpp"

using namespace std::chrono_literals;

using Twist = geometry_msgs::msg::Twist;
using SRCMsg = src_control_message::msg::SRCMsg;

class CmdToSRC : public rclcpp::Node {
private:
  rclcpp::Publisher<SRCMsg>::SharedPtr src_pub;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;
  rclcpp::TimerBase::SharedPtr pub_timer;

  // TODO: src_msg params & yaml file (for accel, daccel, etc...)
  SRCMsg src_msg;

public:
  CmdToSRC() : Node("cmd_vel_to_src_msg") {
    RCLCPP_INFO(get_logger(), "Cmd_Vel to SRC_Msg Node Created");

    src_pub = create_publisher<SRCMsg>("src_control", 10);
    cmd_vel_sub = create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&CmdToSRC::sub_callback, this, std::placeholders::_1)
    );
    pub_timer = this->create_wall_timer(
      50ms, std::bind(&CmdToSRC::timer_callback, this)
    );

    src_msg.speed = 0.0;
    src_msg.steering = 0;
    src_msg.light = false;
    src_msg.direction = true;
    src_msg.lcd_msg = "";
    src_msg.accel = 0.4;
    src_msg.deaccel = 0.4;
    src_msg.scale = 80;
  }

  void sub_callback(const Twist::SharedPtr msg) {
    src_msg.speed = msg->linear.x;
    src_msg.steering = msg->angular.z;
    // TODO: Check direction
    // TODO: Light On Off mode
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