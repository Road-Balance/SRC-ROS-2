#include <memory>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using UInt8 = std_msgs::msg::UInt8;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;

inline bool isTrue(const int &val_in)
{
  return (val_in == 1) ? true : false;
}

class JoyToCmd : public rclcpp::Node
{
public:
  struct XMode
  {
    float left_updown;
    float left_leftright;

    float right_updown;
    float right_leftright;

    bool btn_a;
    bool btn_b;
    bool btn_x;
    bool btn_y;

    bool btn_LB;
    bool btn_RB;

    bool btn_back;
    bool btn_start;
  };

private:
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<UInt8>::SharedPtr src_mode_pub;
  rclcpp::Subscription<Joy>::SharedPtr joy_sub;

  Twist twist;
  UInt8 src_mode;

  XMode joy_keys;

public:
  JoyToCmd() : Node("joy_to_cmd_vel_node")
  {

    cmd_vel_pub = create_publisher<Twist>("cmd_vel", 5);
    src_mode_pub = create_publisher<UInt8>("src_mode", 5);

    joy_sub = create_subscription<Joy>(
      "joy", 10,
      std::bind(&JoyToCmd::sub_callback, this, std::placeholders::_1)
    );

    src_mode.data = 1;
  }

  void sub_callback(const Joy::SharedPtr data)
  {
    // Assume JoyStick is on "X" mode
    joy_keys.left_updown = data->axes[1];
    joy_keys.left_leftright = data->axes[0];
    joy_keys.right_updown = data->axes[4];
    joy_keys.right_leftright = data->axes[3];

    joy_keys.btn_a = isTrue(data->buttons[0]);
    joy_keys.btn_b = isTrue(data->buttons[1]);
    joy_keys.btn_x = isTrue(data->buttons[2]);
    joy_keys.btn_y = isTrue(data->buttons[3]);

    joy_keys.btn_LB = isTrue(data->buttons[4]);
    joy_keys.btn_RB = isTrue(data->buttons[5]);

    joy_keys.btn_back = isTrue(data->buttons[6]);
    joy_keys.btn_start = isTrue(data->buttons[7]);

    calc_cmd_vel();
    pub_mode();
  }

  void calc_cmd_vel()
  {
    twist.linear.x = joy_keys.left_updown;
    twist.linear.y = joy_keys.left_leftright;
    twist.linear.z = 0.0;

    // TODO: Orientation
    twist.angular.z = joy_keys.right_leftright * 2;

    cmd_vel_pub->publish(twist);
  }
  
  void pub_mode(){
    if (joy_keys.btn_x)
      src_mode.data = 1;
    if (joy_keys.btn_a)
      src_mode.data = 2;
    if (joy_keys.btn_b)
      src_mode.data = 3;
    if (joy_keys.btn_y)
      src_mode.data = 4;

    src_mode_pub->publish(src_mode);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto joy_to_cmd = std::make_shared<JoyToCmd>();

  rclcpp::spin(joy_to_cmd);
  rclcpp::shutdown();

  return 0;
}