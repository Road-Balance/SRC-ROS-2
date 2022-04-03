
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <src_control_message/msg/src_msg.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

using Twist = geometry_msgs::msg::Twist;
using SRCMsg = src_control_message::msg::SRCMsg;
using Float32 = std_msgs::msg::Float32;
using Imu = sensor_msgs::msg::Imu;

class YawRatePID {
public:
  enum TwiddleCase {
    CASE_P_1 = 0,
    CASE_P_2,
    CASE_I_1,
    CASE_I_2,
    CASE_D_1,
    CASE_D_2,
  };

private:
  float k_p, k_i, k_d;
  float d_p, d_i, d_d;
  float prev_err = 0.0f;
  float integrate_err = 0.0f;

  float best_err = 1000.0f;

  TwiddleCase twiddle_case_ = CASE_P_1;

public:
  YawRatePID(const float &k_p = 0.0, const float &k_i = 0.0,
             const float &k_d = 0.0)
      : k_p(k_p), k_i(k_i), k_d(k_d) {
    d_p = 1.0;
    d_i = 1.0;
    d_d = 1.0;
  }

  void twiddle(const float& cur_err) {
    if ((d_p + d_i + d_d) > 0.2) {
      if (cur_err < best_err) {
        best_err = cur_err;
        switch (twiddle_case_) {
        case CASE_P_1:
          d_p *= 1.1;
          twiddle_case_ = CASE_I_1;
          break;
        case CASE_P_2:
          d_p *= 1.1;
          twiddle_case_ = CASE_I_1;
          break;
        case CASE_I_1:
          d_i *= 1.1;
          twiddle_case_ = CASE_D_1;
          break;
        case CASE_I_2:
          d_i *= 1.1;
          twiddle_case_ = CASE_D_1;
          break;
        case CASE_D_1:
          d_d *= 1.1;
          twiddle_case_ = CASE_P_1;
          break;
        case CASE_D_2:
          d_d *= 1.1;
          twiddle_case_ = CASE_P_1;
          break;
        default:
          break;
        }
      } else {
        switch (twiddle_case_) {
        case CASE_P_1:
          k_p -= 2 * d_p;
          twiddle_case_ = CASE_P_2;
          break;
        case CASE_P_2:
          k_p += d_p;
          d_p *= 0.9;
          twiddle_case_ = CASE_I_1;
          break;
        case CASE_I_1:
          k_i -= 2 * d_i;
          twiddle_case_ = CASE_I_2;
          break;
        case CASE_I_2:
          k_i += d_i;
          d_i *= 0.9;
          twiddle_case_ = CASE_D_1;
          break;
        case CASE_D_1:
          k_d -= 2 * d_d;
          twiddle_case_ = CASE_D_2;
          break;
        case CASE_D_2:
          k_d += d_d;
          d_d *= 0.9;
          twiddle_case_ = CASE_P_1;
          break;
        default:
          break;
        }
      }
    }
  }

  void setGain(const float &k_p_in, const float &k_i_in, const float &k_d_in) {
    k_p = k_p_in;
    k_i = k_i_in;
    k_d = k_d_in;
  }

  std::vector<float> getGain() { return std::vector<float>{k_p, k_i, k_d}; }

  uint8_t update(float object_val, float cur_val) {

    auto cur_err = object_val - cur_val;
    auto diff_err = cur_err - prev_err;
    integrate_err += cur_err;

    twiddle(cur_err);

    prev_err = cur_err;
    std::cout << "cur_err : " << cur_err << std::endl;

    uint8_t output = -k_p * cur_err - k_i * integrate_err - k_d * diff_err;

    std::cout << "output : " << int(output) << std::endl;

    std::cout << "\nk_p : " << k_p << 
        "\n k_i : " << k_i << 
        "\n k_d : " << k_d << std::endl;

    return output;
  }

  ~YawRatePID() {}
};

class CmdToSRC : public rclcpp::Node {
private:
  rclcpp::Publisher<SRCMsg>::SharedPtr src_msg_pub_;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<Float32>::SharedPtr accel_vel_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  rclcpp::Time prev_time;
  double prev_heading_;
  double yaw_speed_;

  // TODO: src_msg params & yaml file (for accel, daccel, etc...)
  SRCMsg src_msg_;
  uint steering_offset_ = 20;

  float accel_;
  float deaccel;
  unsigned int scale;

  YawRatePID pid_controller_;

public:
  CmdToSRC() : Node("cmd_vel_to_src_msg") {
    RCLCPP_INFO(get_logger(), "Cmd_Vel to SRC_Msg Node Created");

    src_msg_pub_ = create_publisher<SRCMsg>("src_control", 10);

    imu_sub_ = create_subscription<Imu>(
        "imu/data", 10,
        std::bind(&CmdToSRC::imu_cb, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<Twist>(
        "cmd_vel", 10,
        std::bind(&CmdToSRC::cmd_vel_cb, this, std::placeholders::_1));

    accel_vel_sub_ = create_subscription<Float32>(
        "accel_vel", 10,
        std::bind(&CmdToSRC::accel_vel_cb, this, std::placeholders::_1));

    pub_timer_ = this->create_wall_timer(
        20ms, std::bind(&CmdToSRC::timer_callback, this));

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
    prev_heading_ = 0.0;

    pid_controller_.setGain(30.0, 0.0, 0.0);
  }

  void imu_cb(const Imu::SharedPtr msg) {

    auto current_time = this->get_clock()->now();
    const double dt = (current_time - prev_time).seconds();
    // RCLCPP_INFO(get_logger(), "dt : %f", dt); // dt 0.02 ok

    prev_time = current_time;

    tf2::Quaternion q_;

    q_[0] = msg->orientation.x;
    q_[1] = msg->orientation.y;
    q_[2] = msg->orientation.z;
    q_[3] = msg->orientation.w;

    tf2::Matrix3x3 m(q_);
    double roll, pitch, heading;
    m.getRPY(roll, pitch, heading);

    yaw_speed_ = (heading - prev_heading_) / dt;
    prev_heading_ = heading;

    RCLCPP_INFO(get_logger(), "yaw : %f", heading);
    RCLCPP_INFO(get_logger(), "angular_z : %f", yaw_speed_);
  }

  void cmd_vel_cb(const Twist::SharedPtr msg) {
    src_msg_.speed = msg->linear.x;

    // TODO : move controller into timer cb

    if (fabs(src_msg_.speed) >= 0.1)
      src_msg_.steering =
          steering_offset_ + pid_controller_.update(msg->angular.z, yaw_speed_);
    else
      src_msg_.steering = steering_offset_;
    RCLCPP_INFO(get_logger(), "steering : %d", src_msg_.steering); // dt 0.02 ok

    // src_msg_.steering =
    //     -((msg->angular.z) / 2 * steering_offset_) + steering_offset_;

    // TODO: Light On Off mode

    src_msg_.direction = src_msg_.speed >= 0 ? 1 : 0;
  }

  void accel_vel_cb(const Float32::SharedPtr msg) {
    src_msg_.accel = msg->data;
    src_msg_.deaccel = msg->data;
  }

  void timer_callback() { src_msg_pub_->publish(src_msg_); }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto cmd_to_src = std::make_shared<CmdToSRC>();

  rclcpp::spin(cmd_to_src);
  rclcpp::shutdown();

  return 0;
}