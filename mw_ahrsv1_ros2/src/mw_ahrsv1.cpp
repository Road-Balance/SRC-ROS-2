#include "mw_ahrsv1_ros2/Serial.h"
#include <iostream>
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

typedef struct {
  // ang
  float roll;
  float pitch;
  float yaw;

  // acc
  float linear_acc_x;
  float linear_acc_y;
  float linear_acc_z;

  // gyr
  float angular_vel_x;
  float angular_vel_y;
  float angular_vel_z;

} IMUMsg;

class MW_AHRS : public rclcpp::Node {

private:
  // Device Name
  int dev = 0;

  // Data buffer
  char buffer[120];
  char small_buffer[10];

  // utils for serial cmd
  unsigned char angle_cmd[5] = {0x61, 0x6E, 0x67, 0x0D, 0x0A}; // ang Enter
  unsigned char gyr_cmd[5] = {0x67, 0x79, 0x72, 0x0D, 0x0A};   // ang Enter
  unsigned char acc_cmd[5] = {0x61, 0x63, 0x63, 0x0D, 0x0A};   // ang Enter
  unsigned char reset_cmd[5] = {0x7A, 0x72, 0x6F, 0x0D, 0x0A}; // zro Enter
  unsigned char av_cmd[7] = {0x61, 0x76, 0x3D, 0x31,
                             0x30, 0x0D, 0x0A}; // av = 10
  unsigned char speed_cmd[8] = {0x73, 0x70, 0x3D, 0x31,
                                0x30, 0x30, 0x0D, 0x0A}; // sp=100 Enter
  unsigned char ros_data_cmd[6] = {0x73, 0x73, 0x3D,
                                   0x37, 0x0D, 0x0A}; // ss=7 Enter

  // Serperate Euler Angle Variable
  int ang_count = 0;

  // ASCII CODE
  const unsigned char A = 0x61;
  const unsigned char N = 0x6E;
  const unsigned char G = 0x67;
  const unsigned char CR = 0x0D;
  const unsigned char LF = 0x0A;

  // Unit converting constants
  double convertor_g2a = 9.80665; // for linear_acceleration (g to m/s^2)
  double convertor_d2r =
      M_PI / 180.0; // for angular_velocity (degree to radian)
  double convertor_r2d =
      180.0 / M_PI;                // for easy understanding (radian to degree)
  double convertor_ut2t = 1000000; // for magnetic_field (uT to Tesla)
  double convertor_c = 1.0;        // for temperature (celsius)

  // ROS Parts

  // whether pub tf or not
  bool publish_tf = true;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_,
      imu_data_pub_;

  // hold raw data
  IMUMsg imu_raw_data;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

public:
  MW_AHRS() : Node("mv_ahrsv1_node") {
    for (int i = 0; i < sizeof(buffer); i++)
      buffer[i] = 0;

    // int open_serial(char *dev_name, int baud, int vtime, int vmin);
    dev = open_serial((char *)"/dev/ttyUSB0", 115200, 0, 0);

    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&MW_AHRS::test, this));

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(), "Constructor...");
  }

  ~MW_AHRS() {
    RCLCPP_INFO(get_logger(), "Destructor...");
    close_serial(dev);
  }

  tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw) {
    float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) -
               (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
    float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
    float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) -
               (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
    float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

    tf2::Quaternion q(qx, qy, qz, qw);
    return q;
  }

  void reset_imu() {
    write(dev, reset_cmd, 5);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    if (buffer[0] == 'z' && buffer[1] == 'r' && buffer[2] == 'o') {
      printf("IMU Reset\n");
    }
  }

  void speed_setup() {
    write(dev, speed_cmd, 8);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    if (buffer[0] == 's' && buffer[1] == 'p' && buffer[2] == '=' &&
        buffer[3] == '1' && buffer[4] == '0') {
      printf("Serial Speed Reset\n");
    }
  }

  void start_data_stream() {
    write(dev, ros_data_cmd, 6);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';
  }

  void get_angle_data(IMUMsg &msg_in) {
    write(dev, angle_cmd, 5);
    read(dev, &buffer, sizeof(buffer));

    if (buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g') {
      char *ptr = strtok(buffer, " ");

      ang_count = 0;

      while (ptr != NULL) {
        ang_count++;

        ptr = strtok(NULL, " ");

        if (ang_count == 1) {
          msg_in.roll = atof(ptr);
        } else if (ang_count == 2) {
          msg_in.pitch = atof(ptr);
        } else if (ang_count == 3) {
          msg_in.yaw = atof(ptr);
        }
      }
    }
  }

  void parse_ss_data(IMUMsg &msg_in) {
    read(dev, &buffer, sizeof(buffer));

    if (int(buffer[0]) < 97) {
      // std::cout << buffer << std::endl;
      char *rest;
      char *token;
      char *ptr = buffer;

      ang_count = 0;

      while (token = strtok_r(ptr, " ", &rest)) {
        ang_count++;

        if (ang_count == 1) {
          msg_in.linear_acc_x = atof(token);
        } else if (ang_count == 2) {
          msg_in.linear_acc_y = atof(token);
        } else if (ang_count == 3) {
          msg_in.linear_acc_z = atof(token);
        } else if (ang_count == 4) {
          msg_in.angular_vel_x = atof(token);
        } else if (ang_count == 5) {
          msg_in.angular_vel_y = atof(token);
        } else if (ang_count == 6) {
          msg_in.angular_vel_z = atof(token);
        } else if (ang_count == 7) {
          msg_in.roll = atof(token) * -convertor_d2r;
        } else if (ang_count == 8) {
          msg_in.pitch = atof(token) * -convertor_d2r;
        } else if (ang_count == 9) {
          msg_in.yaw = atof(token) * -convertor_d2r;
        }
        ptr = rest;
      }
    }
  }

  void test() {
    auto imu_data_msg = sensor_msgs::msg::Imu();

    parse_ss_data(imu_raw_data);

    tf2::Quaternion orientation = Euler2Quaternion(
        imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    tf2::Quaternion yaw_rotate;
    yaw_rotate.setRPY(0, 0, 90 * convertor_d2r);

    tf2::Quaternion q;
    q.setRPY(0, 0, -90 * convertor_d2r);

    // Apply rotation for 90 degree.
    tf2::Quaternion new_orientation = q * orientation * yaw_rotate;

    // Debugging Console
    std::cout << new_orientation[0] << std::endl;
    std::cout << new_orientation[1] << std::endl;
    std::cout << new_orientation[2] << std::endl;
    std::cout << "or" << std::endl;

    rclcpp::Time now = this->now();

    imu_data_msg.header.stamp = now;
    imu_data_msg.header.frame_id = "imu_link";

    // orientation
    imu_data_msg.orientation.x = new_orientation[0];
    imu_data_msg.orientation.y = new_orientation[1];
    imu_data_msg.orientation.z = new_orientation[2];
    imu_data_msg.orientation.w = new_orientation[3];

    // original data used the g unit, convert to m/s^2
    imu_data_msg.linear_acceleration.x =
        -imu_raw_data.linear_acc_y * convertor_g2a;
    imu_data_msg.linear_acceleration.y =
        imu_raw_data.linear_acc_x * convertor_g2a;
    imu_data_msg.linear_acceleration.z =
        imu_raw_data.linear_acc_z * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_msg.angular_velocity.x =
        imu_raw_data.angular_vel_x * convertor_d2r;
    imu_data_msg.angular_velocity.y =
        imu_raw_data.angular_vel_y * convertor_d2r;
    imu_data_msg.angular_velocity.z =
        imu_raw_data.angular_vel_z * convertor_d2r;

    imu_data_msg.linear_acceleration_covariance[0] =
        imu_data_msg.linear_acceleration_covariance[4] =
            imu_data_msg.linear_acceleration_covariance[8] = 1000;

    imu_data_msg.angular_velocity_covariance[0] =
        imu_data_msg.angular_velocity_covariance[4] =
            imu_data_msg.angular_velocity_covariance[8] = 1;

    imu_data_msg.orientation_covariance[0] =
        imu_data_msg.orientation_covariance[4] =
            imu_data_msg.orientation_covariance[8] = 0;

    if (publish_tf) {
      std::cout << "Flag" << std::endl;

      geometry_msgs::msg::TransformStamped transform;

      transform.header.stamp = now;
      transform.header.frame_id = "base_link";
      transform.child_frame_id = "imu_link";

      transform.transform.rotation.x = new_orientation[0];
      transform.transform.rotation.y = new_orientation[1];
      transform.transform.rotation.z = new_orientation[2];
      transform.transform.rotation.w = -new_orientation[3];

      broadcaster_->sendTransform(transform);
    }

    imu_data_pub_->publish(imu_data_msg);
  }

  void pub_msg() {

    auto imu_data_msg = sensor_msgs::msg::Imu();

    parse_ss_data(imu_raw_data);

    // tf::Quaternion orientation = tf::createQuaternionFromRPY(
    //     imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    // tf::Quaternion yaw_rotate;
    // yaw_rotate.setRPY(0, 0, 90 * convertor_d2r);

    // tf::Quaternion q;
    // q.setRPY(0, 0, -90 * convertor_d2r);

    tf2::Quaternion orientation = Euler2Quaternion(
        imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    tf2::Quaternion yaw_rotate;
    yaw_rotate.setRPY(0, 0, 90 * convertor_d2r);

    tf2::Quaternion q;
    q.setRPY(0, 0, -90 * convertor_d2r);

    // tf::Quaternion new_orientation = orientation;
    // tf::Quaternion new_orientation = q * orientation;

    tf2::Quaternion new_orientation = q * orientation * yaw_rotate;

    // tf::Quaternion new_orientation = orientation;

    // tf::Quaternion yaw_rotate(0, 0, -0.7071068, 0.7071068);
    // tf::Quaternion yaw_rotate(0, 0, -1, 0);

    rclcpp::Time now = this->now();

    imu_data_msg.header.stamp = now;
    imu_data_msg.header.frame_id = "imu_link";

    // orientation
    imu_data_msg.orientation.x = new_orientation[0];
    imu_data_msg.orientation.y = new_orientation[1];
    imu_data_msg.orientation.z = new_orientation[2];
    imu_data_msg.orientation.w = new_orientation[3];

    // original data used the g unit, convert to m/s^2
    imu_data_msg.linear_acceleration.x =
        -imu_raw_data.linear_acc_y * convertor_g2a;
    imu_data_msg.linear_acceleration.y =
        imu_raw_data.linear_acc_x * convertor_g2a;
    imu_data_msg.linear_acceleration.z =
        imu_raw_data.linear_acc_z * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_msg.angular_velocity.x =
        imu_raw_data.angular_vel_x * convertor_d2r;
    imu_data_msg.angular_velocity.y =
        imu_raw_data.angular_vel_y * convertor_d2r;
    imu_data_msg.angular_velocity.z =
        imu_raw_data.angular_vel_z * convertor_d2r;

    // imu_data_msg.linear_acceleration_covariance[0] =
    //     imu_data_msg.linear_acceleration_covariance[4] =
    //         imu_data_msg.linear_acceleration_covariance[8] = 1000;

    // imu_data_msg.angular_velocity_covariance[0] =
    //     imu_data_msg.angular_velocity_covariance[4] =
    //         imu_data_msg.angular_velocity_covariance[8] = 1;

    // imu_data_msg.orientation_covariance[0] =
    // imu_data_msg.orientation_covariance[4]
    // =
    //     imu_data_msg.orientation_covariance[8] = 0;

    imu_data_pub_->publish(imu_data_msg);

    if (publish_tf) {

      std::cout << "Flag" << std::endl;

      // tf::Transform transform;

      // transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      // transform.setRotation(new_orientation);
      // broadcaster_.sendTransform(tf::StampedTransform(
      //     transform, ros::Time::now(), "imu_link", "base_link"));

      // broadcaster_.sendTransform(tf::StampedTransform(
      //     tf::Transform(orientation, tf::Vector3(0.0, 0.0, 0.0)),
      //     ros::Time::now(), "imu_link", "base_link"));

      geometry_msgs::msg::TransformStamped tf;

      tf.header.stamp = now;
      tf.header.frame_id = "imu_link";
      tf.child_frame_id = "base_link";

      tf.transform.translation.x = 0.0f;
      tf.transform.translation.y = 0.0f;
      tf.transform.translation.z = 0.0f;

      // tf.transform.rotation = imu_data_msg.orientation;

      tf.transform.rotation.x = new_orientation[0];
      tf.transform.rotation.y = new_orientation[1];
      tf.transform.rotation.z = new_orientation[2];
      tf.transform.rotation.w = 0; //-new_orientation[3];

      broadcaster_->sendTransform(tf);
    }
  }
};

int main(int argc, char **argv) {
  // ros::init(argc, argv, "mw_ahrsv1");

  // MW_AHRS ahrs_obj;
  // ros::Rate rate(10);

  // ahrs_obj.reset_imu();
  // ahrs_obj.speed_setup();
  // ahrs_obj.start_data_stream();

  // // ros::Duration(1.5).sleep();
  // IMUMsg test_imu_raw_data;

  // while (ros::ok()) {

  //   // ahrs_obj.parse_ss_data(test_imu_raw_data);
  //   ahrs_obj.pub_msg();

  //   rate.sleep();
  // }

  rclcpp::init(argc, argv);

  auto imu_node = std::make_shared<MW_AHRS>();

  imu_node->reset_imu();
  imu_node->speed_setup();
  imu_node->start_data_stream();

  rclcpp::spin(imu_node);

  // rclcpp::WallRate r(1);

  // while (rclcpp::ok()) {
  //   imu_node->pub_msg();
  //   rclcpp::spin_some(imu_node);
  //   r.sleep();
  // }

  rclcpp::shutdown();

  return 0;
}