#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

using Twist = geometry_msgs::msg::Twist;
using JointState = sensor_msgs::msg::JointState;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class PubSubTest : public rclcpp::Node {
private:
    rclcpp::Publisher<Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr steering_mid_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time now_;
    const float pub_time_ = 5.0;

    Twist twist_msg_;
public:
    PubSubTest(): Node("pub_sub_test"){
        timer_ = this->create_wall_timer(500ms, std::bind(&PubSubTest::timer_callback, this));
        
        now_ = this->get_clock()->now();
        twist_pub_ = this->create_publisher<Twist>("cmd_vel", 10);

        steering_mid_sub_ = this->create_subscription<Float64MultiArray>("steering_angle_middle", 10, 
            std::bind(&PubSubTest::sub_mid_steering, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<JointState>("joint_states", 10, 
            std::bind(&PubSubTest::sub_joint_state, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PubSubTest Node started");
    }

    void timer_callback(){

        if((this->get_clock()->now() - now_).seconds() < pub_time_)
            twist_msg_.linear.x = 0.5;
        else
            twist_msg_.linear.x = 0.0;
        
        twist_pub_->publish(twist_msg_);
    }

    void sub_joint_state(const JointState::SharedPtr msg){
        std::cout << "msg in" << std::endl;
        RCLCPP_INFO(this->get_logger(), "%s", msg->name[0].c_str());
    }

    void sub_mid_steering(const Float64MultiArray::SharedPtr msg){
        std::cout << msg->data[0] << std::endl;
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubSubTest>());
    rclcpp::shutdown();

    return 0;
}
