#include "rclcpp/rclcpp.hpp"
#include "src_odometry/odometry.hpp"

using Odometry = ackermann_steering_controller::Odometry;

class TestClass : public rclcpp::Node
{
private:
    std::string _name = "test";
    Odometry my_odom;
public:
    TestClass(): Node("topic_sub_oop_node"){
        RCLCPP_INFO(this->get_logger(), "I am test Class");
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestClass>());
    rclcpp::shutdown();

    return 0;
}