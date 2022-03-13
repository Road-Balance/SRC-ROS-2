ROS 2 시간 체계

https://answers.ros.org/question/287946/ros-2-time-handling/

* #include <ros/time.h> => #include <rclcpp/rclcpp.hpp>
* const ros::Time &time => const rclcpp::Time &time
* const ros::Duration &period => const rclcpp::Duration &period
* Time duration
```C++
const double dt = (time - timestamp_).toSec();
if (dt < 0.0001)
```

```C++
const double dt = (time - timestamp_).seconds();
if (dt < 0.0001)
```

단순하게 생각하자.

* joint_state & FloatMultiArray 받아서 speed, angle joint 값을 얻는다.
* odom계산해 주고 
* tf + odom topic을 publish 하자.
=> 이거 부터 하고 그 외의 기교를 넣자고

ackermann_steering_controller에서 가져와야 하는 것들 

* init
* update
* starting
* brake
* setOdomPubFields
=> 이정도?

odometry에서 사용되는 api
* init
* setVelocityRollingWindowSize
* setWheelParams
* udpate -> updateOpenLoop / udpate -> update
* getHeading / getX / getY / getLinear / getAngular

주의
항상 이 순서가 아니다.

- left_front_wheel_joint
- left_rear_wheel_joint
- right_front_wheel_joint
- right_rear_wheel_joint
- left_steering_hinge_joint
- right_steering_hinge_joint

Duration API 관련 - https://github.com/cra-ros-pkg/robot_localization/blob/b5345749e1449a5ec3536d3c8cc80f1de65ae9f6/src/navsat_transform.cpp#L196

ROS 1 - ros::Duration이 double도 받는다.
```
periodicUpdateTimer_ = nh.createTimer(ros::Duration(1./frequency), &NavSatTransform::periodicUpdate, this);
```

ROS 2 - rclcpp::Duration은 int seconds, int nanosecond만 받음, 따라서 아래와 같이 timer를 만들어줌
```
auto interval = std::chrono::duration<double>(1.0 / frequency);
timer_ = this->create_wall_timer(interval, std::bind(&NavSatTransform::transformCallback, this));
```

# TODO

* [] realtime stuffs
    - https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#id4
    - http://control.ros.org/