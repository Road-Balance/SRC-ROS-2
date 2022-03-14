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

문제
현재 odom은 다음 두개의 값을 받음
```
double wheel_pos  = rear_wheel_joint_.getPosition();
double steer_pos = front_steer_joint_.getPosition();
```
gazebo에서는 뒷바퀴의 양쪽 회전각이 다르다.
/steering_angle_middle 처럼, /throttling_angle_middle을 만들어서 publish 해보자.

steering_angle_middle은 되지만, throttling_angle_middle은 안된다.
throttling_vel만 다룰 수 있음

!!결론!! => 양쪽 뒷바퀴 pose를 평균내서 사용하자. (linear 관계라고 가정하면 성립함)


주기적으로 0이 나온다.
```
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
0 0 0
```

createQuaternionMsgFromYaw => 직접 만들어 쓰기
https://answers.ros.org/question/364561/tfcreatequaternionfromyaw-equivalent-in-ros2/

한바퀴 아직 안돌았는데 이미 한바퀴 넘어 있음
open loop 써도 그러네

=> controller가 잘못되었나 보다 ㅅㅂ...
수치가 잘못되었는지 살펴보자.

![image](https://user-images.githubusercontent.com/12381733/158050479-0305fa59-55b7-4563-95cf-3d66a2d3a24e.png)

샤시가 뒤쪽이었다.

![image](https://user-images.githubusercontent.com/12381733/158050513-d010aa3f-0a4c-4267-9c5e-0d4b72774f5e.png)

right_rear_wheel로는 바퀴 사이 거리 유추 불가

```
$ ros2 run tf2_ros tf2_echo chassis right_front_wheel
[INFO] [1647158272.592267167] [tf2_echo]: Waiting for transform chassis ->  right_front_wheel: Invalid frame ID "chassis" passed to canTransform argument target_frame - frame does not exist
At time 1647158273.536024634
- Translation: [0.325, -0.100, 0.000]
- Rotation: in Quaternion [-0.500, -0.500, 0.500, -0.500]
```

이건 이 둘 사이 거리니까, 실제 car_wheel_threat는 2배가 되어야지 ㅇㅇ 맞네 ㅠ
왼쪽 오른쪽 사이 거리 확인 

```xml
<joint name="left_steering_hinge_joint" type="revolute">
    <origin xyz="0.325 0.0 0" rpy="0 1.5708 0" />
    <parent link="chassis" />
```

![image](https://user-images.githubusercontent.com/12381733/158051253-99f378b6-4cd9-40f1-a3b4-d773c624a084.png)

완전 가운데가 아니네, 바퀴 두께가 있었음 (0.045)
```
<xacro:macro name="left_wheels_collision_geometry">
    <origin xyz="0 0 -0.0225" rpy="0 0 0" />
    <geometry>
        <cylinder length="0.045" radius="0.05" />
    </geometry>
</xacro:macro>
```
그렇다면, 실제 오른쪽/왼쪽 바퀴 사이 거리는 (0.1 + 0.0225) * 2 = 0.245
=> 여전히 빠르다. 

컨트롤러를 바꾸자. => Bycicle Model로!
```

```

rqt_plot을 통해 수치 비교를 해보자. (odom을 가지고)
에러 => rqt_plot 했는데, 시간축이 안간다.

![image](https://user-images.githubusercontent.com/12381733/158099629-8f8413ac-967e-4bf0-be5d-447b24357b25.png)

실제 topic stamp 업데이트가 안되고 있었음
```c++
odom->header.frame_id = odom_frame_id_;
odom->child_frame_id = base_frame_id_;
odom->header.stamp = last_state_publish_time_;
```

선속도 0.3 각속도 0.5여서 반지름 0.6짜리 원을 그려야 하는 상황
- open loop 사용시
![image](https://user-images.githubusercontent.com/12381733/158100684-4cbbfdec-9446-434a-ab48-1444218678ff.png)
rqt_plot 결과 odom은 맞다. => 그렇다면, 컨트롤러가 잘못 된 것!

- closed loop 사용 시
지름 0.75짜리가 나왔다.
![image](https://user-images.githubusercontent.com/12381733/158101170-aef02955-0ff4-4529-9a15-40d638f46c99.png)

=> 일단 open loop로 두고, 여기에 컨트롤러를 맞춰보자.

gazebo 절대 좌표도 같이 tracking하기
```
ros2 launch src_gazebo src_gazebo.launch.py
ros2 run src_gazebo_controller odom_utility_tools
ros2 run src_odometry src_odometry_gazebo
rqt
```

![image](https://user-images.githubusercontent.com/12381733/158103127-9dc1d328-60ac-42e6-a658-d735600dba9f.png)

지금 1.2가 나와야하는데
아래를 보면 실제 gt odom은 1.5 좀 적게 나온다. 
=> 컨트롤러 바꿔보자. (w를 키우거나, v를 줄이거나)

throttle값 통일
```
self.throttling_msg.data = [
    wheel_turnig_speed_com,
    wheel_turnig_speed_com,
    wheel_turnig_speed_com,
    wheel_turnig_speed_com,
]
```

지름 더 커짐, 약 1.7
![image](https://user-images.githubusercontent.com/12381733/158104023-6046f158-a62d-4770-9c23-1050e2030f5a.png)
회전각 변경
```
# alfa_right_front_wheel = math.atan(self.L / turning_radius_right_front_wheel)
alfa_right_front_wheel = math.atan( self.L / turning_radius_middle )

# alfa_left_front_wheel = math.atan(self.L / turning_radius_left_front_wheel)
alfa_left_front_wheel = math.atan( self.L / turning_radius_middle )
```
![image](https://user-images.githubusercontent.com/12381733/158104023-6046f158-a62d-4770-9c23-1050e2030f5a.png)
살짝 작아짐... 하지만 여전이 1.5근방

새로운 방식 써보자.
```
alfa_right_front_wheel = (omega_base_link * self.L ) / vel_base_link
```
![image](https://user-images.githubusercontent.com/12381733/158104849-617bac3a-6a5d-4805-9d72-4e172d57a5e3.png)
좀 더 작아졌지만 1.25 넘어버리는 상황

wheel speed를 순수 속도와 반지름으로만 사용하도록 수정
```
raw_wheel_speed = vel_base_link / self.wheel_radius
self.throttling_msg.data = [
    raw_wheel_speed,
    raw_wheel_speed,
    raw_wheel_speed,
    raw_wheel_speed,
]
```
아직도 1.25 넘는다.
![image](https://user-images.githubusercontent.com/12381733/158105482-7a9035e8-012b-478d-968f-adb35e760405.png)

angle에 atan값을 먹여보았다.
```
alfa_left_front_wheel = math.atan((omega_base_link * self.L ) / vel_base_link)
```
=> 더커진다 ㅠ

![image](https://user-images.githubusercontent.com/12381733/158106036-a71dc337-acd2-4947-92e2-7136394b6328.png)

# TODO

* [] realtime stuffs
    - https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#id4
    - http://control.ros.org/