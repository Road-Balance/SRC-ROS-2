# SRC-ROS-2

Environment Setup 

* Ubuntu 20.04
* ROS2 Foxy
* Install Gazebo
* Dependency Packages

## Simplified SRC Description

* robot state publisher & joint state publisher => robot description
* rviz launch

```
ros2 launch src_description src_description.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164446136-6d672a84-7492-4b1e-980c-d7bd01c17c86.png)

## SRC Simulation 1 - SRC & Empty World

* Gazebo launch
* SRC spawn & ros2_control launch

```
ros2 launch src_gazebo src_gazebo_empty_world.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164446956-0b621647-d80b-4c97-909c-9325354dd427.png)

## SRC Simulation 2 - SRC & MIT Racecourse

* Gazebo launch
* rqt_robot_steering launch
* rviz launch

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164447235-754808f0-bf47-4b63-88f7-92846a81f026.png)

![image](https://user-images.githubusercontent.com/12381733/164447257-73ab6f38-aade-4d16-ae4b-35b3d4aff65c.png)

## Controller Pkg

* Move forward for 5 seconds

```
ros2 run src_gazebo_controller basic_control
```

* Ground Truth odometry Publisher utility

```
ros2 run src_gazebo_controller odom_utility_tools
```

![image](https://user-images.githubusercontent.com/12381733/164449881-4698ee8c-9185-4960-b453-120f7869efbc.png)

자체 제작 odom과의 비교

![image](https://user-images.githubusercontent.com/12381733/164452303-43e9c5e3-2a31-41e1-94ba-0d0e6cd96adb.png)

* src_gazebo_controller.py

ackermann steering을 위한 controller 수식은 `readme` 참고



# TODO
- [] 우분투에서 캡쳐 다시하기
- [] 