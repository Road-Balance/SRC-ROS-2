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

## Sensor Fusion

```
ros2 launch src_sensor_fusion src_gazebo_racecourse.launch.py
ros2 launch src_sensor_fusion robot_localization.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164702848-1e41dbc1-b5d5-4dca-b10c-0409ef716bf5.png)

## SLAM

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py use_rviz:=false
ros2 launch src_slam src_slam_gazebo.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164704324-b26fb411-e78a-4c69-90b6-bceed81d3976.png)

## AMCL (Localization)

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py use_rviz:=false
ros2 launch src_amcl amcl.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164706444-f65bbe6a-73aa-441f-abb7-a1ce057123d4.png)

## Navigation 

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py use_rviz:=false
ros2 launch src_nav bringup_launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164715379-02655e8b-58b4-48e4-a09c-c5f4a97fdef4.png)

# TODO
- [] 우분투에서 캡쳐 다시하기 (그림자, 카메라 이미지)
- [] 