```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py use_rviz:=false

ros2 launch src_nav bringup_launch.py use_sim_time:=true open_rviz:=true
=> 이건 된다!! 이상하게 따로 하면 안됨...

# terminal 1
ros2 launch src_nav localization_launch.py use_sim_time:=true
# terminal 2 => 여기서 에러 
ros2 launch src_nav navigation_launch.py use_sim_time:=true
# terminal 3
ros2 launch src_nav rviz_view_launch.py use_sim_time:=true
```

```
[ERROR] [bt_navigator-4]: process has died [pid 4986, exit code -8, cmd '/opt/ros/foxy/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmpfe0pkevn -r /tf:=tf -r /tf_static:=tf_static'].
```

=> basic_mobile_robot에서도 그런다. 
이건 bt navigator 자체의 문제임 => 소스코드 빌드 시도 

```
[ERROR] [bt_navigator-12]: process has died [pid 3412, exit code -8, cmd '/opt/ros/foxy/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmpzglxw2pq -r /tf:=tf -r /tf_static:=tf_static'].
```

=> 소스코드 빌드해야 한다!!!

```
git clone -b foxy-devel https://github.com/ros-planning/navigation2.git
cba
```

