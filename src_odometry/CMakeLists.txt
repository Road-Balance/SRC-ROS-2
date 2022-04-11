cmake_minimum_required(VERSION 3.5)
project(src_odometry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
)

add_executable(pub_sub_exercise src/pub_sub_exercise.cpp)
ament_target_dependencies(pub_sub_exercise ${dependencies})

add_executable(src_odometry_gazebo src/src_odometry_gazebo.cpp src/odometry.cpp)
ament_target_dependencies(src_odometry_gazebo ${dependencies})

add_executable(src_odometry_real src/src_odometry_real.cpp src/odometry_real.cpp)
ament_target_dependencies(src_odometry_real ${dependencies})

add_executable(odom_test src/test.cpp src/odometry.cpp)
ament_target_dependencies(odom_test rclcpp geometry_msgs sensor_msgs std_msgs)

add_executable(src_odom src/src_odom.cpp)
ament_target_dependencies(src_odom rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_msgs nav_msgs)

add_executable(src_odom_compatible src/src_odom_compatible.cpp)
ament_target_dependencies(src_odom_compatible rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_msgs nav_msgs)


install(
  TARGETS
    odom_test
    pub_sub_exercise
    # gazebo odom
    src_odometry_gazebo
    # gazebo odom without boost
    src_odometry_real
    # real odom
    src_odom
    # gazebo + real intergrated odom
    src_odom_compatible
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)


ament_package()
