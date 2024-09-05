cmake_minimum_required(VERSION 3.8)
project(robotic_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(robotic_manipulation src/robotic_manipulation.cpp src/scene_controller.cpp src/arm_controller.cpp src/vla_controller.cpp src/scenarios/pickup.cpp src/scenarios/move_toy.cpp src/scenarios/put_in_box.cpp src/scenario.cpp src/rosbag_recorder.cpp)
target_include_directories(robotic_manipulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robotic_manipulation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robotic_manipulation
  "moveit_ros_planning_interface"
  "rclcpp"
  "gazebo_msgs"
  "vision_msgs"
  "rosbag2_cpp"
)

install(TARGETS robotic_manipulation
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
