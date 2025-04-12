cmake_minimum_required(VERSION 3.8)
project(binbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Install directory to share foler.
install(
  DIRECTORY launch meshes urdf config rviz objects bt include
  DESTINATION share/${PROJECT_NAME}/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

##############  Add executable Hello Moveit ##############
add_executable(hello_moveit src/hello_moveit.cpp)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  
ament_target_dependencies(
    hello_moveit
    "moveit_ros_planning_interface"
    "rclcpp"
    "behaviortree_cpp_v3"
)


install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

##############  Add executable bt_graspbot ##############
add_executable(bt_graspbot 
src/bt_graspbot.cpp 
src/panda_robot_interface.cpp
src/move_to_pose_node.cpp
src/open_gripper_node.cpp
src/close_gripper_node.cpp
src/attach_object_node.cpp
src/detach_object_node.cpp
src/add_collision_object_node.cpp
src/move_home_node.cpp
src/wait_node.cpp
)

target_include_directories(bt_graspbot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(bt_graspbot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17 

ament_target_dependencies(
  bt_graspbot
  "rclcpp"
  "moveit_ros_planning_interface"
  "behaviortree_cpp_v3"
)

install(TARGETS bt_graspbot
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(bt_graspbot
  BT::behaviortree_cpp_v3
)

###### End #######

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
