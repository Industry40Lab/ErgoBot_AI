cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robot_interface REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit
  moveit_msgs
  moveit_visual_tools
  moveit_ros_planning_interface)

include_directories(include)



add_executable(arm_handler src/holding_controller.cpp src/moverobotclass.cpp)
ament_target_dependencies(arm_handler rclcpp robot_interface Boost ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(arm_handler ${arm_controller_LIBRARIES})



install(TARGETS
  arm_handler
  DESTINATION   lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
