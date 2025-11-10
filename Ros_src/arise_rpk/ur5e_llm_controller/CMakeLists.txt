cmake_minimum_required(VERSION 3.8)
project(ur5e_llm_controller)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()