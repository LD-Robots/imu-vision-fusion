cmake_minimum_required(VERSION 3.8)
project(imu_vision_fusion_bringup)

find_package(ament_cmake REQUIRED)

# Config-only package: install launch files and parameter YAMLs.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
