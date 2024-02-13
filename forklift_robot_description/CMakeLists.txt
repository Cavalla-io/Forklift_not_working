cmake_minimum_required(VERSION 3.5)
project(forklift_robot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Declare dependencies
ament_package()

# Install directories
install(DIRECTORY
  launch
  urdf
  meshes
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)
