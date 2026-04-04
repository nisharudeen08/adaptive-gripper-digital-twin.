cmake_minimum_required(VERSION 3.5)
project(robotic_description)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files (controllers.yaml)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files (for future STL usage)
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS
    scripts/arm_mover.py
    scripts/gripper_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
