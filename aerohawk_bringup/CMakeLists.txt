cmake_minimum_required(VERSION 3.8)
project(aerohawk_bringup)

find_package(ament_cmake REQUIRED)

# Nothing to compile - just copy the launch and config folders into the install space
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
