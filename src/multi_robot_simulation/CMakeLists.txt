cmake_minimum_required(VERSION 3.5)
project(multi_robot_simulation)

install(DIRECTORY 
   config
   launch
   DESTINATION share/${PROJECT_NAME}/
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(launch_testing_ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS 
DESTINATION lib/${PROJECT_NAME})

ament_package()
