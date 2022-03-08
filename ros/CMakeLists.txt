cmake_minimum_required(VERSION 3.5)
project(object_detector_tensorflow)

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
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
    include
)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Detections.msg"
  "srv/DetectObjects.srv"
  DEPENDENCIES sensor_msgs
)

# Install launch and config files
install(DIRECTORY
   launch
   config
   DESTINATION share/${PROJECT_NAME}/
)

# Install python library
install(DIRECTORY
   ../odtf
   DESTINATION lib/python3.8/site-packages/
)

# Install python nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    ${PROJECT_NAME}/client.py
    ${PROJECT_NAME}/continuous_detection_node.py
    ${PROJECT_NAME}/detection_node.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
