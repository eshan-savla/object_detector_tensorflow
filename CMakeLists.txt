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
#find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

# for python code
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# for msgs generation
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "srv/DetectObjects.srv"
  DEPENDENCIES sensor_msgs
 )

include_directories(
    include
)

set(DEPENDENCIES 
    #rclcpp
)

#add_executable(FeatureExtractor
#    src/feature_extractor.cpp
#)

#ament_target_dependencies(FeatureExtractor
#    ${DEPENDENCIES}
#)

#target_link_libraries(FeatureExtractor
#    ${OpenCV_LIBS}
#    petra_core::petra_core_lib
#)

# Install launch files
#install(DIRECTORY
#    launch
#    DESTINATION share/${PROJECT_NAME}/
#)

#install(TARGETS
#  FeatureExtractor
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY
    data
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    ${PROJECT_NAME}/ros_node.py
    ${PROJECT_NAME}/diagnostics.py
    ${PROJECT_NAME}/object_detection.py
    ${PROJECT_NAME}/visualization.py
    ${PROJECT_NAME}/example_usage.py
  DESTINATION lib/${PROJECT_NAME}
)

# for python code
ament_python_install_package(${PROJECT_NAME})

ament_package()
