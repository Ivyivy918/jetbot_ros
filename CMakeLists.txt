cmake_minimum_required(VERSION 3.5)
project(jetbot_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find CUDA and jetson-utils if needed
find_package(CUDA QUIET)
if(CUDA_FOUND)
  find_package(jetson-utils QUIET)
endif()

# Include directories
include_directories(
  ${CUDA_INCLUDE_DIRS}
)

# Build jetbot_camera node if jetson-utils is found
if(jetson-utils_FOUND)
  add_executable(jetbot_camera 
    src/jetbot_camera.cpp 
    src/image_converter.cpp 
    src/ros_compat.cpp
  )
  ament_target_dependencies(jetbot_camera
    rclcpp
    sensor_msgs
    std_msgs
  )
  target_link_libraries(jetbot_camera 
    ${CUDA_LIBRARIES}
    jetson-utils
  )
  
  install(TARGETS
    jetbot_camera
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/jetbot_motors.py
  scripts/jetbot_oled.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config files
install(DIRECTORY
  ../../rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install config files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()