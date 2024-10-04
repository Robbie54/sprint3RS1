cmake_minimum_required(VERSION 3.5)
project(object_identifier_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create the executable
add_executable(object_identifier_node src/object_identifier.cpp)  # Changed to match your project name

# Link ROS2 dependencies
ament_target_dependencies(object_identifier_node  # Updated to match the executable name
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  
)

# Install the executable
install(TARGETS 
  object_identifier_node  # Updated to match the executable name
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Specify include directories if you have any headers
# include_directories(include)

# Package configuration
ament_package()
