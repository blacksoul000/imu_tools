cmake_minimum_required(VERSION 3.5)
project(imu_complementary_filter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(include)

## Declare a cpp library
add_library(complementary_filter
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
)

# create complementary_filter_node executable
add_executable(complementary_filter_node src/complementary_filter_node.cpp)
target_link_libraries(complementary_filter_node complementary_filter)

ament_export_include_directories(include)
ament_target_dependencies(complementary_filter rclcpp message_filters sensor_msgs std_msgs
                          tf2_ros geometry_msgs tf2_geometry_msgs)
ament_export_dependencies(rclcpp message_filters sensor_msgs std_msgs tf2_ros
                          geometry_msgs tf2_geometry_msgs)

install(TARGETS
  complementary_filter_node
  complementary_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
