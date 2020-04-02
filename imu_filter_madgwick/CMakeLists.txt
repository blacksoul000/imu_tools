cmake_minimum_required(VERSION 3.5)
project(imu_filter_madgwick)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)

# find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros nodelet pluginlib message_filters dynamic_reconfigure)

# find_package(Boost REQUIRED COMPONENTS system thread signals)

# # Generate dynamic parameters
# generate_dynamic_reconfigure_options(cfg/ImuFilterMadgwick.cfg)

# catkin_package(
#   DEPENDS Boost
#   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs nodelet pluginlib message_filters dynamic_reconfigure
#   INCLUDE_DIRS
#   LIBRARIES imu_filter imu_filter_nodelet
# )

include_directories(include)

# create imu_filter library
add_library (imu_filter src/imu_filter.cpp  src/imu_filter_ros.cpp src/stateless_orientation.cpp)
# add_dependencies(imu_filter ${PROJECT_NAME}_gencfg)
# target_link_libraries(imu_filter ${Boost_LIBRARIES})

# # create imu_filter_nodelet library
# add_library (imu_filter_nodelet src/imu_filter_nodelet.cpp)
# add_dependencies(imu_filter_nodelet ${PROJECT_NAME}_gencfg)
# target_link_libraries(imu_filter_nodelet imu_filter)

# create imu_filter_node executable
add_executable(imu_filter_node src/imu_filter_node.cpp)
# add_dependencies(imu_filter_node ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_node imu_filter)# ${Boost_LIBRARIES})

ament_export_include_directories(include)
ament_target_dependencies(imu_filter rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
tf2_ros pluginlib message_filters)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
                          tf2_ros pluginlib message_filters)  # nodelet

install(TARGETS
  imu_filter
  # imu_filter_nodelet
  imu_filter_node
  DESTINATION lib/${PROJECT_NAME})

# install(FILES imu_filter_nodelet.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# if(BUILD_TESTING)
#   catkin_add_gtest(${PROJECT_NAME}-madgwick_test
#     test/stateless_orientation_test.cpp
#     test/madgwick_test.cpp
#   )
#   target_link_libraries(${PROJECT_NAME}-madgwick_test
#     imu_filter
#     ${catkin_LIBRARIES}
#   )
# endif()

ament_package()
