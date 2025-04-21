cmake_minimum_required(VERSION 3.8)
project(docking_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)  # Added

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(docking_pose_node src/docking_pose_node.cpp)
ament_target_dependencies(docking_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  tf2_sensor_msgs
)
target_link_libraries(docking_pose_node ${PCL_LIBRARIES})

install(TARGETS docking_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 rviz 文件
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()