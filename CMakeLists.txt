cmake_minimum_required(VERSION 3.8)
project(docking_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)  # Added
find_package(std_srvs REQUIRED) # 新增

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)


#目标感知对接部署
add_executable(deploy_target_perception src/deploy_target_perception.cpp)
ament_target_dependencies(deploy_target_perception
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  tf2_sensor_msgs
)
target_link_libraries(deploy_target_perception ${PCL_LIBRARIES})

install(TARGETS deploy_target_perception
  DESTINATION lib/${PROJECT_NAME}
)

#目标感知位姿估计
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
  std_srvs
)
target_link_libraries(docking_pose_node ${PCL_LIBRARIES} stdc++fs) # 新增：链接 filesystem

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

# 安装 template 文件
install(DIRECTORY template
  DESTINATION share/${PROJECT_NAME}
)

# 安装 script 文件
install(DIRECTORY script
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  # 保持脚本的执行权限
)

ament_package()