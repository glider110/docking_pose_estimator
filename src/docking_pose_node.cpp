#include "docking_pose_estimator/docking_pose_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  // 修改：使用 .hpp 头文件
#include <cmath>

DockingPoseNode::DockingPoseNode() : Node("docking_pose_node"), first_scan_(true) {
  // 声明参数
  declare_parameter("pcd_file", "/home/std/workspace/test_pkg_ws/rtabmap_optimized.pcd");
  declare_parameter("pointcloud_topic", "/camera/points");
  declare_parameter("pose_topic", "/docking_pose");
  declare_parameter("template_cloud_topic", "/template_cloud");
  declare_parameter("transformed_cloud_topic", "/transformed_cloud");
  declare_parameter("voxel_size", 0.05);
  declare_parameter("camera_frame", "camera_rgb_frame");
  declare_parameter("robot_frame", "base_footprint");
  declare_parameter("docking_frame", "docking_target");
  // 滤波范围参数（单位：米）
  declare_parameter("x_min", 0.0);
  declare_parameter("x_max", 3.0);
  declare_parameter("y_min", -1.0);
  declare_parameter("y_max", 1.0);
  declare_parameter("z_min", 0.0);
  declare_parameter("z_max", 2.0);
  // 初始位姿猜测参数（平移单位：米，旋转单位：度）
  declare_parameter("initial_x", 0.0);
  declare_parameter("initial_y", 0.0);
  declare_parameter("initial_z", 0.0);
  declare_parameter("initial_roll", 0.0);
  declare_parameter("initial_pitch", 0.0);
  declare_parameter("initial_yaw", 0.0);

  // 获取参数
  std::string pcd_file, pointcloud_topic, pose_topic, template_cloud_topic, transformed_cloud_topic, camera_frame, robot_frame, docking_frame;
  double voxel_size;
  get_parameter("pcd_file", pcd_file);
  get_parameter("pointcloud_topic", pointcloud_topic);
  get_parameter("pose_topic", pose_topic);
  get_parameter("template_cloud_topic", template_cloud_topic);
  get_parameter("transformed_cloud_topic", transformed_cloud_topic);
  get_parameter("voxel_size", voxel_size);
  get_parameter("camera_frame", camera_frame);
  get_parameter("robot_frame", robot_frame);
  get_parameter("docking_frame", docking_frame);

  // 检查话题名称是否为空
  if (template_cloud_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Template cloud topic is empty! Using default: /template_cloud");
    template_cloud_topic = "/template_cloud";
  }
  if (transformed_cloud_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Transformed cloud topic is empty! Using default: /transformed_cloud");
    transformed_cloud_topic = "/transformed_cloud";
  }

  // 加载对接目标点云模板
  load_docking_template(pcd_file);

  // 设置 ICP 参数
  icp_.setMaximumIterations(50);
  icp_.setTransformationEpsilon(1e-8);
  icp_.setMaxCorrespondenceDistance(0.1);

  // 初始化 TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // 初始化订阅和发布
  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10, std::bind(&DockingPoseNode::pointcloud_callback, this, std::placeholders::_1));
  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
  pub_template_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(template_cloud_topic, 10);
  pub_transformed_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(transformed_cloud_topic, 10);

  // 初始化定时器，每 1 秒发布一次模板点云
  template_publish_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DockingPoseNode::publish_template_cloud, this));

  RCLCPP_INFO(this->get_logger(), "Loaded Docking Pose Estimation Node for estimating docking target pose relative to AMR.");
}

void DockingPoseNode::load_docking_template(const std::string &pcd_file) {
  docking_template_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  std::string expanded_file = pcd_file;
  if (pcd_file.find("~") == 0) {
    expanded_file = std::string(getenv("HOME")) + pcd_file.substr(1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(expanded_file, *docking_template_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read docking target template PCD file: %s", expanded_file.c_str());
    docking_template_.reset();
    return;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  double voxel_size;
  get_parameter("voxel_size", voxel_size);
  voxel.setInputCloud(docking_template_);
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.filter(*docking_template_);

  RCLCPP_INFO(this->get_logger(), "Loaded docking target template with %ld points.", docking_template_->size());

  if (!docking_template_->empty()) {
    icp_.setInputTarget(docking_template_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Docking template is empty after filtering!");
  }
}

void DockingPoseNode::publish_template_cloud() {
  if (!pub_template_cloud_) {
    RCLCPP_ERROR(this->get_logger(), "Template cloud publisher is not initialized!");
    return;
  }

  if (!docking_template_ || docking_template_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Docking template point cloud is empty or not initialized!");
    return;
  }

  sensor_msgs::msg::PointCloud2 template_msg;
  pcl::toROSMsg(*docking_template_, template_msg);

  std::string robot_frame;
  get_parameter("robot_frame", robot_frame);
  template_msg.header.frame_id = robot_frame;
  template_msg.header.stamp = this->get_clock()->now();

  pub_template_cloud_->publish(template_msg);
}

sensor_msgs::msg::PointCloud2 DockingPoseNode::transform_pointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud,
    const std::string &target_frame) {
  std::string camera_frame;
  get_parameter("camera_frame", camera_frame);
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, camera_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform point cloud to base_footprint: %s", ex.what());
    return *cloud;
  }

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*cloud, transformed_cloud, transform);
  transformed_cloud.header.frame_id = target_frame;
  return transformed_cloud;
}

void DockingPoseNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::string robot_frame, docking_frame;
  get_parameter("robot_frame", robot_frame);
  get_parameter("docking_frame", docking_frame);

  auto transformed_msg = transform_pointcloud(msg, robot_frame);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_msg, *cloud);

  // 降采样实时点云
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  double voxel_size;
  get_parameter("voxel_size", voxel_size);
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.filter(*cloud);

  // 三方向滤波（X、Y、Z 轴）
  double x_min, x_max, y_min, y_max, z_min, z_max;
  get_parameter("x_min", x_min);
  get_parameter("x_max", x_max);
  get_parameter("y_min", y_min);
  get_parameter("y_max", y_max);
  get_parameter("z_min", z_min);
  get_parameter("z_max", z_max);

  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_min, x_max);
  pass_x.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_min, y_max);
  pass_y.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_min, z_max);
  pass_z.filter(*cloud);

  RCLCPP_INFO(this->get_logger(), "Filtered point cloud: %ld points remaining.", cloud->size());

  if (cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud is empty after filtering! Skipping ICP.");
    return;
  }

  // 将滤波后的点云（ICP 匹配前的点云）转换为 ROS 消息并发布
  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*cloud, filtered_msg);
  filtered_msg.header.frame_id = robot_frame;
  filtered_msg.header.stamp = msg->header.stamp;
  pub_transformed_cloud_->publish(filtered_msg);
  RCLCPP_INFO(this->get_logger(), "Published filtered point cloud before ICP.");

  icp_.setInputSource(cloud);

  // 从参数中获取初始位姿猜测
  double initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw;
  get_parameter("initial_x", initial_x);
  get_parameter("initial_y", initial_y);
  get_parameter("initial_z", initial_z);
  get_parameter("initial_roll", initial_roll);
  get_parameter("initial_pitch", initial_pitch);
  get_parameter("initial_yaw", initial_yaw);

  // 构造初始变换矩阵
  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  // 设置平移
  initial_guess(0, 3) = initial_x;
  initial_guess(1, 3) = initial_y;
  initial_guess(2, 3) = initial_z;
  // 设置旋转（欧拉角转旋转矩阵，单位：度 -> 弧度）
  Eigen::AngleAxisf rollAngle(initial_roll * M_PI / 180.0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(initial_pitch * M_PI / 180.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(initial_yaw * M_PI / 180.0, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f initial_rotation = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();  // 修改：使用 initial_rotation
  initial_guess.block<3, 3>(0, 0) = initial_rotation;

  // 打印初始位姿以供调试
  RCLCPP_INFO(this->get_logger(),
              "Using initial guess: x=%.3f, y=%.3f, z=%.3f, roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg",
              initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw);

  if (!first_scan_) {
    // 可添加前一次位姿作为初始猜测（当前未实现）
  }

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp_.align(aligned_cloud, initial_guess);

  if (!icp_.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "ICP did not converge for docking pose estimation.");
    return;
  }

  Eigen::Matrix4f transform = icp_.getFinalTransformation();

  Eigen::Matrix3f final_rotation = transform.block<3, 3>(0, 0);  // 修改：使用 final_rotation
  Eigen::Quaternionf quat(final_rotation);
  Eigen::Vector3f translation = transform.block<3, 1>(0, 3);

  Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  double roll = euler[0] * 180.0 / M_PI;
  double pitch = euler[1] * 180.0 / M_PI;
  double yaw = euler[2] * 180.0 / M_PI;

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = robot_frame;
  pose_msg.pose.position.x = translation.x();
  pose_msg.pose.position.y = translation.y();
  pose_msg.pose.position.z = translation.z();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();
  pub_pose_->publish(pose_msg);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg->header.stamp;
  tf_msg.header.frame_id = robot_frame;
  tf_msg.child_frame_id = docking_frame;
  tf_msg.transform.translation.x = translation.x();
  tf_msg.transform.translation.y = translation.y();
  tf_msg.transform.translation.z = translation.z();
  tf_msg.transform.rotation.x = quat.x();
  tf_msg.transform.rotation.y = quat.y();
  tf_msg.transform.rotation.z = quat.z();
  tf_msg.transform.rotation.w = quat.w();
  tf_broadcaster_->sendTransform(tf_msg);

  RCLCPP_INFO(this->get_logger(),
              "Estimated docking target pose relative to AMR: x=%.3f, y=%.3f, z=%.3f, roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg",
              translation.x(), translation.y(), translation.z(),
              roll, pitch, yaw);

  first_scan_ = false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DockingPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}