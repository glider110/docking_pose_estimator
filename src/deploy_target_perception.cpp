#include "docking_pose_estimator/deploy_target_perception.hpp"
#include <std_msgs/msg/string.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <chrono>
#include <cmath>

DeployTargetPerception::DeployTargetPerception() : Node("deploy_target_perception") {
  // 声明参数
  declare_parameter("pcd_file", "/home/std/workspace/test_pkg_ws/rtabmap_optimized.pcd");
  declare_parameter("pointcloud_topic", "/camera/points");
  declare_parameter("deployed_template_cloud_topic", "/deployed_template_cloud");
  declare_parameter("filtered_source_cloud_topic", "/filtered_source_cloud");
  declare_parameter("filtered_template_cloud_topic", "/filtered_template_cloud");
  declare_parameter("save_command_topic", "/save_pointcloud_command");
  declare_parameter("save_path", "./");
  declare_parameter("voxel_size", 0.05);
  declare_parameter("camera_frame", "camera_rgb_frame");
  declare_parameter("robot_frame", "base_footprint");
  // 实时点云滤波参数
  declare_parameter("x_min", 0.0);
  declare_parameter("x_max", 3.0);
  declare_parameter("y_min", -1.0);
  declare_parameter("y_max", 1.0);
  declare_parameter("z_min", 0.0);
  declare_parameter("z_max", 2.0);
  // 模板点云滤波参数
  declare_parameter("template_x_min", 0.0);
  declare_parameter("template_x_max", 3.0);
  declare_parameter("template_y_min", -1.0);
  declare_parameter("template_y_max", 1.0);
  declare_parameter("template_z_min", 0.0);
  declare_parameter("template_z_max", 2.0);
  // 初始位姿
  declare_parameter("initial_x", 0.0);
  declare_parameter("initial_y", 0.0);
  declare_parameter("initial_z", 0.0);
  declare_parameter("initial_roll", 0.0);
  declare_parameter("initial_pitch", 0.0);
  declare_parameter("initial_yaw", 0.0);
  declare_parameter("use_ndt", false);

  // 获取参数
  std::string pcd_file, pointcloud_topic, deployed_template_cloud_topic, filtered_source_cloud_topic, filtered_template_cloud_topic, save_command_topic, save_path;
  get_parameter("pcd_file", pcd_file);
  get_parameter("pointcloud_topic", pointcloud_topic);
  get_parameter("deployed_template_cloud_topic", deployed_template_cloud_topic);
  get_parameter("filtered_source_cloud_topic", filtered_source_cloud_topic);
  get_parameter("filtered_template_cloud_topic", filtered_template_cloud_topic);
  get_parameter("save_command_topic", save_command_topic);
  get_parameter("save_path", save_path_);
  get_parameter("voxel_size", voxel_size_);
  get_parameter("camera_frame", camera_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("use_ndt", use_ndt_);

  // 检查话题名称
  if (deployed_template_cloud_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Deployed template cloud topic is empty! Using default: /deployed_template_cloud");
    deployed_template_cloud_topic = "/deployed_template_cloud";
  }
  if (filtered_source_cloud_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Filtered source cloud topic is empty! Using default: /filtered_source_cloud");
    filtered_source_cloud_topic = "/filtered_source_cloud";
  }
  if (filtered_template_cloud_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Filtered template cloud topic is empty! Using default: /filtered_template_cloud");
    filtered_template_cloud_topic = "/filtered_template_cloud";
  }
  if (save_command_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Save command topic is empty! Using default: /save_pointcloud_command");
    save_command_topic = "/save_pointcloud_command";
  }

  // 确保保存路径以斜杠结尾
  if (!save_path_.empty() && save_path_.back() != '/') {
    save_path_ += '/';
  }

  // 加载模板点云
  template_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *template_cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load template point cloud from: %s", pcd_file.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded template point cloud with %ld points.", template_cloud_->size());

  // 降采样模板点云
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(template_cloud_);
  voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  voxel.filter(*template_cloud_);
  RCLCPP_INFO(this->get_logger(), "Template point cloud after voxel filter: %ld points.", template_cloud_->size());

  // 对模板点云进行直通滤波
  filtered_template_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double template_x_min, template_x_max, template_y_min, template_y_max, template_z_min, template_z_max;
  get_parameter("template_x_min", template_x_min);
  get_parameter("template_x_max", template_x_max);
  get_parameter("template_y_min", template_y_min);
  get_parameter("template_y_max", template_y_max);
  get_parameter("template_z_min", template_z_min);
  get_parameter("template_z_max", template_z_max);

  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(template_cloud_);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(template_x_min, template_x_max);
  pass_x.filter(*filtered_template_cloud_);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(filtered_template_cloud_);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(template_y_min, template_y_max);
  pass_y.filter(*filtered_template_cloud_);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(filtered_template_cloud_);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(template_z_min, template_z_max);
  pass_z.filter(*filtered_template_cloud_);

  RCLCPP_INFO(this->get_logger(), "Template point cloud after pass-through filter: %ld points.", filtered_template_cloud_->size());

  // 设置 ICP 参数（使用裁剪后的模板点云）
  icp_.setMaximumIterations(250);
  icp_.setTransformationEpsilon(1e-6);
  icp_.setMaxCorrespondenceDistance(3.0);
  icp_.setInputTarget(filtered_template_cloud_);

  // 设置 NDT 参数（如果启用，使用裁剪后的模板点云）
  if (use_ndt_) {
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.1);
    ndt_.setResolution(1.0);
    ndt_.setMaximumIterations(35);
    ndt_.setInputTarget(filtered_template_cloud_);
  }

  // 初始化 TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化订阅和发布
  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&DeployTargetPerception::pointcloud_callback, this, std::placeholders::_1));
  pub_deployed_template_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      deployed_template_cloud_topic, 10);
  pub_filtered_source_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_source_cloud_topic, 10);
  pub_filtered_template_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_template_cloud_topic, 10);
  sub_save_command_ = create_subscription<std_msgs::msg::String>(
      save_command_topic, 10,
      std::bind(&DeployTargetPerception::save_command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "DeployTargetPerception node initialized.");
}

void DeployTargetPerception::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // 转换实拍点云到机器人坐标系
  sensor_msgs::msg::PointCloud2 transformed_msg = transform_pointcloud(msg, robot_frame_);
  if (transformed_msg.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Transformed point cloud is empty. Skipping processing.");
    return;
  }

  // 转换为 PCL 点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_msg, *cloud);

  // 点云预处理（降采样 + 直通滤波）
  cloud = preprocess_pointcloud(cloud);

  if (cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud is empty after preprocessing! Skipping registration.");
    return;
  }

  // 发布滤波后的实拍点云
  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*cloud, filtered_msg);
  filtered_msg.header.frame_id = robot_frame_;
  filtered_msg.header.stamp = msg->header.stamp;
  pub_filtered_source_cloud_->publish(filtered_msg);
  RCLCPP_INFO(this->get_logger(), "Published filtered source point cloud.");

  // 获取初始位姿猜测
  Eigen::Matrix4f initial_guess = get_initial_guess();

  // 点云配准
  Eigen::Matrix4f transform;
  if (use_ndt_) {
    // NDT 粗配准（使用裁剪后的模板点云）
    ndt_.setInputSource(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_output(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_.align(*ndt_output, initial_guess);
    if (!ndt_.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "NDT did not converge.");
      return;
    }
    initial_guess = ndt_.getFinalTransformation();
  }

  // ICP 精配准（使用裁剪后的模板点云）
  icp_.setInputSource(cloud);
  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp_.align(aligned_cloud, initial_guess);
  if (!icp_.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
    return;
  }
  transform = icp_.getFinalTransformation();

  // 转换原始模板点云到实拍点云坐标系
  Eigen::Matrix4f inverse_transform = transform.inverse();
  pcl::PointCloud<pcl::PointXYZ>::Ptr deployed_template(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*template_cloud_, *deployed_template, inverse_transform); // 使用原始模板

  // 转换裁剪后的模板点云到实拍点云坐标系
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_filtered_template(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*filtered_template_cloud_, *transformed_filtered_template, inverse_transform);

  // 发布转换后的裁剪模板点云
  sensor_msgs::msg::PointCloud2 template_msg;
  pcl::toROSMsg(*transformed_filtered_template, template_msg);
  template_msg.header.frame_id = robot_frame_;
  template_msg.header.stamp = msg->header.stamp;
  pub_filtered_template_cloud_->publish(template_msg);
  RCLCPP_INFO(this->get_logger(), "Published transformed filtered template point cloud.");

  // 发布转换后的原始模板点云
  sensor_msgs::msg::PointCloud2 deployed_template_msg;
  pcl::toROSMsg(*deployed_template, deployed_template_msg);
  deployed_template_msg.header.frame_id = robot_frame_;
  deployed_template_msg.header.stamp = msg->header.stamp;
  pub_deployed_template_cloud_->publish(deployed_template_msg);
  RCLCPP_INFO(this->get_logger(), "Published deployed template point cloud.");

  // 打印变换位姿（从模板点云到实拍点云坐标系）
  Eigen::Matrix3f rotation = inverse_transform.block<3, 3>(0, 0);
  Eigen::Vector3f translation = inverse_transform.block<3, 1>(0, 3);
  Eigen::Quaternionf quat(rotation);
  Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
  double roll = euler[0] * 180.0 / M_PI;
  double pitch = euler[1] * 180.0 / M_PI;
  double yaw = euler[2] * 180.0 / M_PI;

  RCLCPP_INFO(this->get_logger(),
              "Transformed pose (template to source): x=%.3f, y=%.3f, z=%.3f, roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg",
              translation.x(), translation.y(), translation.z(), roll, pitch, yaw);

  // 存储当前点云以供保存
  last_filtered_cloud_ = cloud;
  last_deployed_template_ = deployed_template;
}

void DeployTargetPerception::save_command_callback(const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data != "save") {
    RCLCPP_INFO(this->get_logger(), "Received command '%s', expected 'save'. Ignoring.", msg->data.c_str());
    return;
  }

  if (!last_filtered_cloud_ || !last_deployed_template_) {
    RCLCPP_ERROR(this->get_logger(), "No valid point clouds to save. Ensure point cloud processing has run.");
    return;
  }

  // 生成时间戳
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  // 保存滤波后的实拍点云
  std::string pose_cloud_path = save_path_ + "deploy_pose_cloud_" + std::to_string(timestamp) + ".pcd";
  if (pcl::io::savePCDFileASCII(pose_cloud_path, *last_filtered_cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save filtered point cloud to: %s", pose_cloud_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Saved filtered point cloud to: %s", pose_cloud_path.c_str());
  }

  // 保存转换后的模板点云
  // std::string template_cloud_path = save_path_ + "deploy_template_cloud_" + std::to_string(timestamp) + ".pcd";
  std::string template_cloud_path = save_path_ + "deploy_template_cloud_"  + ".pcd";
  if (pcl::io::savePCDFileASCII(template_cloud_path, *last_deployed_template_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save deployed template point cloud to: %s", template_cloud_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Saved deployed template point cloud to: %s", template_cloud_path.c_str());
  }
}

sensor_msgs::msg::PointCloud2 DeployTargetPerception::transform_pointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud,
    const std::string &target_frame) {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
        target_frame, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform point cloud to %s: %s",
                target_frame.c_str(), ex.what());
    return *cloud;
  }

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*cloud, transformed_cloud, transform);
  transformed_cloud.header.frame_id = target_frame;
  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DeployTargetPerception::preprocess_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  // 降采样
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  voxel.filter(*downsampled);

  // 直通滤波
  double x_min, x_max, y_min, y_max, z_min, z_max;
  get_parameter("x_min", x_min);
  get_parameter("x_max", x_max);
  get_parameter("y_min", y_min);
  get_parameter("y_max", y_max);
  get_parameter("z_min", z_min);
  get_parameter("z_max", z_max);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(downsampled);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_min, x_max);
  pass_x.filter(*filtered);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_min, y_max);
  pass_y.filter(*filtered);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(filtered);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_min, z_max);
  pass_z.filter(*filtered);

  return filtered;
}

Eigen::Matrix4f DeployTargetPerception::get_initial_guess() {
  double initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw;
  get_parameter("initial_x", initial_x);
  get_parameter("initial_y", initial_y);
  get_parameter("initial_z", initial_z);
  get_parameter("initial_roll", initial_roll);
  get_parameter("initial_pitch", initial_pitch);
  get_parameter("initial_yaw", initial_yaw);

  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  initial_guess(0, 3) = initial_x;
  initial_guess(1, 3) = initial_y;
  initial_guess(2, 3) = initial_z;
  Eigen::AngleAxisf rollAngle(initial_roll * M_PI / 180.0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(initial_pitch * M_PI / 180.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(initial_yaw * M_PI / 180.0, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f initial_rotation = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
  initial_guess.block<3, 3>(0, 0) = initial_rotation;

  RCLCPP_INFO(this->get_logger(),
              "Using initial guess: x=%.3f, y=%.3f, z=%.3f, roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg",
              initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw);

  return initial_guess;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeployTargetPerception>());
  rclcpp::shutdown();
  return 0;
}