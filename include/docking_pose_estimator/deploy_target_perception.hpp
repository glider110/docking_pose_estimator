#ifndef DEPLOY_TARGET_PERCEPTION_HPP_
#define DEPLOY_TARGET_PERCEPTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

class DeployTargetPerception : public rclcpp::Node {
public:
  DeployTargetPerception();

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void save_command_callback(const std_msgs::msg::String::SharedPtr msg);
  sensor_msgs::msg::PointCloud2 transform_pointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr &cloud,
      const std::string &target_frame);
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_pointcloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  Eigen::Matrix4f get_initial_guess();

  // 成员变量
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_command_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_deployed_template_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_source_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_template_cloud_; // 发布裁剪模板
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_; // 原始模板点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_template_cloud_; // 裁剪后的模板点云（用于配准）
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_filtered_cloud_; // 最新滤波后的实时点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_deployed_template_; // 最新转换后的模板点云
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  std::string camera_frame_;
  std::string robot_frame_;
  std::string save_path_;
  double voxel_size_;
  bool use_ndt_;
};

#endif // DEPLOY_TARGET_PERCEPTION_HPP_