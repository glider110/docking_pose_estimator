#ifndef DOCKING_POSE_NODE_HPP_
#define DOCKING_POSE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <std_srvs/srv/set_bool.hpp> // 新增：服务支持动态加载

class DockingPoseNode : public rclcpp::Node {
public:
  DockingPoseNode();

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_template_cloud();
  sensor_msgs::msg::PointCloud2 transform_pointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr &cloud,
      const std::string &target_frame);
  void load_docking_template(const std::string &pcd_file);
  void load_template_service(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response); // 新增：服务回调

  // 成员变量
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_template_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_transformed_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_cloud_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_load_template_; // 新增：服务
  rclcpp::TimerBase::SharedPtr template_publish_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr docking_template_;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
  bool first_scan_;
  Eigen::Matrix4f last_transform_; // 新增：存储上一次配准变换
};

#endif // DOCKING_POSE_NODE_HPP_