import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg
import numpy as np

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')

        # 参数化：PCD 文件路径、frame_id、发布频率等
        self.declare_parameter('pcd_file', 'rtabmap_optimized.pcd')
        # self.declare_parameter('frame_id', 'map')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('topic', '/map_pcd_cloud')
        self.declare_parameter('rate', 1.0)

        # 获取参数
        self.pcd_file = self.get_parameter('pcd_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        # 创建点云发布者
        self.publisher = self.create_publisher(PointCloud2, self.topic, 10)

        # 加载 PCD 文件
        pcd = o3d.io.read_point_cloud(self.pcd_file)
        points = np.asarray(pcd.points)

        self.get_logger().info(f'PCD file {self.pcd_file} loaded.')

        # 将 PCD 数据转换为 ROS 消息
        cloud_msg = pc2.create_cloud_xyz32(std_msgs.msg.Header(), points)
        cloud_msg.header.frame_id = self.frame_id
        self.cloud_msg = cloud_msg

        # 定时发布点云
        self.timer = self.create_timer(1.0 / self.rate, self.publish_point_cloud)

    def publish_point_cloud(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
