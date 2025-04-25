# save_pointcloud.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import os

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_map',
            self.listener_callback,
            10)
        self.count = 0

    def listener_callback(self, msg):
        self.get_logger().info('Receiving point cloud')
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.save_to_pcd(points)
        self.count += 1
        if self.count >= 1:  # 保存一次后退出
            rclpy.shutdown()

    def save_to_pcd(self, points):
        output_file = os.path.expanduser("./rtabmap_optimized1.pcd")
        with open(output_file, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for p in points:
                f.write(f"{p[0]:.8f} {p[1]:.8f} {p[2]:.8f}\n")
        self.get_logger().info(f"Saved {len(points)} points to {output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()