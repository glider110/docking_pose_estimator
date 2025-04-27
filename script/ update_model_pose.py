#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState  # 导入 EntityState 消息类型
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
import math
import sys

class ModelPoseUpdater(Node):
    def __init__(self):
        super().__init__('model_pose_updater')
        self.client = self.create_client(SetEntityState, '/set_entity_state')

        # 等待服务可用
        timeout = 60.0
        start_time = self.get_clock().now()
        while not self.client.wait_for_service(timeout_sec=1.0):
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error(f'Service /set_entity_state not available after {timeout} seconds. Exiting.')
                sys.exit(1)
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Waiting for /set_entity_state service...')

    def set_model_pose(self, model_name, x, y, z, yaw_deg):
        # 将角度从度转换为弧度
        yaw_rad = math.radians(yaw_deg)

        # 计算四元数（仅绕 Z 轴旋转，yaw）
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        # 创建 Pose 消息
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 创建 Twist 消息（默认静止）
        twist = Twist()
        twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        # 创建 EntityState 消息
        entity_state = EntityState()
        entity_state.name = model_name
        entity_state.pose = pose
        entity_state.twist = twist
        entity_state.reference_frame = 'world'

        # 创建服务请求
        request = SetEntityState.Request()
        request.state = entity_state  # 将 entity_state 赋值给 request.state

        # 调用服务
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully updated pose of {model_name} to (x={x}, y={y}, z={z}, yaw={yaw_deg} deg)')
            else:
                self.get_logger().error(f'Failed to update pose: {response.status_message}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    # 创建节点并设置位姿
    node = ModelPoseUpdater()
    # node.set_model_pose('cart_model2_no_whell', 4.0, 0.0, 0.0, 180.0)
    # node.set_model_pose('cart_model2_no_whell', 3.5 -0.262, 0.0, 180.0)
    # node.set_model_pose('waffle', 3.0, 0.0, 0.0, 180.0)
    node.set_model_pose('waffle',3.5 ,-0.262, 0.0, 0.0)

    # 销毁节点并关闭 ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()