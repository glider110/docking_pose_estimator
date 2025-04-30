import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import math
from simple_navigator import SimpleNavigator

class DockPoseReceiver(Node):
    def __init__(self):
        super().__init__('move_to_dock_pose')
        self.sub_pose = self.create_subscription(PoseStamped, '/docking_pose', self.pose_callback, 10)
        self.sub_model = self.create_subscription(ModelStates, '/model_states', self.model_callback, 10)

        self.relative_pose = None
        self.robot_pose = None
        self.robot_name = 'waffle'  # 修改为你的模型名

    def pose_callback(self, msg):
        self.get_logger().info("接收到相对目标位姿")
        self.relative_pose = msg.pose
        self.try_navigate()

    def model_callback(self, msg):
        try:
            idx = msg.name.index(self.robot_name)
            self.robot_pose = msg.pose[idx]
        except ValueError:
            self.get_logger().warn(f"未找到模型 {self.robot_name}")
        self.try_navigate()

    def try_navigate(self):
        if self.relative_pose is None or self.robot_pose is None:
            return  # 等待两个消息都到达

        # 机器人当前位置与朝向
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        q = self.robot_pose.orientation
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        # 相对目标坐标
        dx = self.relative_pose.position.x
        dy = self.relative_pose.position.y
        dq = self.relative_pose.orientation
        relative_yaw = math.atan2(2.0 * (dq.w * dq.z + dq.x * dq.y), 1.0 - 2.0 * (dq.y ** 2 + dq.z ** 2))

        # 坐标变换：从相对坐标转到世界坐标
        target_x = rx + dx * math.cos(robot_yaw) - dy * math.sin(robot_yaw)
        target_y = ry + dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        target_yaw = self.normalize_angle(robot_yaw + relative_yaw)

        self.get_logger().info(f'转换后目标位姿: x={target_x:.2f}, y={target_y:.2f}, yaw={math.degrees(target_yaw):.2f}°')

        # 启动导航器
        navigator = SimpleNavigator()
        navigator.target_x = target_x
        navigator.target_y = target_y
        navigator.target_yaw = target_yaw

        self.get_logger().info("启动导航器")
        rclpy.spin(navigator)

        self.get_logger().info("导航完成，关闭")
        self.destroy_node()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseReceiver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
