import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import math
import time
from simple_navigator import SimpleNavigator

class DockPoseReceiver(Node):
    def __init__(self):
        super().__init__('move_to_dock_pose')
        self.sub_pose = self.create_subscription(PoseStamped, '/docking_pose', self.pose_callback, 10)
        self.sub_model = self.create_subscription(ModelStates, '/model_states', self.model_callback, 10)

        self.relative_pose = None
        self.robot_pose = None
        self.robot_name = 'waffle'  # 修改为你的模型名
        self.reached_deployment = False
        self.reached_dock = False
        self.navigation_in_progress = False
        self.poses_computed = False  # 是否已计算位姿
        self.nav_start_time = None
        self.timeout = 15.0  # 导航超时（秒）
        # 存储部署点和对接点位姿
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.dock_x = None
        self.dock_y = None
        self.dock_yaw = None

    def pose_callback(self, msg):
        # 仅在位姿未计算时处理 /docking_pose
        if self.poses_computed:
            self.get_logger().debug("位姿已计算，忽略新的 /docking_pose 数据")
            return

        self.get_logger().info("接收到相对目标位姿")
        self.relative_pose = msg.pose
        rel_x = self.relative_pose.position.x
        rel_y = self.relative_pose.position.y
        rel_z = self.relative_pose.position.z
        rel_q = self.relative_pose.orientation
        try:
            rel_yaw = math.atan2(
                2.0 * (rel_q.w * rel_q.z + rel_q.x * rel_q.y),
                1.0 - 2.0 * (rel_q.y ** 2 + rel_q.z ** 2)
            )
            self.get_logger().info(
                f"相对目标位姿: x={rel_x:.2f} m, y={rel_y:.2f} m, z={rel_z:.2f} m, "
                f"yaw={math.degrees(rel_yaw):.2f}°"
            )
        except Exception as e:
            self.get_logger().warn(f"无法计算相对目标偏航角: {e}")
        self.try_navigate()

    def model_callback(self, msg):
        # 实时更新机器人位姿
        try:
            idx = msg.name.index(self.robot_name)
            self.robot_pose = msg.pose[idx]
            self.get_logger().debug(f"接收到机器人 {self.robot_name} 位姿")
        except ValueError:
            self.get_logger().warn(f"未找到模型 {self.robot_name}")
        self.try_navigate()

    def get_current_robot_pose(self):
        """获取并返回当前机器人位姿 (x, y, yaw)，使用最新 self.robot_pose"""
        if self.robot_pose is None:
            self.get_logger().warn("机器人位姿不可用")
            return None, None, None
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        q = self.robot_pose.orientation
        try:
            robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
            )
        except Exception as e:
            self.get_logger().warn(f"无法计算机器人偏航角: {e}")
            robot_yaw = 0.0  # 默认值
        return rx, ry, robot_yaw

    def try_navigate(self):
        if self.navigation_in_progress:
            self.get_logger().debug("导航进行中，跳过")
            return
        if self.relative_pose is None or self.robot_pose is None:
            self.get_logger().debug("等待相对位姿或机器人位姿")
            return
        if self.reached_dock:
            self.get_logger().info("已到达对接点，导航结束")
            self.destroy_node()
            return

        self.navigation_in_progress = True

        # 仅首次计算部署点和对接点位姿
        if not self.poses_computed:
            # 获取当前机器人位姿
            rx, ry, robot_yaw = self.get_current_robot_pose()
            if rx is None:
                self.get_logger().warn("无法获取机器人位姿，延迟位姿计算")
                self.navigation_in_progress = False
                return

            # 打印当前机器人坐标
            self.get_logger().info(
                f"当前机器人坐标: x={rx:.2f} m, y={ry:.2f} m, "
                f"yaw={math.degrees(robot_yaw):.2f}°"
            )

            # 相对目标坐标
            dx = self.relative_pose.position.x
            dy = self.relative_pose.position.y
            dq = self.relative_pose.orientation
            try:
                relative_yaw = math.atan2(
                    2.0 * (dq.w * dq.z + dq.x * dq.y),
                    1.0 - 2.0 * (dq.y ** 2 + dq.z ** 2)
                )
            except Exception as e:
                self.get_logger().warn(f"无法计算相对目标偏航角: {e}")
                self.navigation_in_progress = False
                return

            # 坐标变换：部署点
            self.target_x = rx + dx * math.cos(robot_yaw) - dy * math.sin(robot_yaw)
            self.target_y = ry + dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
            self.target_yaw = self.normalize_angle(robot_yaw + relative_yaw)

            self.get_logger().info(
                f"转换后目标位姿（部署点）: x={self.target_x:.2f} m, y={self.target_y:.2f} m, "
                f"yaw={math.degrees(self.target_yaw):.2f}°"
            )

            # 实际对接目标位姿（正前方 0.5 m）
            self.dock_x = self.target_x + 0.5 * math.cos(self.target_yaw)
            self.dock_y = self.target_y + 0.5 * math.sin(self.target_yaw)
            self.dock_yaw = self.target_yaw
            self.get_logger().info(
                f"实际对接目标位姿: x={self.dock_x:.2f} m, y={self.dock_y:.2f} m, "
                f"yaw={math.degrees(self.dock_yaw):.2f}°"
            )

            self.poses_computed = True  # 标记位姿已计算

        # 第一段导航：到达部署点
        if not self.reached_deployment:
            self.get_logger().info("启动第一段导航：前往部署点")
            navigator = SimpleNavigator()
            navigator.target_x = self.target_x
            navigator.target_y = self.target_y
            navigator.target_yaw = self.target_yaw
            self.nav_start_time = time.time()

            try:
                # 运行导航直到超时
                while time.time() - self.nav_start_time < self.timeout:
                    rclpy.spin_once(navigator, timeout_sec=0.1)
                self.reached_deployment = True
                # 打印最新机器人坐标
                rx, ry, robot_yaw = self.get_current_robot_pose()
            except Exception as e:
                self.get_logger().error(f"第一段导航失败: {e}")
            finally:
                navigator.destroy_node()
                self.navigation_in_progress = False

        # 第二段导航：直线行驶到对接点
        elif self.reached_deployment and not self.reached_dock:
            self.get_logger().info("启动第二段导航：直线行驶到对接点")
            navigator = SimpleNavigator()
            navigator.target_x = self.dock_x
            navigator.target_y = self.dock_y
            navigator.target_yaw = self.dock_yaw
            self.nav_start_time = time.time()

            try:
                # 运行导航直到超时
                while time.time() - self.nav_start_time < self.timeout:
                    rclpy.spin_once(navigator, timeout_sec=0.1)
                self.reached_dock = True
                # 打印最新机器人坐标
                rx, ry, robot_yaw = self.get_current_robot_pose()
            except Exception as e:
                self.get_logger().error(f"第二段导航失败: {e}")
            finally:
                navigator.destroy_node()
                self.navigation_in_progress = False
                if self.reached_dock:
                    self.get_logger().info("导航完成，关闭")
                    self.destroy_node()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = DockPoseReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()