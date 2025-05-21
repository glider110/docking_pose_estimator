import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # 导航目标参数
        self.declare_parameter('target_x', 3.5)
        self.declare_parameter('target_y', -0.2)
        self.declare_parameter('target_yaw_deg', 0.0)
        self.declare_parameter('robot_model_name', 'waffle')  # 根据你模型名修改

        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
        self.target_yaw = math.radians(self.get_parameter('target_yaw_deg').get_parameter_value().double_value)
        self.model_name = self.get_parameter('robot_model_name').get_parameter_value().string_value

        # 导航参数
        self.linear_speed = 0.1
        self.angular_speed = 0.1
        self.distance_tolerance = 0.03
        self.angle_tolerance = math.radians(0.5)  # 约 ±5°

        # 当前位姿
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.reached_position = False
        self.reached_yaw = False

        self.use_gazebo_pose = True

        # ROS 订阅/发布器
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.model_sub = self.create_subscription(ModelStates, '/model_states', self.model_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        if not self.use_gazebo_pose:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            self.current_x = pos.x
            self.current_y = pos.y
            self.current_yaw = math.atan2(
                2.0 * (ori.w * ori.z + ori.x * ori.y),
                1.0 - 2.0 * (ori.y ** 2 + ori.z ** 2)
            )

    def model_callback(self, msg):
        if self.use_gazebo_pose:
            try:
                if self.model_name not in msg.name:
                    self.get_logger().warn(f"模型名 {self.model_name} 不在 Gazebo 模型列表中: {msg.name}")
                    return

                idx = msg.name.index(self.model_name)
                pose = msg.pose[idx]
                ori = pose.orientation
                self.current_x = pose.position.x
                self.current_y = pose.position.y
                self.current_yaw = math.atan2(
                    2.0 * (ori.w * ori.z + ori.x * ori.y),
                    1.0 - 2.0 * (ori.y ** 2 + ori.z ** 2)
                )
            except Exception as e:
                self.get_logger().error(f"处理模型状态失败: {e}")

    def control_loop(self):
        cmd = Twist()

        if not self.reached_position:
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)

            self.get_logger().info(
                f"[位置控制] 当前:({self.current_x:.2f}, {self.current_y:.2f}) 目标:({self.target_x:.2f}, {self.target_y:.2f}) "
                f"距离:{distance:.3f}, 角度误差:{math.degrees(angle_error):.2f}°"
            )

            if distance > self.distance_tolerance:
                if abs(angle_error) > self.angle_tolerance:
                    cmd.angular.z = self.angular_speed * (1 if angle_error > 0 else -1)
                else:
                    cmd.linear.x = self.linear_speed
            else:
                self.reached_position = True
                self.get_logger().info(f"✅ 已到达目标位置: ({self.target_x:.2f}, {self.target_y:.2f})")

        elif not self.reached_yaw:
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            self.get_logger().info(
                f"[朝向控制] 当前Yaw: {math.degrees(self.current_yaw):.2f}° 目标Yaw: {math.degrees(self.target_yaw):.2f}° "
                f"误差: {math.degrees(yaw_error):.2f}°"
            )

            if abs(yaw_error) > self.angle_tolerance:
                cmd.angular.z = self.angular_speed * (1 if yaw_error > 0 else -1)
            else:
                self.reached_yaw = True
                self.get_logger().info(f"✅ 已到达目标朝向: {math.degrees(self.target_yaw):.2f}°")

                # 强制多次发布0速度确保静止
                for _ in range(5):
                    self.cmd_pub.publish(Twist())
                    rclpy.spin_once(self, timeout_sec=0.05)

                # 打印最终到达的位姿
                self.get_logger().info(
                    f"最终到达的位姿：x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.2f}°"
                )

                self.get_logger().info("✅ 机器人已完全停止")
                self.timer.cancel()
                self.destroy_node()
                return

        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
