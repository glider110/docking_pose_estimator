import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose')
        # 声明参数
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 3.0)
        self.declare_parameter('target_yaw_deg', 90.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.1)  # 降低角速度
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.1)  # 增大角度容差

        # 初始化发布者和订阅者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz 控制循环

        # 获取参数
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
        self.target_yaw = math.radians(self.get_parameter('target_yaw_deg').get_parameter_value().double_value)
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value

        # 当前位姿
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.reached_position = False
        self.reached_pose = False
        # 平滑系数
        self.alpha = 0.95  # 添加里程计平滑

    def odom_callback(self, msg):
        # 平滑更新当前位姿
        try:
            self.current_x = self.alpha * self.current_x + (1 - self.alpha) * msg.pose.pose.position.x
            self.current_y = self.alpha * self.current_y + (1 - self.alpha) * msg.pose.pose.position.y
            orientation = msg.pose.pose.orientation
            new_yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
            )
            self.current_yaw = self.alpha * self.current_yaw + (1 - self.alpha) * new_yaw
        except Exception as e:
            self.get_logger().warn(f"无效的里程计数据: {e}")

    def control_loop(self):
        if self.reached_pose:
            return

        cmd_vel = Twist()
        # 初始化日志变量
        distance = 0.0
        angle_error = 0.0
        yaw_error = 0.0

        if not self.reached_position:
            # 计算位置误差和连线方向
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if distance > self.distance_tolerance:
                if abs(angle_error) > self.angle_tolerance:
                    # 动态调整角速度，朝向连线方向
                    cmd_vel.angular.z = self.angular_speed * min(1.0, abs(angle_error) / 0.3) * (1 if angle_error > 0 else -1)
                else:
                    # 沿连线方向直线移动
                    cmd_vel.linear.x = self.linear_speed * min(1.0, distance / 0.5)
            else:
                self.reached_position = True
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info(f'到达位置: ({self.target_x}, {self.target_y})')
        else:
            # 调整到目标朝向
            yaw_error = self.target_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            if abs(yaw_error) > self.angle_tolerance:
                # 动态调整角速度，朝向 target_yaw
                cmd_vel.angular.z = self.angular_speed * min(1.0, abs(yaw_error) / 0.3) * (1 if yaw_error > 0 else -1)
            else:
                self.reached_pose = True
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info(f'到达目标位姿: ({self.target_x}, {self.target_y}, yaw={math.degrees(self.target_yaw)} 度)')
                # 多次发布零速度确保停止
                for _ in range(5):
                    self.cmd_vel_pub.publish(Twist())
                self.timer.cancel()
                self.destroy_node()

        self.cmd_vel_pub.publish(cmd_vel)
        # 日志记录
        self.get_logger().info(
            f'距离: {distance:.3f}, 角度误差: {angle_error:.3f}, 偏航误差: {yaw_error:.3f}, '
            f'线速度: {cmd_vel.linear.x:.3f}, 角速度: {cmd_vel.angular.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()