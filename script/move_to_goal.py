import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz control loop
        # Target pose
        self.target_x = 2.0
        self.target_y = 1.0
        self.target_yaw = math.radians(0.0)
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        # Control parameters
        self.linear_speed = 0.1  # Reduced linear speed
        self.angular_speed = 0.2  # Reduced angular speed
        self.distance_tolerance = 0.01  # Increased distance tolerance
        self.angle_tolerance = 0.01  # Increased angle tolerance (~17°)
        # State flags
        self.reached_position = False
        self.reached_pose = False
        # Smoothing factor
        self.alpha = 0.95  # Strong odometry smoothing

    def odom_callback(self, msg):
        # Smooth current position and orientation
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
            self.get_logger().warn(f"Invalid odometry data: {e}")

    def control_loop(self):
        if self.reached_pose:
            return

        cmd_vel = Twist()
        # Initialize variables to avoid UnboundLocalError
        distance = 0.0
        angle_error = 0.0
        yaw_error = 0.0

        if not self.reached_position:
            # Calculate position error
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if distance > self.distance_tolerance:
                if abs(angle_error) > self.angle_tolerance:
                    # Dynamic angular speed adjustment
                    cmd_vel.angular.z = self.angular_speed * min(1.0, abs(angle_error) / 0.5) * (1 if angle_error > 0 else -1)
                else:
                    # Dynamic linear speed adjustment
                    cmd_vel.linear.x = self.linear_speed * min(1.0, distance / 0.5)
            else:
                self.reached_position = True
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info(f'Reached position: ({self.target_x}, {self.target_y})')
        else:
            # Adjust orientation
            yaw_error = self.target_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            if abs(yaw_error) > self.angle_tolerance:
                # Dynamic angular speed adjustment for yaw
                cmd_vel.angular.z = self.angular_speed * min(1.0, abs(yaw_error) / 0.3) * (1 if yaw_error > 0 else -1)
            else:
                self.reached_pose = True
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info(f'Reached target pose: ({self.target_x}, {self.target_y}, yaw={math.degrees(self.target_yaw)} deg)')
                # Send zero velocity multiple times to ensure stopping
                for _ in range(5):
                    self.cmd_vel_pub.publish(Twist())
                self.timer.cancel()
                self.destroy_node()

        self.cmd_vel_pub.publish(cmd_vel)
        # Log relevant variables
        self.get_logger().info(
            f'Distance: {distance:.3f}, Angle error: {angle_error:.3f}, Yaw error: {yaw_error:.3f}, '
            f'Linear speed: {cmd_vel.linear.x:.3f}, Angular speed: {cmd_vel.angular.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()