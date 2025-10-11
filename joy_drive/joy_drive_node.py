import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyDrive(Node):
    def __init__(self):
        super().__init__('joy_drive')

        # Declare parameters with default values
        self.declare_parameter('forward_button', 0)
        self.declare_parameter('backward_button', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('linear_ramp_rate', 0.1)   # m/s²
        self.declare_parameter('angular_ramp_rate', 0.3)  # rad/s²

        # Get parameters
        self.forward_button = self.get_parameter('forward_button').get_parameter_value().integer_value
        self.backward_button = self.get_parameter('backward_button').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.turbo_button = self.get_parameter('turbo_button').get_parameter_value().integer_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').get_parameter_value().double_value
        self.linear_ramp_rate = self.get_parameter('linear_ramp_rate').get_parameter_value().double_value
        self.angular_ramp_rate = self.get_parameter('angular_ramp_rate').get_parameter_value().double_value

        # Log configuration
        self.get_logger().info("joy_drive node started with parameters:")
        self.get_logger().info(f"  forward_button: {self.forward_button}")
        self.get_logger().info(f"  backward_button: {self.backward_button}")
        self.get_logger().info(f"  angular_axis: {self.angular_axis}")
        self.get_logger().info(f"  turbo_button: {self.turbo_button}")
        self.get_logger().info(f"  linear_speed: {self.linear_speed}")
        self.get_logger().info(f"  angular_speed: {self.angular_speed}")
        self.get_logger().info(f"  turbo_multiplier: {self.turbo_multiplier}")
        self.get_logger().info(f"  linear_ramp_rate: {self.linear_ramp_rate}")
        self.get_logger().info(f"  angular_ramp_rate: {self.angular_ramp_rate}")

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # Target and current velocities
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Timer setup
        self.dt = 0.05  # seconds between updates (20 Hz)
        self.create_timer(self.dt, self.update_velocity)

    def joy_callback(self, msg: Joy):
        turbo_pressed = msg.buttons[self.turbo_button]
        linear_speed = self.linear_speed * self.turbo_multiplier if turbo_pressed else self.linear_speed
        angular_speed = self.angular_speed * self.turbo_multiplier if turbo_pressed else self.angular_speed

        if msg.buttons[self.forward_button]:
            self.target_linear_x = linear_speed
        elif msg.buttons[self.backward_button]:
            self.target_linear_x = -linear_speed
        else:
            self.target_linear_x = 0.0

        self.target_angular_z = msg.axes[self.angular_axis] * angular_speed

    def update_velocity(self):
        # Smoothly move current velocities towards target velocities
        self.current_linear_x = self._smooth_step(
            self.current_linear_x,
            self.target_linear_x,
            self.linear_ramp_rate,
            self.dt,
        )
        self.current_angular_z = self._smooth_step(
            self.current_angular_z,
            self.target_angular_z,
            self.angular_ramp_rate,
            self.dt,
        )

        # Stop publishing when both velocities are effectively zero
        if abs(self.current_linear_x) < 1e-3 and abs(self.current_angular_z) < 1e-3:
            return

        # Publish only when movement is present
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.angular.z = self.current_angular_z
        self.publisher_.publish(twist)

    def _smooth_step(self, current, target, rate, dt):
        # rate: maximum change per second (m/s² or rad/s²)
        max_delta = rate * dt
        if abs(target - current) < max_delta:
            return target
        return current + max_delta if target > current else current - max_delta


def main():
    rclpy.init()
    node = JoyDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
