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
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('ramp_rate', 0.05)  # max delta per update (m/s or rad/s)

        # Get parameters
        self.forward_button = self.get_parameter('forward_button').get_parameter_value().integer_value
        self.backward_button = self.get_parameter('backward_button').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.turbo_button = self.get_parameter('turbo_button').get_parameter_value().integer_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').get_parameter_value().double_value
        self.ramp_rate = self.get_parameter('ramp_rate').get_parameter_value().double_value

        self.get_logger().info("joy_drive node started with parameters:")
        self.get_logger().info(f"  forward_button: {self.forward_button}")
        self.get_logger().info(f"  backward_button: {self.backward_button}")
        self.get_logger().info(f"  angular_axis: {self.angular_axis}")
        self.get_logger().info(f"  turbo_button: {self.turbo_button}")
        self.get_logger().info(f"  speed: {self.speed}")
        self.get_logger().info(f"  turn_speed: {self.turn_speed}")
        self.get_logger().info(f"  turbo_multiplier: {self.turbo_multiplier}")
        self.get_logger().info(f"  ramp_rate: {self.ramp_rate}")

        self.publisher_ = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10
        )

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # Target and current velocities
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Timer to update and publish smoothed velocity
        self.create_timer(0.05, self.update_velocity)  # 20Hz

    def joy_callback(self, msg: Joy):
        turbo_pressed = msg.buttons[self.turbo_button]
        speed = self.speed * self.turbo_multiplier if turbo_pressed else self.speed
        turn_speed = self.turn_speed * self.turbo_multiplier if turbo_pressed else self.turn_speed

        if msg.buttons[self.forward_button]:
            self.target_linear_x = speed
        elif msg.buttons[self.backward_button]:
            self.target_linear_x = -speed
        else:
            self.target_linear_x = 0.0

        self.target_angular_z = msg.axes[self.angular_axis] * turn_speed

    def update_velocity(self):
        # Smoothly move current velocities towards target velocities
        self.current_linear_x = self._smooth_step(
            self.current_linear_x, self.target_linear_x, self.ramp_rate
        )
        self.current_angular_z = self._smooth_step(
            self.current_angular_z, self.target_angular_z, self.ramp_rate
        )

        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.angular.z = self.current_angular_z
        self.publisher_.publish(twist)

    def _smooth_step(self, current, target, step):
        if abs(target - current) < step:
            return target
        return current + step if target > current else current - step


def main():
    rclpy.init()
    node = JoyDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
