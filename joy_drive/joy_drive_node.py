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

        # Get parameters
        self.forward_button = self.get_parameter('forward_button').get_parameter_value().integer_value
        self.backward_button = self.get_parameter('backward_button').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.turbo_button = self.get_parameter('turbo_button').get_parameter_value().integer_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').get_parameter_value().double_value

        self.get_logger().info("joy_drive node started with parameters:")
        self.get_logger().info(f"  forward_button: {self.forward_button}")
        self.get_logger().info(f"  backward_button: {self.backward_button}")
        self.get_logger().info(f"  angular_axis: {self.angular_axis}")
        self.get_logger().info(f"  turbo_button: {self.turbo_button}")
        self.get_logger().info(f"  speed: {self.speed}")
        self.get_logger().info(f"  turn_speed: {self.turn_speed}")
        self.get_logger().info(f"  turbo_multiplier: {self.turbo_multiplier}")

        self.publisher_ = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10
        )

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

    def joy_callback(self, msg: Joy):
        twist = Twist()

        turbo_pressed = msg.buttons[self.turbo_button]
        speed = self.speed * self.turbo_multiplier if turbo_pressed else self.speed
        turn_speed = self.turn_speed * self.turbo_multiplier if turbo_pressed else self.turn_speed

        if msg.buttons[self.forward_button]:
            twist.linear.x = speed
        elif msg.buttons[self.backward_button]:
            twist.linear.x = -speed
        else:
            twist.linear.x = 0.0

        twist.angular.z = msg.axes[self.angular_axis] * turn_speed
        self.publisher_.publish(twist)


def main():
    rclpy.init()
    node = JoyDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
