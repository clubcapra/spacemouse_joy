import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyListener(Node):
    
    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def joy_callback(self, msg):
        # Map the joystick axes to linear and angular velocities
        twist_msg = Twist()

        # Assuming axes[0] = X (forward/backward), axes[1] = Y (left/right), axes[3] = pitch (rotation)
        twist_msg.linear.x = msg.axes[0]  # Forward/backward motion (X-axis)
        twist_msg.linear.y = msg.axes[1]  # Left/right motion (Y-axis)
        twist_msg.linear.z = msg.axes[2]  # Up/down motion (Z-axis)
        twist_msg.angular.x = msg.axes[3]  # Rotation in pitch
        twist_msg.angular.y = msg.axes[4]  # Rotation in roll
        twist_msg.angular.z = msg.axes[5]  # Rotation in yaw

        # Publish the twist message
        self.cmd_vel_publisher.publish(twist_msg)

        # Print the Joy message axes and buttons for debugging
        printable_axes = [f"{x:6.2f}" for x in msg.axes]
        printable_buttons = " ".join(str(b) for b in msg.buttons)
        self.get_logger().info(f"Axes: {' '.join(printable_axes)}   Buttons: {printable_buttons}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down joy subscriber.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
