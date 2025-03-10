import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyListener(Node):
    
    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Format axes with 2 decimal places, replacing leading space with '-'
        axes = [f"{x:6.2f}" for x in msg.axes]
        buttons = " ".join(str(b) for b in msg.buttons)

        print(f"Axes: {' '.join(axes)}   Buttons: {buttons}")


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
