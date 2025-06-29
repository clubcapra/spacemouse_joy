import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import struct
import threading

class SpaceMouseServer(Node):
    def __init__(self):
        super().__init__('spacemouse_tcp_server')
        self.publisher = self.create_publisher(Joy, '/spacemouse_joy', 10)

    def publish_from_data(self, data: bytes):
        if len(data) != 6 * 4 + 2 * 1:
            self.get_logger().warn("Invalid data length")
            return

        axes = list(struct.unpack('6f2B', data))  # returns 8 values
        joy = Joy()
        joy.axes = axes[:6]
        joy.buttons = [int(axes[6]), int(axes[7])]
        self.publisher.publish(joy)

def tcp_server(node, ip='localhost', port=2222):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((ip, port))
    server.listen(1)
    conn, addr = server.accept()
    node.get_logger().info(f"TCP connection from {addr}")
    try:
        while True:
            data = conn.recv(28)  # 6 floats (4B) + 2 bytes = 28 bytes
            if not data:
                break
            node.publish_from_data(data)
    except Exception as e:
        node.get_logger().error(f"TCP server error: {e}")
    finally:
        conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseServer()
    thread = threading.Thread(target=tcp_server, args=(node,), daemon=True)
    thread.start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
