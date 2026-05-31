import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class RudraAlive(Node):

    def __init__(self) -> None:
        super().__init__('rudra_alive')
        self.heartbeat = 0
        self.heartbeat_pub = self.create_publisher(Int32, 'rudra/heartbeat', 10)
        self.get_logger().info('RUDRA alive node started.')
        self.timer = self.create_timer(1.0, self.publish_heartbeat)

    def publish_heartbeat(self) -> None:
        msg = Int32()
        msg.data = self.heartbeat
        self.heartbeat_pub.publish(msg)
        self.get_logger().info(f'RUDRA is alive. Heartbeat = {self.heartbeat}')
        self.heartbeat += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RudraAlive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
