import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String


class MinimalSubscriber2(Node):
    def __init__(self):
        super().__init__('minimal_subscriber2')
        self.subscription = self.create_subscription(
            String,
            'topic1',
            self.listener_callback,
            3)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard in sub2: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber2()
    # rclpy.spin(minimal_subscriber)

    try:
        while rclpy.ok():
            # minimal_subscriber.get_logger().info('Help me to understand timer, you are my only hope')
            rclpy.spin_once(minimal_subscriber)
            # time.sleep(3)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
