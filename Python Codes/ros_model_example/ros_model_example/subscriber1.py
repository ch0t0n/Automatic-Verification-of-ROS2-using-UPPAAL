import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher3 = self.create_publisher(String, 'topic2', 10)
        self.subscription = self.create_subscription(String,'topic1',self.listener_callback,5)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.publisher3.publish(msg)
        self.get_logger().info('I heard in sub1: "%s"' % msg.data)
        


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    r = minimal_subscriber.create_rate(10)

    rclpy.spin(minimal_subscriber)

    try:
        while rclpy.ok():
            # minimal_subscriber.get_logger().info('Help me to understand timer, you are my only hope')
            rclpy.spin_once(minimal_subscriber)
            r.sleep()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
