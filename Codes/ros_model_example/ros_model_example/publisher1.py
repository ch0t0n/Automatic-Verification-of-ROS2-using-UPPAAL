import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher1 = self.create_publisher(String, 'topic1', 10)
        self.timer1 = self.create_timer(2, self.timer1_callback1)
        self.publisher2 = self.create_publisher(String, 'topic1', 10)
        self.timer2 = self.create_timer(3, self.timer2_callback2)
        self.i = 0

    def timer1_callback1(self):
        msg1 = String()
        msg1.data = "msg1"
        self.publisher1.publish(msg1)
        self.get_logger().info("Publishing using pub1= %s" % msg1.data)
        

    def timer2_callback2(self):
        msg2 = String()
        msg2.data = "msg2"
        self.publisher2.publish(msg2)
        self.get_logger().info("Publishing using pub2= %s" % msg2.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
