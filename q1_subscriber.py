#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class HelloSubscriber(Node):
    """
    Creating a node that subscribes to topic /new
    """

    def __init__(self):
        # Adding node name. super() is used to call functions of Node class
        super().__init__("hello_subscriber")
        # Creating subscription object on topic /new
        self.subscription_ = self.create_subscription(
            String, "new", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        """
        Function that is called every time a message is seen on topic
        """
        print(msg.data)


def main(args=None):
    rclpy.init(args=args)
    hello_subscriber = HelloSubscriber()
    rclpy.spin(hello_subscriber)
    hello_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
