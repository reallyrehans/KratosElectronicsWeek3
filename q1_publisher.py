#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class HelloPublisher(Node):
    """
    Creating the node to publish Hello World 15 times a second
    """

    def __init__(self):
        # Adding node name. super() is used to call functions of Node class
        super().__init__("hello_publisher")
        # Creating publisher object on the topic /new
        self.publisher_ = self.create_publisher(String, "new", 10)
        timer_period = 1.0 / 15.0  # Time period corresponding to 15 Hz
        # Creating timer object
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        The function that is called 15 times a second
        """
        msg = String()
        msg.data = "Hello World !"
        self.publisher_.publish(msg)
        print("Added : " + msg.data)


def main(args=None):
    rclpy.init(args=args)
    hello_publisher = HelloPublisher()
    rclpy.spin(hello_publisher)
    hello_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
