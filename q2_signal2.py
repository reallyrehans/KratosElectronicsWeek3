#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class Signal2(Node):
    """
    Creating a node that subscribes to topic /new
    """

    def __init__(self):
        # Adding node name. super() is used to call functions of Node class
        super().__init__("signal_2")
        # Creating subscription object on topic /s1
        self.subscription_ = self.create_subscription(
            String, "s1", self.listener_callback, 10
        )
        # Creating publisher object on topic /s2
        self.publisher_ = self.create_publisher(String, "s2", 10)

    def listener_callback(self, msg):
        """
        Function that is called every time a message is seen on topic
        """
        # Logic to swap color
        if msg.data == "red":
            msg.data = "green"
        elif msg.data == "green":
            msg.data = "red"
        self.publisher_.publish(msg)
        print("Added to /s2 : " + msg.data)


def main(args=None):
    rclpy.init(args=args)
    signal_2 = Signal2()
    rclpy.spin(signal_2)
    signal_2.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
