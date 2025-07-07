#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time


class Signal1(Node):
    """
    Publishes red and green alternately on /s1, each for 10 seconds.
    """

    def __init__(self):
        # Adding node name. super() is used to call functions of Node class
        super().__init__("signal_1")
        # Creating publisher object on the topic /s1
        self.publisher_ = self.create_publisher(String, "s1", 10)
        # Starting color
        self.color = "red"
        self.start_time = time.time()
        # Creating timer object
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # Logic to swap colors every 10 seconds using time()
        if (time.time() - self.start_time) >= 10:
            self.start_time = time.time()
            if self.color == "red":
                self.color = "green"
            else:
                self.color = "red"

        msg = String()
        msg.data = self.color
        # Publishing message on topic /s1
        self.publisher_.publish(msg)
        print("Added to /s1 : " + msg.data)


def main(args=None):
    rclpy.init(args=args)
    signal_1 = Signal1()
    rclpy.spin(signal_1)
    signal_1.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
