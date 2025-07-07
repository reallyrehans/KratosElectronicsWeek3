#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SecondsCounter(Node):
    """
    Creating the node to publish seconds
    """

    def __init__(self):
        # Adding node name. super() is used to call functions of Node class
        super().__init__("seconds_node")
        # Creating publisher object on the topic /new
        self.publisher_ = self.create_publisher(Int32, "second", 10)
        # Creating timer object
        self.timer = self.create_timer(1, self.timer_callback)
        self.sec = 0

    def timer_callback(self):
        """
        Called every second
        """
        msg = Int32(data=self.sec)
        self.publisher_.publish(msg)
        print("Second : ", msg.data)
        if self.sec == 60:
            self.sec = 0
        self.sec += 1


def main(args=None):
    rclpy.init(args=args)
    seconds_node = SecondsCounter()
    rclpy.spin(seconds_node)
    seconds_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
