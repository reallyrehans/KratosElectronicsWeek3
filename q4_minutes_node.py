#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MinsCounter(Node):
    """
    Creating a node that subscribes to topic /second and publishes to /minute
    """

    def __init__(self, name, sub_topic, pub_topic):
        self.pub_topic = pub_topic
        self.sub_topic = sub_topic
        # Adding node name. super() is used to call functions of Node class
        super().__init__(name)
        # Creating subscription object on topic /second
        self.subscription_ = self.create_subscription(
            Int32, sub_topic, self.listener_callback, 10
        )
        # Creating publisher object on topic /minute
        self.publisher_ = self.create_publisher(Int32, pub_topic, 10)
        self.mins = 0

    def listener_callback(self, msg):
        """
        Function that is called every time second updates
        """
        if self.pub_topic == "minute":
            if self.mins == 60:
                self.mins = 0
        if msg.data == 60:
            self.mins += 1
        self.publisher_.publish(Int32(data=self.mins))
        print(self.pub_topic, ":", self.mins)


def main(args=None):
    rclpy.init(args=args)
    mins_counter = MinsCounter("mins_counter", "second", "minute")
    rclpy.spin(mins_counter)
    mins_counter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
