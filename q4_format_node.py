#!/usr/bin/env python3

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class TimeFormatNode(Node):
    """
    Creating the node to publish second
    """

    def __init__(self):
        # Adding node name. super() is used to call functions of Node class
        super().__init__("time_format_node")
        # Creating publisher object on topic /minute
        self.publisher_ = self.create_publisher(String, "clock", 10)
        # Keeping track of times
        self.sec = 0
        self.min = 0
        self.hr = 0
        # Creating subscription object on topics
        self.sub_hour = self.create_subscription(Int32, "hour", self.hour_callback, 10)
        self.sub_minute = self.create_subscription(
            Int32, "minute", self.minute_callback, 10
        )
        self.sub_second = self.create_subscription(
            Int32, "second", self.second_callback, 10
        )

    def hour_callback(self, msg):
        self.hr = msg.data

    def minute_callback(self, msg):
        self.min = msg.data

    def second_callback(self, msg):
        """
        Function that is called every time second updates
        """
        self.sec = msg.data
        time_formatted = String(data=f"{self.hr}:{self.min}:{self.sec}")
        self.publisher_.publish(time_formatted)
        print(time_formatted.data)


def main(args=None):
    rclpy.init(args=args)
    time_format_node = TimeFormatNode()
    rclpy.spin(time_format_node)
    time_format_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
