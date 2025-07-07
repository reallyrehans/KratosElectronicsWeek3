#!/usr/bin/env python3
import rclpy
from q4_minutes_node import MinsCounter as HoursCounter


def main(args=None):
    rclpy.init(args=args)
    hours_counter = HoursCounter("hours_counter", "minute", "hour")
    rclpy.spin(hours_counter)
    hours_counter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
