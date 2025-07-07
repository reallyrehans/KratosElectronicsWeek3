#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import time
from datetime import datetime

"""
EXAMPLE Publish
ROVER DATA at 2025-07-07 19:36:26        
Velocity = Linear[0.0, 0.0, 0.0]        
        Angular[0.0, 0.0, 0.0]        
Distance Travelled = 0.0 metres        
Coordinates = Position[0.0, 0.0, 0.0]        
            Orientation[0.0, 0.0, 0.0, 1.0]        
Battery Level = 100.0%        
Time of Travel = 0.0 seconds        
x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x
"""

class RoverInfo(Node):
    """
    Creating the node to publish rover data
    """

    def __init__(
        self,
        rover_vel: Twist,
        dist_m: float,
        coords: Pose,
        battery: float,
        time_s: float,
    ):

        self.rover_vel = rover_vel
        self.dist_m = dist_m
        self.coords = coords
        self.battery = battery
        self.time_s = time_s

        # Adding node name. super() is used to call functions of Node class
        super().__init__("rover_info")
        # Creating publisher object on the topic /rover_info
        self.publisher_ = self.create_publisher(String, "rover_info", 10)
        # Creating timer object
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        good_time = datetime.fromtimestamp(time.time()).strftime("%Y-%m-%d %H:%M:%S")
        msg.data = f"\n\nROVER DATA at {good_time}\
        \nVelocity = Linear{[self.rover_vel.linear.x, self.rover_vel.linear.y, self.rover_vel.linear.z]}\
        \n           Angular{[self.rover_vel.angular.x, self.rover_vel.angular.y, self.rover_vel.angular.z]}\
        \nDistance Travelled = {self.dist_m} metres\
        \nCoordinates = Position{[self.coords.position.x, self.coords.position.y, self.coords.position.z]}\
        \n              Orientation{[self.coords.orientation.x, self.coords.orientation.y, self.coords.orientation.z, self.coords.orientation.w]}\
        \nBattery Level = {self.battery}%\
        \nTime of Travel = {self.time_s} seconds\
        \nx-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x"

        self.publisher_.publish(msg)
        print(msg.data)


def main(args=None):
    """
    Creating standard values for rover information
    These should be obtained by sensors (other nodes)
    """
    # Creating velocity as a Twist.
    # Twist is made of 2 Vector3 Objects
    rover_vel = Twist()
    rover_vel.linear.x = 0.0
    rover_vel.linear.y = 0.0
    rover_vel.linear.z = 0.0
    rover_vel.angular.x = 0.0
    rover_vel.angular.y = 0.0
    rover_vel.angular.z = 0.0

    # Rover coordinates as Pose
    # Pose is made of a Vector3 for position and Quarternion for orientation
    rover_coord = Pose()
    rover_coord.position.x = 0.0
    rover_coord.position.y = 0.0
    rover_coord.position.z = 0.0
    rover_coord.orientation.x = 0.0
    rover_coord.orientation.y = 0.0
    rover_coord.orientation.z = 0.0
    rover_coord.orientation.w = 1.0  # standard value for normalization

    # Rover distance, battery, time as float
    rover_dist = 0.0
    rover_battery = 100.0
    rover_time = 0.0

    rclpy.init(args=args)
    rover_info = RoverInfo(
        rover_vel, rover_dist, rover_coord, rover_battery, rover_time
    )
    rclpy.spin(rover_info)
    rover_info.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
