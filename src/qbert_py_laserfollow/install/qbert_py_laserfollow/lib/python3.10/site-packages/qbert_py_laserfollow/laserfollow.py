import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from sensor_msgs.msg import LaserScan
    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

class Laserfollow(Node):

    def __init__(self):
        super().__init__("laserfollow")


        self.hazard_subscription = self.create_subscription(
                HazardDetectionVector,
                'qbert/hazard_detection',
                self.hazard_callback,
                qos.qos_profile_sensor_data)

        self.hazard_subscription

        self.laser_subscription = self.create_subscription(LaserScan,
                '/qbert/scan', 
                self.laser_callback, 
                qos.qos_profile_sensor_data)

        self.laser_subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, "qbert/cmd_vel",10)

    def hazard_callback(self, haz):
        pass
    
    def laser_callback(self, las):
        readable = {las.ranges[i]: las.intensities[i] for i in range(len(las.ranges))}
        print(readable)
        #print(las.ranges[0])
        #print(las.ranges[-1])




def main(args=None):
    rclpy.init(args=args)

    laser = Laserfollow()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
