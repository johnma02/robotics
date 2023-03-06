import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

class Reactor(Node):

    def __init__(self):
        super().__init__('reactor')
        
        self.ir_subscription = self.create_subscription(
            IrIntensityVector,
            '/qbert/ir_intensity',
            self.ir_callback,
            qos.qos_profile_sensor_data)

        self.ir_subscription  # prevent unused variable warning
        self.ir_publisher = self.create_publisher(Twist, "qbert/cmd_vel",10)

    def ir_callback(self, ir):
        pass

def main(args=None):
    rclpy.init(args=args)

    reactor = Reactor()
    
    rclpy.spin(reactor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reactor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



