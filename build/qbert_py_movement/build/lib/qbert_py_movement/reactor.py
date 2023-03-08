import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from irobot_create_msgs.msg import IrIntensityVector
    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time
import functools 

class Reactor(Node):

    def __init__(self):
        super().__init__('reactor')
        
        self.ir_subscription = self.create_subscription(
            IrIntensityVector,
            'qbert/ir_intensity',
            self.ir_callback,
            qos.qos_profile_sensor_data)

        self.ir_subscription  # prevent unused variable warning
        self.ir_publisher = self.create_publisher(Twist, "qbert/cmd_vel",10)

    def ir_callback(self, ir):
        vals = [x.value for x in ir.readings]
        vals = enumerate(vals)
        threats = list(filter(lambda x : x[1] > 300, vals))
        if len(threats) > 0:
            threat_direction = list(functools.reduce(lambda a,b : a if a[1] > b[1] else b, threats))
            #print(threat_direction)
            turn = Twist()
            match threat_direction[0]:
                case 0:
                    turn.angular.z = -.4
                case 1:
                    turn.angular.z = -.7
                case 2:
                    turn.angular.z = -.8
                case 3:
                    turn.angular.z = -1.0
                case 4:
                    turn.angular.z = .8
                case 5:
                    turn.angular.z = .7
                case 6:
                    turn.angular.z = .4
            self.ir_publisher.publish(turn)
        else:
            forward = Twist()
            forward.linear.x = .075
            self.ir_publisher.publish(forward)

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



