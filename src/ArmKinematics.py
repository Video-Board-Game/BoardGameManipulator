import rclpy
from rclpy.node import Node
from TerrawardenMansplain.srv import CalculatePose
import numpy as np


class ArmKinematics(Node):

    def __init__(self):
        super().__init__('arm_kinematics')
        self.srv = self.create_service(CalculatePose.srv, 'arm_kinematics', self.callback)
        
    def callback(self, request, response):
        # Assuming the input is a float array
        float_array = request.data

       

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()