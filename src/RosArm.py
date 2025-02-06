import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
from DynamixelArm import DynamixelArm  # Import the DynamixelArm class

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.arm = DynamixelArm()
        self.srv = self.create_service(Empty, 'get_joints', self.get_joints_callback)

    def get_joints_callback(self, request, response):
        joints = self.arm.get_joints()
        msg = Float64MultiArray()
        msg.data = joints
        self.get_logger().info(f'Joints: {joints}')
        response.success = True  # Assuming Empty service has a success field
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
