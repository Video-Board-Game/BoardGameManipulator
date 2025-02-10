import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
from DynamixelArm import DynamixelArm  # Import the DynamixelArm class
from ArmKinematics import ArmKinematics  # Import the ArmKinematics class
from geometry_msgs.msg import PoseStamped
from msg import ArmStatus

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.arm = DynamixelArm()
        self.kinematics = ArmKinematics()
        self.srv = self.create_service(Empty, 'get_joints', self.get_joints_callback)

        self.subscription = self.create_subscription(
            PoseStamped,
            'arm_target',
            self.arm_target_callback,
            10
        )

    def get_joints_callback(self, request, response):
        joints = self.arm.get_joints()
        msg = Float64MultiArray()
        msg.data = joints
        self.get_logger().info(f'Joints: {joints}')
        response.success = True  # Assuming Empty service has a success field
        return response
        

    def arm_target_callback(self, msg):
        target_pose = msg.pose
        joint_angles = self.kinematics.inverse_kinematics(target_pose)
        self.arm.set_joints(joint_angles)
        self.get_logger().info(f'Set joints to: {joint_angles}')

    

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()
