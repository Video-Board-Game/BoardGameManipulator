import rclpy
from rclpy.node import Node
import rclpy.time
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
from terrawarden_mansplain.DynamixelArm import DynamixelArm  # Import the DynamixelArm class
from terrawarden_mansplain.ArmKinematics import ArmKinematics  # Import the ArmKinematics class
from geometry_msgs.msg import PoseStamped
from terrawarden_interfaces.msg import ArmStatus
import numpy as np

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        
        self.arm = DynamixelArm()
        self.arm.set_torque(True)
        self.kinematics = ArmKinematics()
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 10)
        self.srv = self.create_service(Empty, 'get_joints', self.get_joints_callback)
        self.timer = self.create_timer(0.5, self.publish_status)
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

    def publish_status(self):
        status_msg = ArmStatus()
        armPos=self.arm.read_position()
        status_msg.joint1position = float(armPos[0])
        status_msg.joint2position = float(armPos[1])
        status_msg.joint3position = float(armPos[2])
        armVel=self.arm.read_velocity()
        status_msg.joint1velocity = float(armVel[0])
        status_msg.joint2velocity = float(armVel[1])
        status_msg.joint3velocity = float(armVel[2])
        gripperPos=float(self.arm.read_gripper_position())
        print(status_msg)
        self.status_publisher.publish(status_msg)

    def runTrajectory(self, goalx,goaly,goalz,duration):
        print(goalx,goaly,goalz)
        startTime=self.get_clock().now().nanoseconds*1e-6
        currentT=self.kinematics.fk(self.arm.read_position())
        print(self.arm.read_position())
        print(currentT)
        currentPose=[currentT[0][3],currentT[1][3],currentT[2][3]]
        coefficients=self.kinematics.generate_trajectory(currentPose,[goalx,goaly,goalz],duration)
        while(self.get_clock().now().nanoseconds*1e-6-startTime<duration):
            t=self.get_clock().now().nanoseconds*1e-6-startTime
            x=0
            y=0
            z=0
            for i in range(6):
                x+=coefficients[0][i]*t**i
                y+=coefficients[1][i]*t**i
                z+=coefficients[2][i]*t**i
            
            joints = self.kinematics.ik(x,y,z)
            print(x,y,z)
            print(joints)
            self.arm.write_joints(joints)
        
    

    

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    zeropos=node.kinematics.fk([0,0,0])

    node.runTrajectory(zeropos[0][3],zeropos[1][3],zeropos[2][3],2000)
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()
