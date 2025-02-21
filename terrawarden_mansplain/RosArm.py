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
        self.create_service(Empty, 'stow_arm', self.stowArm)
        self.create_service(Empty, 'unstow_arm', self.unStowArm)

        
        self.create_subscription(
            PoseStamped,
            'arm_target',
            self.arm_target_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            'arm_trajectory_target',
            self.arm_traj_callback,
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
        target_pose = msg.pose.position
        self.arm.write_time(0.1)
        joints = self.kinematics.ik(target_pose.x, target_pose.y, target_pose.z)
        if joints is not None:
            self.arm.write_joints(joints)
        else:
            self.get_logger().error("Invalid IK solution")
        
    def arm_traj_callback(self, msg):
        target_pose = msg.pose.position
        duration_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1e6 - self.get_clock().now().nanoseconds * 1e-6        
        self.runTaskTrajectory(target_pose.x,target_pose.y,target_pose.z,duration_ms)
        

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

    def runTaskTrajectory(self, goalx,goaly,goalz,duration_ms):
        print(goalx,goaly,goalz)
        self.arm.write_time(0.1)
        startTime=self.get_clock().now().nanoseconds*1e-6
        currentT=self.kinematics.fk(self.arm.read_position())
        # print("Current Joints: ", self.arm.read_position())
        # print("Current T: ", currentT)
        currentPose=[currentT[0][3],currentT[1][3],currentT[2][3]]
        coefficients=self.kinematics.generate_trajectory(currentPose,[goalx,goaly,goalz],duration_ms)
        rate = self.create_rate(100)
        while(self.get_clock().now().nanoseconds*1e-6-startTime<duration_ms):
            t=self.get_clock().now().nanoseconds*1e-6-startTime
            x=0
            y=0
            z=0
            for i in range(6):
                x+=coefficients[0][i]*t**i
                y+=coefficients[1][i]*t**i
                z+=coefficients[2][i]*t**i
            
            joints = self.kinematics.ik(x,y,z)
            # print("XYZ: ",x,y,z)
            # print("Joints: ",joints)
            if joints is not None:
                self.arm.write_joints(joints)
            rate.sleep()

    def runJointTrajectory(self, goal0,goal1,goal2,duration_ms):
        
        self.arm.write_time(0.1)
        startTime=self.get_clock().now().nanoseconds*1e-6
        currentJoints=self.arm.read_position()
        
        
        coefficients=self.kinematics.generate_trajectory(currentJoints,[goal0,goal1,goal2],duration_ms)
        rate = self.create_rate(100)
        while(self.get_clock().now().nanoseconds*1e-6-startTime<duration_ms):
            t=self.get_clock().now().nanoseconds*1e-6-startTime
            a=0
            b=0
            c=0
            for i in range(6):
                a+=coefficients[0][i]*t**i
                b+=coefficients[1][i]*t**i
                c+=coefficients[2][i]*t**i
            
            joints = [a,b,c]
            # print("Joints: ",joints)
            if joints is not None:
                self.arm.write_joints(joints)
            rate.sleep()
    
    def stowArm(self):
        movetime=2
        pos1 = self.kinematics.fk([-np.pi/2,0,-np.pi/2])
        self.runTaskTrajectory(pos1[0][3],pos1[1][3],pos1[2][3],movetime*1000)
        
        self.runJointTrajectory(-np.pi,0,-np.pi/2,movetime*1000)
        self.runJointTrajectory(-np.pi,-np.pi/2,np.pi/2,movetime*1000)
    
    def unStowArm(self):
        movetime=2
        self.runJointTrajectory(0,0,-np.pi/2,movetime*1000)
        pos1 = self.kinematics.fk([0,0,0])
        self.runTaskTrajectory(pos1[0][3],pos1[1][3],pos1[2][3],movetime*1000)
        
    

    

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()    
    # node.arm.set_torque(False)
    # node.runJointTrajectory(0,0,0,2000)
    
    print(node.arm.read_position())
    node.stowArm()
    node.unStowArm()

    # for i in range(3):
    #     node.arm.write_time(2)
    #     funpos = node.kinematics.fk([np.pi/3,-np.pi/3,np.pi/3]) 
    #     node.runTaskTrajectory(funpos[0][3],funpos[1][3],funpos[2][3],2000)
    #     # rclpy.spin_once(node, timeout_sec=.5)
        
    #     zeropos=node.kinematics.fk([0,0,0])

    #     node.runTaskTrajectory(zeropos[0][3],zeropos[1][3],zeropos[2][3],2000)
    #     # rclpy.spin_once(node, timeout_sec=.5)
    # rclpy.spin(node)
    # node.stowArm()
    # rclpy.spin(node)

if __name__ == '__main__':
    main()
