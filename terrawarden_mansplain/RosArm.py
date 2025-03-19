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
import time


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        
        self.arm = DynamixelArm()
        self.arm.set_torque(True)
        self.kinematics = ArmKinematics()
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 10)
        # self.srv = self.create_service(Empty, 'get_joints', self.get_joints_callback)
        self.create_timer(0.2, self.publish_status)
        
        self.create_service(Empty, 'stow_arm', self.stowArm)
        self.create_service(Empty, 'unstow_arm', self.unStowArm)
        self.init_time=self.get_clock().now().nanoseconds*1e-6
        self.inmotion = False
        self.startingMovementTime=-1
        self.trajCoeff=np.zeros([3,6])
        self.goal = np.zeros(3)
        self.endingMovementTime=-1
        self.create_timer(0.01, self.run_traj_callback)  # Timer to run the trajectory callback
        self.trajMode="task"
        self.stowSteps=[]
        self.create_timer(2, self.run_stowArm_callback)  # Timer to run the stow arm callback
        
        # self.create_subscription(
        #     PoseStamped,
        #     'arm_target',
        #     self.arm_target_callback,
        #     10
        # )

        self.create_subscription(
            PoseStamped,
            'joisie_extract_centroid',
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
        

    # def arm_target_callback(self, msg):
    #     target_pose = msg.pose.position
    #     self.arm.write_time(2)
    #     joints = self.kinematics.ik(target_pose.x, target_pose.y, target_pose.z)
    #     if joints is not None:
    #         self.arm.write_joints(joints)
    #     else:
    #         self.get_logger().error("Invalid IK solution")
        
    def arm_traj_callback(self, msg):
        if len(self.stowSteps)==0:
            print(["Goal", msg.pose.position])
            target_pose = msg.pose.position
            self.trajMode="task"
            self.goal = np.array([target_pose.x, target_pose.y, target_pose.z])
            self.startingMovementTime=self.get_clock().now().nanoseconds*1e-6-self.init_time
            self.endingMovementTime = self.startingMovementTime + 1000
            current_joints = self.arm.read_position()
            current_pos = self.kinematics.fk(current_joints)
            self.trajCoeff=self.kinematics.generate_trajectory([current_pos[0][3],current_pos[1][3],current_pos[2][3]], self.goal, self.startingMovementTime,self.endingMovementTime)
        
    def run_traj_callback(self):
        if self.get_clock().now().nanoseconds*1e-6-self.init_time<self.endingMovementTime:
            self.arm.write_time(0.1)
            t=self.get_clock().now().nanoseconds*1e-6-self.init_time
            a=0
            b=0
            c=0
            for i in range(6):
                a+=self.trajCoeff[0][i]*t**i
                b+=self.trajCoeff[1][i]*t**i
                c+=self.trajCoeff[2][i]*t**i
            if self.trajMode=="task":
                joints = self.kinematics.ik(a,b,c)
            else:
                joints = np.array([a,b,c])
            print("ABC: ",a,b,c)
            print("Joints: ",joints)
            if joints is not None and self.kinematics.check_move_safe(joints):
                self.arm.write_joints(joints)

    def publish_status(self):
        status_msg = ArmStatus()
        armPos=self.arm.read_position()
        status_msg.joint1position = float(armPos[0])
        status_msg.joint2position = float(armPos[1])
        status_msg.joint3position = float(armPos[2])
        eepos=self.kinematics.fk(armPos)
        status_msg.eex=float(eepos[0][3])
        status_msg.eey=float(eepos[1][3])
        status_msg.eez=float(eepos[2][3])
        armVel=self.arm.read_velocity()
        status_msg.joint1velocity = float(armVel[0])
        status_msg.joint2velocity = float(armVel[1])
        status_msg.joint3velocity = float(armVel[2])
        # gripperPos=float(self.arm.read_gripper_position())
        # print(status_msg)
        self.status_publisher.publish(status_msg)

    def runTaskTrajectory(self, goalx,goaly,goalz,duration_ms):
        self.trajMode="task"
        self.goal = np.array([goalx,goaly,goalz])
        self.startingMovementTime=self.get_clock().now().nanoseconds*1e-6-self.init_time
        self.endingMovementTime = self.startingMovementTime + duration_ms
        self.trajCoeff=self.kinematics.generate_trajectory(self.arm.read_position(), self.goal, self.startingMovementTime,self.endingMovementTime)



    def runJointTrajectory(self, goal0,goal1,goal2,duration_ms):
        self.trajMode="joint"
        self.goal = np.array([goal0,goal1,goal2])
        self.startingMovementTime=self.get_clock().now().nanoseconds*1e-6-self.init_time
        self.endingMovementTime = self.startingMovementTime + duration_ms
        # print(self.arm.read_position())
        # print(self.startingMovementTime)
        # print(self.endingMovementTime)
        self.trajCoeff=self.kinematics.generate_trajectory(self.arm.read_position(), self.goal, self.startingMovementTime,self.endingMovementTime)
        # print(self.trajCoeff)
        # self.endingMovementTime=-1

    def run_stowArm_callback(self):
        if len(self.stowSteps)>0:
            print("running next stow step")
            step=self.stowSteps.pop(0)
            if step[0]=="task":
                self.runTaskTrajectory(step[1],step[2],step[3],1000)
            elif step[0]=="joint":
                self.runJointTrajectory(step[1],step[2],step[3],1000)
            else:
                print("Invalid stow step")
    
    def stowArm(self):
        # pos1 = self.kinematics.fk([0,0,-np.pi/2])       
        self.stowSteps=[["joint",0,0,-np.pi/2],
                        ["joint",-31*np.pi/32,0,-np.pi/2],
                        ["joint",-31*np.pi/32,-np.pi/2.1,np.pi/2.1],
                        ["joint",-2*np.pi/3,-np.pi/2.1,np.pi/2.1]]
        
        
        
    
    def unStowArm(self):
        pos1 = self.kinematics.fk([0,0,0])
        self.stowSteps=[["joint",-31*np.pi/32,-np.pi/2.1,np.pi/2.1],
                        ["joint",-31*np.pi/32,-np.pi/6,-np.pi/2],
                        ["joint",0,0,0]]
        
        
    

    

def main(args=None):
    
    rclpy.init(args=args)
    node = ArmNode()    
    print("Starting arm node")
    # node.arm.reboot()
    node.arm.set_torque(True)
    # node.runJointTrajectory(0,np.pi/2,-np.pi/2,3000)
    #node.runJointTrajectory(0,0,0,750)
    # node.runJointTrajectory(-3*np.pi/4,-np.pi/2.1,np.pi/2,3000)
    # node.runJointTrajectory(0,0,-np.pi/2,1000)
    
    # print(node.arm.read_position())
    node.stowArm()
    # node.unStowArm()

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
    rclpy.spin(node)

if __name__ == '__main__':
    main()
