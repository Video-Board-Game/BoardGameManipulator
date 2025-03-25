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
GOAL = np.array([0,0,0])
#GOAL = np.array([0,np.pi/2,-np.pi/2])
#GOAL = np.array([0,0,-np.pi/2])
MOVE_TIME = 3*1000

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        
        # Initialization
        self.arm = DynamixelArm()
        self.arm.set_torque(True)
        self.kinematics = ArmKinematics()
        
        #storing node start time to shrink the times stored to avoid numerical errors
        self.init_time=self.get_clock().now().nanoseconds*1e-6
        self.startingMovementTime=-1 #time when the trajectory starts
        self.endingMovementTime=-1 #time when the trajectory ends

        self.trajCoeff=np.zeros([3,6])
        self.goal = np.zeros(3) #current goal position
        self.isStowed=False
        
        self.trajMode="task"
        self.stowSteps=[] # list of steps to run in stowing routine

        # Publishers
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 10)  # Publisher for arm status

        # Services
        self.create_service(Empty, 'stow_arm', self.stowArm) #service to stow the arm
        self.create_service(Empty, 'unstow_arm', self.unStowArm) #service to unstow the arm

        # Timers
        self.create_timer(0.005, self.run_traj_callback)  # Timer to run the trajectory callback
        self.create_timer(2, self.run_stowArm_callback)  # Timer to run the stow arm callback
        self.create_timer(0.2, self.publish_status)  # Timer to publish arm status
        
        # Subscriptions
        # self.create_subscription(
        #     PoseStamped,
        #     'arm_target',
        #     self.arm_target_callback,
        #     10
        # )

        self.create_subscription(
            PoseStamped,
            'joisie_arm_trajectory_target',
            self.arm_traj_callback,
            10
        )

        rclpy.get_default_context().on_shutdown(self.on_shutdown)
    
    def on_shutdown(self):
        # Turn off arm when node turn off
        self.arm.set_torque(False)

    # Subscription callbacks
    # def arm_target_callback(self, msg):
    #     target_pose = msg.pose.position
    #     self.arm.write_time(2)
    #     joints = self.kinematics.ik(target_pose.x, target_pose.y, target_pose.z)
    #     if joints is not None:
    #         self.arm.write_joints(joints)
    #     else:
    #         self.get_logger().error("Invalid IK solution")
        
        
    def arm_traj_callback(self, msg):
        """Callback function for the arm trajectory topic.
        This function processes incoming PoseStamped messages and generates a trajectory for the arm to follow.
        It checks if the arm is currently unstowed and if so, it generates a trajectory to the specified goal position.
        Args:
            msg (PoseStamped): The incoming PoseStamped message containing the target pose.
        Returns:
            None
        """
        self.arm.write_time(1500)
        target_pose = msg.pose.position
        self.goal = np.array([target_pose.x, target_pose.y, target_pose.z])
        joints=self.kinematics.ik(self.goal[0],self.goal[1],self.goal[2])
        if not joints is None:
            self.arm.write_joints(joints)
        # if len(self.stowSteps)==0 and not self.isStowed:
        #     print(["Goal", msg.pose.position])
        #     target_pose = msg.pose.position
        #     self.trajMode="task"
        #     self.goal = np.array([target_pose.x, target_pose.y, target_pose.z])
        #     self.startingMovementTime=self.get_clock().now().nanoseconds*1e-6-self.init_time
        #     self.endingMovementTime = self.startingMovementTime + 500
            
        #     current_joints = self.arm.read_position()
        #     current_pos = self.kinematics.fk(current_joints)
        #     current_jac = self.kinematics.vk(current_joints)
        #     current_joint_vel = self.arm.read_velocity()
        #     current_vel=(current_jac @ np.vstack(current_joint_vel)).flatten()
        #     print(current_vel)
        #     current_vel=[0,0,0]
        #     self.trajCoeff=self.kinematics.generate_trajectory(start=[current_pos[0][3],current_pos[1][3],current_pos[2][3]], end=self.goal, start_vel=current_vel, start_time=self.startingMovementTime,end_time=self.endingMovementTime)
        
        
    # Timer callbacks
    def run_traj_callback(self):
        """Runs the trajectory for the arm based on the current mode (task or joint).
        This function checks if the current time is within the trajectory duration and updates the arm's position accordingly.
        If the trajectory is in task mode, it calculates the joint angles using inverse kinematics.
        If the trajectory is in joint mode, it directly sets the joint angles.
        Args:
            None
        Returns:
            None
        """
        # Check if the trajectory is still running
        if self.get_clock().now().nanoseconds*1e-6-self.init_time<self.endingMovementTime:
            self.arm.write_time(0.08)#makes sure movements are fast
            t=self.get_clock().now().nanoseconds*1e-6-self.init_time
            a=0
            b=0
            c=0
            #calculates the points within the trajectory whether joint or task
            for i in range(6):
                a+=self.trajCoeff[0][i]*t**i
                b+=self.trajCoeff[1][i]*t**i
                c+=self.trajCoeff[2][i]*t**i
            #if task mode, calculates the joint angles using inverse kinematics
            if self.trajMode=="task":
                joints = self.kinematics.ik(a,b,c)
            else:
                joints = np.array([a,b,c])
            # print("ABC: ",a,b,c)
            # print("Joints: ",joints)
            if joints is not None and self.kinematics.check_move_safe(joints):
                self.arm.write_joints(joints)


    def publish_status(self):
        """
        Publishes the current status of the robotic arm.
        This function reads the current joint positions, end-effector position, 
        and joint velocities of the robotic arm, and publishes this information 
        as an `ArmStatus` message.
        Args:
            None
        Returns:
            None
        """

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


    # Trajectory methods
    def runTaskTrajectory(self, goalx,goaly,goalz,duration_ms):
        """
        Creates a task space trajectory for the robotic arm.
        This function generates a trajectory in task space for the robotic arm
        to move from its current position to the specified goal position in 
        Cartesian coordinates (x, y, z) within the given duration.
        Args:
            goalx (float): The target x-coordinate in task space.
            goaly (float): The target y-coordinate in task space.
            goalz (float): The target z-coordinate in task space.
            duration_ms (float): The duration of the trajectory in milliseconds.
        Returns:
            None
        """
        
        self.trajMode="task"
        self.goal = np.array([goalx,goaly,goalz])
        self.startingMovementTime=self.get_clock().now().nanoseconds*1e-6 - self.init_time
        self.endingMovementTime = self.startingMovementTime + duration_ms
        
        self.trajCoeff=self.kinematics.generate_trajectory(self.arm.read_position(), self.goal, start_time=self.startingMovementTime,end_time=self.endingMovementTime)


    def runJointTrajectory(self, goal0,goal1,goal2,duration_ms):
        """
        Creates a joint space trajectory for the robotic arm.
        This function generates a trajectory in joint space for the robotic arm
        to move from its current position to the specified goal positions for 
        each joint within the given duration.
        Args:
            goal0 (float): The target position for joint 0.
            goal1 (float): The target position for joint 1.
            goal2 (float): The target position for joint 2.
            duration_ms (float): The duration of the trajectory in milliseconds.
        Returns:
            None
        """

        self.trajMode="joint"
        self.goal = np.array([goal0,goal1,goal2])
        self.startingMovementTime=self.get_clock().now().nanoseconds*1e-6-self.init_time
        self.endingMovementTime = self.startingMovementTime + duration_ms
        self.trajCoeff=self.kinematics.generate_trajectory(self.arm.read_position(), self.goal, start_time=self.startingMovementTime,end_time=self.endingMovementTime)
        


    # Stow/unstow methods
    def run_stowArm_callback(self):
        """Starts the next trajectory step in the stowing process.
        This method processes the next step in the `stowSteps` list. Depending on the type of step, 
        it either runs a task trajectory or a joint trajectory. If the `stowSteps` list becomes empty, 
        the stow state is flipped to reflect the new state.
        """

        if len(self.stowSteps)>0:
            print("running next stow step")
            # Takes out the next stow step
            step=self.stowSteps.pop(0)

            if self.stowSteps==[]:#changes stow state on last step
                self.isStowed= not self.isStowed

            #sets trajectory based of step type
            if step[0]=="task": 
                self.runTaskTrajectory(step[1],step[2],step[3],1000)
            elif step[0]=="joint":
                self.runJointTrajectory(step[1],step[2],step[3],1000)
            else:
                print("Invalid stow step")
    
    
    def stowArm(self,req=None,resp=None):
        """Initiates the stowing sequence if the arm is unstowed.

        Args:
            req: The service request (unused).
            resp: The service response (unused).
        """
        joints = self.arm.read_position()
        if not self.isStowed and np.pi/2>joints[0]>-np.pi/2:
            self.stowSteps=[["joint", 0, 0, -np.pi/2],
                            ["joint", -31*np.pi/32, 0, -np.pi/2],
                            ["joint", -31*np.pi/32, -np.pi/2.1, np.pi/2.1],
                            ["joint", -2*np.pi/3, -np.pi/2.1, np.pi/2.1]]
        return resp
        
        
    def unStowArm(self,req=None,resp=None):
        """Initiates the unstowing sequence if the arm is stowed.

        Args:
            req: The service request (unused).
            resp: The service response (unused).
        """
        joints = self.arm.read_position()
        if self.isStowed and (np.pi/2<joints[0] or joints[0]<-np.pi/2):
            self.stowSteps=[["joint", -31*np.pi/32, -np.pi/2.1, np.pi/2.1],
                            ["joint", -31*np.pi/32, -np.pi/6, -np.pi/2],
                            ["joint", 0, 0, 0]]
        return resp
        

def main(args=None):
    # Main entry point
    rclpy.init(args=args)
    node = ArmNode()    
    print("Starting arm node")
    # node.arm.reboot()
    node.arm.set_torque(True)
    node.runJointTrajectory(GOAL[0],GOAL[1],GOAL[2],MOVE_TIME)
    # node.runJointTrajectory(0,-np.pi/2,np.pi/2,3000)
    node.runJointTrajectory(0,0,0,2000)

    node.arm.set_torque(False)
    # node.runJointTrajectory(-3*np.pi/4,-np.pi/2.1,np.pi/2,3000)
    # node.runJointTrajectory(0,0,-np.pi/2,1000)
    
    # print(node.arm.read_position())
    # node.stowArm()
    # node.unStowArm()
# 
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
