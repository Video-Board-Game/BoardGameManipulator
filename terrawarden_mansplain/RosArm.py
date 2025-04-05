import rclpy
import rclpy.context
from rclpy.node import Node
import rclpy.time
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray, Bool
from terrawarden_mansplain.DynamixelArm import DynamixelArm  # Import the DynamixelArm class
from terrawarden_mansplain.ArmKinematics import ArmKinematics  # Import the ArmKinematics class
from geometry_msgs.msg import PoseStamped
from terrawarden_interfaces.msg import ArmStatus
import numpy as np
import threading


GOAL = np.array([0,0,0])
#GOAL = np.array([0,np.pi/2,-np.pi/2])
#GOAL = np.array([0,0,-np.pi/2])
MOVE_TIME = 3*1000

# Time in milliseconds for each step, used to space out multi-step routines
TIME_PER_TRAJ_STEP = 1200 

TIME_PER_STOW_STEP = 1250  # tuned for not much drone jerk
STOW_ROUTINE_SETPOINTS = [
    # Stow routine setpoints (type, val1, val2, val3)
    ["joint", 0, 0, -17*np.pi/32], 
    ["joint", -31*np.pi/32, 0, -17*np.pi/32], # rotate straight down so as not to obstruct the drone optical flow
    ["joint", -30*np.pi/32, -17*np.pi/32, 16*np.pi/32],  # fold upwards base joint back
    ["joint", -19*np.pi/32, -17*np.pi/32, 16*np.pi/32],  
    ["joint", -19*np.pi/32, -14*np.pi/32, 16*np.pi/32]   # Final position resting folded on the drone leg
]
UNSTOW_ROUTINE_SETPOINTS = [
    # Unstow routine setpoints (type, val1, val2, val3)
    ["joint", -19*np.pi/32, -14*np.pi/32, 16*np.pi/32], # stow position, then unstow basically backwards
    ["joint", -19*np.pi/32, -17*np.pi/32, 16*np.pi/32],  
    ["joint", -31*np.pi/32, -17*np.pi/32, 16*np.pi/32],    
    ["joint", -31*np.pi/32, 3*np.pi/32, -14*np.pi/32],
    ["joint", -31*np.pi/32, 0*np.pi/32, -17*np.pi/32],
    ["joint", 0, 0*np.pi/32, -12*np.pi/32], # make the massive swing slower    
    ["joint", 0, 0, 0]  # Final position to ensure the arm is fully unstowed in "L" shape forward
]

from enum import Enum
class TrajectoryModes(Enum):
    JOINT_SPACE = "joint"
    TASK_SPACE = "task"
    STOW_UNSTOW = "stow"

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        
        self.get_logger().info('Initializing Arm Node...')
        
        # Initialization
        self.arm = DynamixelArm()
        self.kinematics = ArmKinematics()
        self.arm.set_torque(True)
        self.arm.set_gripper_torque(True)
        
        #storing node start time to shrink the times stored to avoid numerical errors
        self.init_time_sec=self.get_clock().now().nanoseconds*1e-9
        self.startingMovementTimeSec=-1 #time when the trajectory starts
        self.endingMovementTimeSec=-1 #time when the trajectory ends

        self.traj_coeffs=np.zeros([3,6])
        self.goal = np.zeros(3) #current goal position
       
        self.isStowed=False
        if not self.check_if_arm_stowed(): # Check if the arm is in a stowed position at startup
            # is stowed, snap back to position
            self.isStowed = True
        
        self.trajMode = TrajectoryModes.TASK_SPACE

        # TODO: Why not just set all trajectories to use one array
        #       and then have a main loop that processes it rather than just using setpoints
        #       this is actually just way worse than 3001 code -K
        self.stowSteps=[] # list of steps to run in stowing routine

        # Publishers
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 10)  # Publisher for arm status


        # Services
        self.create_service(Empty, 'stow_arm', self.stowArm) #service to stow the arm
        self.create_service(Empty, 'unstow_arm', self.unStowArm) #service to unstow the arm


        # Timers
        # TODO Ew what -K
        self.create_timer(0.01, self.run_traj_callback)  # Timer to run the trajectory callback
        self.create_timer(0.1, self.run_stowArm_callback)  # Timer to run the stow arm callback
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

        self.follow_target = True  # Initialize follow_target state


        self.create_subscription(
            Bool,
            'follow_target',
            self.follow_target_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            'move_to_grasp',
            self.move_to_grasp_callback,
            10
        )
            

        self.tracking_offset = 0.075 # Offset radius to move to before moving to grasp
        self.move_to_grasp_timer = None  # Initialize the timer for move_to_grasp if needed    
            
        self.get_logger().info(f"Follow target initialized to: {self.follow_target}") 
        self.get_logger().info(f"Arm stowed state: {self.isStowed}") 
        try:
            self.get_logger().info(f"Grip position at startup: {self.arm.read_gripper_position()}") 
        except:
            self.get_logger().error("Error reading position!")
            self.on_shutdown_()

        rclpy.get_default_context().on_shutdown(self.on_shutdown_)
    
    def on_shutdown_(self):
        # Turn off arm when node turn off
        self.get_logger().info("Shutting down arm node")
        self.arm.set_torque(False)
        self.arm.set_gripper_torque(False)


    # Subscription callbacks

    def follow_target_callback(self, msg):
        """
        Callback function for the follow_target topic.
        Updates the follow_target state based on the received boolean message.
        Args:
            msg (Bool): The incoming message containing the follow_target state.
        """
        self.follow_target = msg.data

    def move_to_grasp_callback(self, msg):
        """Callback function for the arm trajectory topic.
        This function processes incoming PoseStamped messages and generates a trajectory for the arm to follow.
        It checks if the arm is currently unstowed and if so, it generates a trajectory to the specified goal position.
        Args:
            msg (PoseStamped): The incoming PoseStamped message containing the target pose.
        Returns:
            None
        """
        if not self.isStowed :
            # self.arm.write_time(0.500)
            target_pose = msg.pose.position
            self.goal = np.array([target_pose.x, target_pose.y, target_pose.z])           
            joints=self.kinematics.ik(self.goal[0],self.goal[1],self.goal[2])
            if not joints is None:
                self.arm.write_joints(joints)
            self.follow_target = False  # Stop following the target after moving to grasp
            self.move_to_grasp_timer=threading.Timer(0.5,self.arm.close_gripper).start() # Close the gripper after a short delay to ensure the arm is in position
        
    def arm_traj_callback(self, msg):
        """Callback function for the arm trajectory topic.
        This function processes incoming PoseStamped messages and generates a trajectory for the arm to follow.
        It checks if the arm is currently unstowed and if so, it generates a trajectory to the specified goal position.
        Args:
            msg (PoseStamped): The incoming PoseStamped message containing the target pose.
        Returns:
            None
        """
        if not self.isStowed and self.follow_target:
            # self.arm.write_time(1.500)
            target_pose = msg.pose.position
            self.goal = np.array([target_pose.x, target_pose.y, target_pose.z])
            theta = np.arctan2(self.goal[1], self.goal[0]) # Get the angle in radians
            # print("Theta: ",theta, np.cos(theta)*self.tracking_offset, np.sin(theta)*self.tracking_offset)
            r = np.sqrt(self.goal[0]**2 + self.goal[1]**2) # Get the radius from the origin to the goal point
            gx = self.goal[0] - self.tracking_offset * np.cos(theta) # Adjust x to track inward by the offset radius
            gy = self.goal[1] - self.tracking_offset * np.sin(theta) # Adjust y to track inward by the offset radius
            # print("Goal: ",self.goal)
            # print("Tracking inward to: ", gx, gy, self.goal[2])
            joints=self.kinematics.ik(gx,gy,self.goal[2])
            # print("Calculated joints: ", joints)
            if not joints is None:
                self.arm.write_joints(joints)
        # if len(self.stowSteps)==0 and not self.isStowed:
        #     print(["Goal", msg.pose.position])
        #     target_pose = msg.pose.position
        #     self.trajMode="task"
        #     self.goal = np.array([target_pose.x, target_pose.y, target_pose.z])
        #     self.startingMovementTime=self.get_clock().now().nanoseconds*1e-9-self.init_time
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

        ### TODO: NO GUARANTEE THAT ARM IS AT RIGHT POSITION AT ANY POINT
        ### TODO: NO GUARANTEE ARM IS DONE MOVING AT self.endingMovementTime

        # Check if the trajectory is still running    
        # self.get_logger().info(f"Running traj_callback at time: {self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec:.3f} seconds, endTime {self.endingMovementTimeSec}") # Log the current time for debugging
        if self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec < self.endingMovementTimeSec:
            ### TODO: This is dumb - arm has a maximum velocity that this should be based on
            # self.arm.write_time(0.01)#makes sure movements are fast
            t_sec=self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec
            a=0
            b=0
            c=0
            #calculates the points within the trajectory whether joint or task
            for i in range(6):
                a+=self.traj_coeffs[0][i]*t_sec**i
                b+=self.traj_coeffs[1][i]*t_sec**i
                c+=self.traj_coeffs[2][i]*t_sec**i
            #if task mode, calculates the joint angles using inverse kinematics
            if self.trajMode==TrajectoryModes.TASK_SPACE:
                joints = self.kinematics.ik(a,b,c)
            else:
                joints = np.array([a,b,c])
            # print("ABC: ",a,b,c)
            # print("Joints: ",joints)
            if joints is not None and self.kinematics.check_move_safe(joints):
                # print(f"Writing joints: {joints} at time {t_sec:.3f} seconds") # Log the joint positions being written for debugging
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
        status_msg.joint1_position = float(armPos[0])
        status_msg.joint2_position = float(armPos[1])
        status_msg.joint3_position = float(armPos[2])
        
        eepos=self.kinematics.fk(armPos)
        status_msg.ee_x=float(eepos[0][3])
        status_msg.ee_y=float(eepos[1][3])
        status_msg.ee_z=float(eepos[2][3])
        
        armVel=self.arm.read_velocity()
        status_msg.joint1_velocity = float(armVel[0])
        status_msg.joint2_velocity = float(armVel[1])
        status_msg.joint3_velocity = float(armVel[2])

        status_msg.gripper_position = float(self.arm.read_gripper_position()) # Read the gripper position

        
        status_msg.is_stowed = self.isStowed # Include the stow state in the status message
        status_msg.is_tracking = self.follow_target # Include the tracking state in the status message
        status_msg.trajectory_mode = str(self.trajMode) # Include the trajectory mode (task or joint) in the status message
        
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
        
        self.trajMode = TrajectoryModes.TASK_SPACE
        self.goal = np.array([goalx,goaly,goalz])
        # TODO: ROS Time Object has both seconds + nanoseconds as ints. use both please
        self.startingMovementTimeSec = self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec
        self.endingMovementTimeSec = self.startingMovementTimeSec + duration_ms
        currentFk = self.arm.fk(self.arm.read_position())
        currrntPos=np.array([currentFk[0][3],currentFk[1][3],currentFk[2][3]]) # Get the current end-effector position
        self.traj_coeffs=self.kinematics.generate_trajectory(currrntPos, self.goal, start_time=self.startingMovementTimeSec,end_time=self.endingMovementTimeSec)


    def runJointTrajectory(self, goal0, goal1, goal2, duration_ms):
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

        self.trajMode = TrajectoryModes.JOINT_SPACE
        position = self.arm.read_position() # Get the current joint positions
        self.goal = np.array([goal0,goal1,goal2])
        self.get_logger().info(f"Running joint trajectory from: {position} to: {self.goal}") # Log the trajectory for debugging
        
        self.startingMovementTimeSec = self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec
        self.endingMovementTimeSec = self.startingMovementTimeSec + (duration_ms / 1000.0)
        self.traj_coeffs=self.kinematics.generate_trajectory(position, self.goal, start_time=self.startingMovementTimeSec,end_time=self.endingMovementTimeSec)

# TODO: this line I beliee should be there onlz once, otherwise it breaks stuff with too much bandwidth
        self.arm.write_time(0.08)#makes sure movements are fast
        # this function is depracated as it does not behave as expected, when smaller number should be faster, not always
        


    # --- Stow/unstow methods
    def check_if_arm_stowed(self):
        """
        Checks if the arm is in a stowed position.
        This function checks the current joint positions to determine if the arm is in a stowed position.
        Returns:
            bool: True if the arm is stowed, False otherwise.
        """
        joints = self.arm.read_position()
        self.get_logger().info(f"Checking if arm is stowed with joint positions: {joints[0]}") # Log the joint positions for debugging
        if -np.pi/2.1 < joints[0] < np.pi/2:
            return False
        else:
            return True
    
    def run_stowArm_callback(self):
        """Starts the next trajectory step in the stowing process.
        This method processes the next step in the `stowSteps` list. Depending on the type of step, 
        it either runs a task trajectory or a joint trajectory. If the `stowSteps` list becomes empty, 
        the stow state is flipped to reflect the new state.
        """
        # FUNCTION NOW RUNS @ 20 Hz

        if len(self.stowSteps) > 0:                 
# TODO: Get the current time since start???
            current_time_s = self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec 
            # self.get_logger().info(f"Current time in ms: {current_time_s}, Starting: {self.startingMovementTimeSec}, Ending: {self.endingMovementTimeSec}") # Log the timing for debugging
            if self.endingMovementTimeSec - current_time_s > 0.05:
                # If we are more than half a timestep away from the ending time of the previous procedure
                # Don't move to the next one
                return

            self.get_logger().info(f"Running stow step: {self.stowSteps[0]} at time {current_time_s}") # Log the current stow step for debugging
            # Takes out the next stow step
            step=self.stowSteps.pop(0)

# TODO: better state flipping logic than a toggle at the very end, this is error-prone
            if self.stowSteps == []:    #changes stow state on last step
                self.isStowed = not self.isStowed

            #sets trajectory based of step type
            if step[0] == "task": 
                self.runTaskTrajectory(step[1], step[2], step[3], TIME_PER_STOW_STEP)
            elif step[0] == "joint":
                self.runJointTrajectory(step[1], step[2], step[3], TIME_PER_STOW_STEP)
            else:
                self.get_logger().error(f"Unknown stow step type")
    
        # if len(self.stowSteps) > 0:
        #     # If there are still steps left, schedule the next call to continue the stow process
        #     # Use a timer to ensure it runs after the current callback completes
        #     threading.Timer(1, self.run_stowArm_callback).start()
        
    def stowArm(self,req=None,resp=None):
        """Initiates the stowing sequence if the arm is unstowed.

        Args:
            req: The service request (unused).
            resp: The service response (unused).
        """        
        #TODO: better check if arm is stowed and snap to stow position   
        if not self.check_if_arm_stowed():
            self.stowSteps=STOW_ROUTINE_SETPOINTS.copy() 
            
        # self.run_stowArm_callback() # Start the stow process immediately
        return resp
        
        
    def unStowArm(self,req=None,resp=None):
        """Initiates the unstowing sequence if the arm is stowed.

        Args:
            req: The service request (unused).
            resp: The service response (unused).
        """
        self.get_logger().info(f"uns Arm Service is stowed: {self.check_if_arm_stowed()}") # Log the stow check for debugging
        if self.check_if_arm_stowed():
            # self.stowSteps=[["joint", -31*np.pi/32, -np.pi/2.1, np.pi/2.1],
            #                 ["joint", -31*np.pi/32, -np.pi/6, -np.pi/2.1],                        
            #                 ["joint", 0, 0, 0]]
            self.stowSteps=UNSTOW_ROUTINE_SETPOINTS.copy()        
        # self.run_stowArm_callback() # Start the unstow process immediately
        return resp
        
#     ### ------------------ KAY ARCHITECTURE CHANGES ----------------------
#     #              See comment in loop() (below) for more info

#     def update_trajectory_position(self):
#         """Runs the trajectory for the arm based on the current mode (task or joint).
#         This function checks if the current time is within the trajectory duration and updates the arm's position accordingly.
#         If the trajectory is in task mode, it calculates the joint angles using inverse kinematics.
#         If the trajectory is in joint mode, it directly sets the joint angles.
#         Args:
#             None
#         Returns:
#             None
#         """       
#         # arm.write_time(short) incentivises the arm to travel as fast as possible to the 
#         # intermediatetrajectory positions, which allows for smooth following. 
#         # However, loop() updates at 10Hz, 
#         # so any faster would be unnecessary and perhaps beyond the arm's capabilities
#         self.arm.write_time(0.1)

#         time_obj = self.get_clock().now()
#         t_sec = time_obj.nanoseconds*1e-9 #     

# # sort out confusion between sec and ms used here
#         if t_sec > TIME_PER_TRAJ_STEP:
#             self.get_logger().warn(f"Trajectory has exceeded normal motion time! t = {t_sec} seconds")
#             return
        
#         # temp variables which store the trajectory equation output
#         a = 0
#         b = 0
#         c = 0
#         # Reminder to the reader that self.traj_coeffs are the coefficients for a quintic trajectory
#         for i in range(6):
#             a += self.traj_coeffs[0][i] * t_sec**i
#             b += self.traj_coeffs[1][i] * t_sec**i
#             c += self.traj_coeffs[2][i] * t_sec**i
#         #if task mode, calculates the joint angles using inverse kinematics
#         if self.trajMode == TrajectoryModes.TASK_SPACE:
#             joints = self.kinematics.ik(a,b,c)
#         else:
#             # Both TrajectoryModes.JOINT_SPACE and TrajectoryModes.STOW use joint space motions
#             joints = np.array([a,b,c])
#         # print("ABC: ",a,b,c)
#         # print("Joints: ",joints)
#         if joints is not None and self.kinematics.check_move_safe(joints):
#             self.arm.write_joints(joints)

#     def is_in_position(self, target_position: tuple[float, float, float], 
#                                 trajectory_mode: TrajectoryModes = None):
#         """
#             Basic position checking with tolerances variables that don't currently exist :)
#         """
#         if trajectory_mode == None:
#             trajectory_mode = self.trajMode
#         # Read current arm position
#         current_joint_positions = self.arm.read_position()
#         current_arm_position = self.kinematics.fk(current_joint_positions)
#         in_position = True
#         for i in range(len(current_joint_positions)):
#             if trajectory_mode == TrajectoryModes.TASK_SPACE:
#                 # If any of xyz is out of the tolerance, the arm is not in position
#                 if abs(current_arm_position[i] - target_position[i]) > self.pos_tolerance:
#                     in_position = False
#             else:
#                 # Both joint_space and stow modes use joint space motion
#                 # If *any* joint is out of the tolerance, the arm is not in position
#                 if abs(current_joint_positions[i] - target_position[i]) > self.joint_tolerance:
#                     in_position = False
#         return in_position

#     def loop(self):
#         """
#             I don't want to fully implement this before I get the okay from the team
#             but the idea of this function is to be a primary control loop (instead of the 2? 3? that Sam has)
#             which would control all motions (including stow) all in one place
#             The above function (update_trajectory_position) is a cleaned up version of run_traj_callback.
#             Overall, this node is full of edge cases, sloppy code, and race conditions, and needs help
#             I am happy to take charge and do a lot of this myself, I just don't want to completely overstep
#             Feel free to ask me if you have any questions
            
#             Thanks - Kay
#         """
#         # Runs at 10Hz, because commanding the arm to move faster than that is silly

#         # This queue would store a series of setpoints that the arm would loop through,
#         # Set by callbacks asynchronously
#         # Normal go_to_point callbacks would clear the queue before setting a new setpoint
#         # But stow/unstow/more complex motion could be set using this queue setup
#         self.setpoint_queue = []

#         if len(self.setpoint_queue) > 0:
            
#             if self.traj_running:
#                 # Preconditions: 
#                 # - Trajectory is running
#                 # - Trajectory variables are set properly from below (they shouldn't be set elsewhere)

#                 # Handle all updates to the arm position from the trajectory
#                 self.update_trajectory_position()

#             # Get current setpoint target
#             next_setpoint = self.setpoint_queue[0]

#             # Check if arm is in next_setpoint
#             in_position = self.is_in_position(next_setpoint)
            
#             # If the arm is in_position (or no trajectory has begun), iterate to next setpoint
#             if in_position:
#                 # If we've reached a setpoint, pop it from the queue
#                 self.setpoint_queue.pop(0)
#                 # Set trajectory to "not running"
#                 # so it will check if there's a new setpoint for us
#                 self.traj_running = False

#             if not self.traj_running:

#                 if len(self.setpoint_queue) > 0:
#                     new_next_setpoint = self.setpoint_queue[0]
#                     # Set new trajectory coeffs
#                     self.traj_coeffs = self.kinematics.generate_trajectory(
#                         start      = self.arm.read_position(),
#                         end        = new_next_setpoint,
#                         start_vel  = self.arm.read_velocity(),
#                         start_time = 0,
#                         end_time   = TIME_PER_TRAJ_STEP)
#                     self.traj_running = True
#         else:
#             # if the queue is empty, then no trajectory is running
#             self.traj_running = False

def main(args=None):
    # Main entry point
    rclpy.init(args=args)
    node = ArmNode()    

    # t=node.kinematics.fk([0,0,0]) # Test the forward kinematics to ensure it works
    # node.get_logger().info(f'Initialized Arm Node with FK [0,0,0] result: {t}') # Log the FK result for debugging
        
    # dummymsg=PoseStamped()
    # dummymsg.pose.position.x=t[0][3]
    # dummymsg.pose.position.y=t[1][3]
    # dummymsg.pose.position.z=t[2][3]
    # # node.arm_traj_callback(dummymsg) # Call the arm trajectory callback to initialize the arm position
    # node.move_to_grasp_callback(dummymsg) # Call the arm trajectory callback to initialize the arm position
    
    # print(node.arm.read_position())
    # node.stowArm()
    # node.arm.reboot()
    # node.arm.set_torque(True)
    # node.runJointTrajectory(GOAL[0],GOAL[1],GOAL[2],MOVE_TIME)
    # node.runJointTrajectory(0,-np.pi/2,np.pi/2,3000)
    # node.runJointTrajectory(0,0,0,2000)

    # node.arm.set_torque(False)
    # node.runJointTrajectory(-3*np.pi/4,-np.pi/2.1,np.pi/2,3000)
    # node.runJointTrajectory(0,0,-np.pi/2,1000)

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
    
    try:
        import time
        node.arm.stow_gripper()
        node.unStowArm()        
        node.stowArm()
        # node.runJointTrajectory(0, 0, -17*np.pi/32, 3000) # Move to a stowed position
        rclpy.spin(node)
                   
    except Exception as e:
        node.get_logger().error(f"Exception occurred: {str(e)}")
        node.on_shutdown_()    
    
    except KeyboardInterrupt:        
        node.get_logger().info("Keyboard interrupt received, shutting down arm node...")
        node.on_shutdown_()

if __name__ == '__main__':
    main()
