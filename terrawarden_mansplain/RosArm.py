import rclpy
import rclpy.context
from rclpy.node import Node
import rclpy.time
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray, Bool
from terrawarden_mansplain.DynamixelArm import DynamixelArm  # Import the DynamixelArm class
from terrawarden_mansplain.ArmKinematics import ArmKinematics  # Import the ArmKinematics class
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from terrawarden_interfaces.msg import ArmStatus, ArmCommand
import numpy as np

# TODO: TEST RX ERRORS WITH TASK SPACE TRAJECTORY WHEN ARM AT PI/2

from enum import Enum
class TrajectoryModes(Enum):
    JOINT_SPACE = "joint"
    TASK_SPACE = "task"
    LIVE_TRACK = "track"

class Grasps(Enum):
    """
    Enum for grasp modes, used to determine how the gripper should behave during a trajectory.
    """
    OPEN = "open"
    CLOSE = "close"
    STOW = "stow" 
    NONE = "none"  # Used when no change in grasp is desired


GOAL = np.array([0,0,0])
GRASP_CURRENT_THRESHOLD = 300  # Threshold for gripper current to consider it grasped (this is arbitrary and should be tuned)

# Time in milliseconds for each step, used to space out multi-step routines
TIME_PER_TRAJ_STEP = 1.2
TIME_MARGIN = 0.0  # seconds, margin for trajectory timing

TIME_PER_STOW_STEP = 0.9  # tuned for not much drone jerk
STOW_JOINT_MOVE_TOLERANCE = 0.05 # radians, tolerance for stow joint positions, this is to ensure the arm is in a safe stow position
STOW_JOINT_DETECT_TOLERANCE = tuple([8*np.pi/32, 2*np.pi/32, 4*np.pi/32]) # radians, half-tolerance for joints 2 and 3 to detect if arm is in folded position

# routines check whether the arm is folded or not, and skip to the last two stow steps if it is
STOW_ROUTINE_SETPOINTS = [ 
    # Stow routine setpoints (type, val1, val2, val3)
    (TrajectoryModes.JOINT_SPACE, 0, 0, -17*np.pi/32,TIME_PER_STOW_STEP,STOW_JOINT_MOVE_TOLERANCE, Grasps.STOW), 
    (TrajectoryModes.JOINT_SPACE, -31*np.pi/32, 0, -17*np.pi/32,TIME_PER_STOW_STEP,STOW_JOINT_MOVE_TOLERANCE), # rotate straight down so as not to obstruct the drone optical flow
    (TrajectoryModes.JOINT_SPACE, -30*np.pi/32, -17*np.pi/32, 15*np.pi/32, TIME_PER_STOW_STEP,STOW_JOINT_MOVE_TOLERANCE),  # fold upwards base joint back
    (TrajectoryModes.JOINT_SPACE, -19*np.pi/32, -17*np.pi/32, 15*np.pi/32, TIME_PER_STOW_STEP,STOW_JOINT_MOVE_TOLERANCE),  
    (TrajectoryModes.JOINT_SPACE, -19*np.pi/32, -14*np.pi/32, 15*np.pi/32, TIME_PER_STOW_STEP,STOW_JOINT_MOVE_TOLERANCE, Grasps.STOW)   # Final position resting folded on the drone leg
]
UNSTOW_ROUTINE_SETPOINTS = [
    # Unstow routine setpoints (type, val1, val2, val3)
    (TrajectoryModes.JOINT_SPACE, -19*np.pi/32, -14*np.pi/32, 15*np.pi/32,TIME_PER_STOW_STEP, STOW_JOINT_MOVE_TOLERANCE), # stow position, then unstow basically backwards
    (TrajectoryModes.JOINT_SPACE, -19*np.pi/32, -17*np.pi/32, 15*np.pi/32,TIME_PER_STOW_STEP / 2, STOW_JOINT_MOVE_TOLERANCE),  
    (TrajectoryModes.JOINT_SPACE, -31*np.pi/32, -17*np.pi/32, 15*np.pi/32,TIME_PER_STOW_STEP / 2, STOW_JOINT_MOVE_TOLERANCE),        
    (TrajectoryModes.JOINT_SPACE, -31*np.pi/32, 0*np.pi/32, -17*np.pi/32,TIME_PER_STOW_STEP * 2, STOW_JOINT_MOVE_TOLERANCE, Grasps.OPEN),
    (TrajectoryModes.JOINT_SPACE, 0, 0, -17*np.pi/32,TIME_PER_STOW_STEP, STOW_JOINT_MOVE_TOLERANCE), # make the massive swing slower    
    (TrajectoryModes.JOINT_SPACE, 0, 0, 0,TIME_PER_STOW_STEP, STOW_JOINT_MOVE_TOLERANCE)  # Final position to ensure the arm is fully unstowed in "L" shape forward
]


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        
        self.get_logger().info('Initializing Arm Node...')
        
        # Initialization
        self.arm = DynamixelArm()
        self.kinematics = ArmKinematics()
        self.arm.set_arm_torque(True)
        self.arm.set_gripper_torque(True)
        
        
        #storing node start time to shrink the times stored to avoid numerical errors
        self.init_time_sec=self.get_clock().now().nanoseconds*1e-9
        self.start_move_time_sec=-1 #time when the trajectory starts
        self.end_move_time_sec=-1 #time when the trajectory ends

        self.traj_coeffs=np.zeros([3,6])
        self.goal = Point()  #current goal position XYZ
        self.trajMode = TrajectoryModes.TASK_SPACE
        self.traj_running = False  # Flag to indicate if a trajectory is currently running
        self.setpoint_queue = [] # Queue to hold setpoints for the arm to follow, this allows for chaining multiple trajectories together
        self.tolerance = 0.005  # Tolerance for task space position checking (meters)
        self.isStowed=False
        if self.check_if_arm_stowed(): # Check if the arm is in a stowed position at startup
            # is stowed, snap back to position
            self.isStowed = True

        self.grasp_at_end_move = Grasps.NONE # Flag to indicate if we should grasp at the end of a move, default to False
        self.arm_command = None # Initialize to None, will be set in the callback

        # Publishers
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 10)  # Publisher for arm status
        self.pos_publisher = self.create_publisher(PointStamped, 'ee_pos', 10)  # Publisher for arm pos for rviz


        # Services
        self.create_service(Empty, 'stow_arm', self.stowArm) #service to stow the arm
        self.create_service(Empty, 'unstow_arm', self.unStowArm) #service to unstow the arm


        # Timers
        self.create_timer(0.01, self.loop)  # Timer to run the trajectory callback
        self.create_timer(0.01, self.publish_status)  # Timer to publish arm status
        
        # Subscriptions
        self.create_subscription(
            ArmCommand,
            'move_arm_command',  # Topic to receive arm commands
            self.arm_command_callback,
            10  # QoS profile
        )
            
        self.create_subscription(
            Bool,
            'force_grasp',
            self.force_grasp_callback,
            10
        )
        
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
        self.arm.set_arm_torque(False)
        self.arm.set_gripper_torque(False)

    # Subscription callbacks

    def arm_command_callback(self, msg: ArmCommand):
        self.arm_command = msg  # Store the latest arm command
        #one remaining setpoint in the queue, means that the arm is in tracking or at the end of unstow which is safe to move
        if len(self.setpoint_queue)<2 and not self.check_if_arm_stowed():
            self.goal = msg.goal            
            if msg.trajectory_mode == TrajectoryModes.TASK_SPACE.value:
                # Generate a task trajectory to the new goal
                # Clear the queue and set the new goal
                if msg.grasp_at_end_of_movement:
                    self.setpoint_queue = [(TrajectoryModes.TASK_SPACE, self.goal.x, self.goal.y, self.goal.z, msg.movement_time, msg.tolerance, Grasps.CLOSE)]  
                else:
                    self.setpoint_queue = [(TrajectoryModes.TASK_SPACE, self.goal.x, self.goal.y, self.goal.z, msg.movement_time, msg.tolerance)]   
                self.get_logger().info(f"New goal set: {self.goal}")
            elif msg.trajectory_mode == TrajectoryModes.JOINT_SPACE.value:
                # Generate a joint trajectory to the new goal
                goal_joints = self.kinematics.ik(self.goal.x, self.goal.y, self.goal.z)  # Calculate the joint angles for the goal position
                
                # Clear the queue and set the new goal
                if goal_joints is None:
                    self.get_logger().error(f"Cannot calculate joint angles for the given goal position {self.goal}, aborting trajectory.")
                    return
                if msg.grasp_at_end_of_movement:
                    # If grasp at end of movement is requested, add the grasp mode to the setpoint queue
                    self.setpoint_queue = [(TrajectoryModes.JOINT_SPACE, goal_joints[0], goal_joints[1], goal_joints[2], msg.movement_time, msg.tolerance, Grasps.CLOSE)]
                else:
                    # Otherwise, just set the joint trajectory without grasping 
                    self.setpoint_queue = [(TrajectoryModes.JOINT_SPACE, goal_joints[0], goal_joints[1], goal_joints[2], msg.movement_time, msg.tolerance)] 
                self.get_logger().info(f"New joint goal set: {goal_joints} with movement time: {msg.movement_time}")
            elif msg.trajectory_mode == TrajectoryModes.LIVE_TRACK.value:
                # Generate a joint trajectory to the new goal
                goal_joints = self.kinematics.ik(self.goal.x, self.goal.y, self.goal.z)  # Calculate the joint angles for the goal position
                if goal_joints is None:
                    self.get_logger().error(f"Cannot calculate joint angles for the given goal position {self.goal}, aborting trajectory.")
                    return
                if msg.grasp_at_end_of_movement:
                    # If grasp at end of movement is requested, add the grasp mode to the setpoint queue
                    self.setpoint_queue = [(TrajectoryModes.LIVE_TRACK, goal_joints[0], goal_joints[1], goal_joints[2], msg.movement_time, msg.tolerance, Grasps.CLOSE)]
                else:
                    # Otherwise, just set the joint trajectory without grasping 
                    self.setpoint_queue = [(TrajectoryModes.LIVE_TRACK, goal_joints[0], goal_joints[1], goal_joints[2], msg.movement_time, msg.tolerance)]
            else:
                self.get_logger().error(f"Invalid trajectory mode: {msg.trajectory_mode}. Cannot set new trajectory.")
                return
            self.get_logger().info(f"Current setpoint queue: {self.setpoint_queue}")
            self.traj_running=False
                      
    def force_grasp_callback(self, msg: Bool):
        """
        Callback for the force_grasp topic which forces the gripper open or closed.
        Args:
            msg: The Bool message indicating whether to open grasp (True) or not (False).
        Returns:
            None
        """
        if msg.data: 
            self.arm.open_gripper()  # Ensure the gripper is open before setting the flag
        else:
            self.arm.close_gripper()
    

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
        armPos=self.arm.read_arm_position()
        status_msg.joint1_position = float(armPos[0])
        status_msg.joint2_position = float(armPos[1])
        status_msg.joint3_position = float(armPos[2])
        
        eepos=self.kinematics.fk(armPos)
        status_msg.ee_pos.x=float(eepos[0][3])
        status_msg.ee_pos.y=float(eepos[1][3])
        status_msg.ee_pos.z=float(eepos[2][3])
        pnt_stmp=PointStamped()
        pnt_stmp.point=status_msg.ee_pos
        pnt_stmp.header.frame_id="drone_frame"
        self.pos_publisher.publish(pnt_stmp)
        
        armVel=self.arm.read_arm_velocity()
        status_msg.joint1_velocity = float(armVel[0])
        status_msg.joint2_velocity = float(armVel[1])
        status_msg.joint3_velocity = float(armVel[2])

        status_msg.gripper_position = float(self.arm.read_gripper_position()) # Read the gripper position

        
        status_msg.is_stowed = self.check_if_arm_stowed() # Include the stow state in the status message
        status_msg.is_tracking = self.traj_running # Include the tracking state in the status message
        status_msg.trajectory_mode = str(self.trajMode) # Include the trajectory mode (task or joint) in the status message
        status_msg.grasping_object = self.detect_if_grasped()  # Detect if the gripper has grasped an object based on the gripper current

        self.status_publisher.publish(status_msg)



    # --- Stow/unstow methods
    def check_if_arm_stowed(self):
        """
        Checks if the arm is in a stowed position.
        This function checks the current joint positions to determine if the arm is in a stowed position.
        Returns:
            bool: True if the arm is stowed, False otherwise.
        """
        joints = self.arm.read_arm_position()   
        # get the stow position from the last setpoint in the stow routine
        stow_pos_joints = STOW_ROUTINE_SETPOINTS[-1][1:4]
        if abs(joints[1] - stow_pos_joints[1]) < 2*abs(STOW_JOINT_DETECT_TOLERANCE[1]) and abs(joints[2] - stow_pos_joints[2]) < 2*abs(STOW_JOINT_DETECT_TOLERANCE[2]):
            return True
        else:
            return False
        
        # self.get_logger().info(f"Checking if arm is stowed with joint positions: {joints[0]}") # Log the joint positions for debugging
        # if -np.pi/2.1 < joints[0] < np.pi/2:
        #     return False
        # else:
        #     return True
    
    
    def check_if_arm_folded(self):
        """
        Checks if the arm is in a folded position.
        This function checks the current joint positions to determine if the arm is in a folded position.
        Returns:
            bool: True if the arm is folded, False otherwise.
        """
        joints = self.arm.read_arm_position()   
        # get the stow position from the last setpoint in the stow routine
        stow_pos_joints = STOW_ROUTINE_SETPOINTS[-1][1:4]
        if abs(joints[1] - stow_pos_joints[1]) < abs(STOW_JOINT_DETECT_TOLERANCE[1]) and abs(joints[2] - stow_pos_joints[2]) < abs(STOW_JOINT_DETECT_TOLERANCE[2]):
            return True
        else:
            return False

    def stowArm(self,req=None,resp=None):
        """Initiates the stowing sequence if the arm is unstowed.
            Skips directly to the last two steps to snap the arm back to the stow position if the arm is folded

        Args:
            req: The service request (unused).
            resp: The service response (unused).
        """        
        if not self.check_if_arm_stowed():
            self.setpoint_queue=STOW_ROUTINE_SETPOINTS.copy() 
            self.traj_running = False # Ensure loop will recalcoefficients for the next trajectory step
            
        # if arm is stowed, check if it is folded on the leg and tighten it
        elif self.check_if_arm_folded():
                self.get_logger().info("Arm is folded, skipping to last stow step")
                self.setpoint_queue=STOW_ROUTINE_SETPOINTS[-2:].copy()  # Skip to the last two steps of the stow routine            
        return resp
        
        
    def unStowArm(self,req=None,resp=None):
        """Initiates the unstowing sequence if the arm is stowed.

        Args:
            req: The service request (unused).
            resp: The service response (unused).
        """
        self.get_logger().info(f"uns Arm Service is stowed: {self.check_if_arm_stowed()}") # Log the stow check for debugging
        if self.check_if_arm_stowed():           
            self.setpoint_queue=UNSTOW_ROUTINE_SETPOINTS.copy()                  
            self.traj_running = False # Ensure loop will recalcoefficients for the next trajectory step
       
        return resp
    
    def detect_if_grasped(self):
        """
        Detects if the gripper has grasped an object based on the gripper position.
        This is a simple heuristic that can be improved with actual force/torque sensors.
        Returns:
            bool: True if the gripper is closed (indicating a grasp), False otherwise.
        """
        gripper_cur = self.arm.read_gripper_current()
        gripper_pos = self.arm.read_gripper_position()
        gripper_goal = self.arm.read_gripper_goal_position()
        
        if gripper_pos >= gripper_goal and gripper_cur > GRASP_CURRENT_THRESHOLD:
            return True

        return False


    def update_trajectory_position(self):
        """Runs the trajectory for the arm based on the current mode (task or joint).
        This function checks if the current time is within the trajectory duration and updates the arm's position accordingly.
        If the trajectory is in task mode, it calculates the joint angles using inverse kinematics.
        If the trajectory is in joint mode, it directly sets the joint angles.
        Args:
            None
        Returns:
            None
        """       
        # arm.write_time(short) incentivises the arm to travel as fast as possible to the 
        # intermediatetrajectory positions, which allows for smooth following. 

        time_obj = self.get_clock().now()
        t_sec = time_obj.nanoseconds*1e-9 - self.start_move_time_sec - self.init_time_sec # ns to s   

        # sort out confusion between sec and ms used here
        if t_sec > self.end_move_time_sec + TIME_MARGIN:
            self.get_logger().warn(f"Trajectory has exceeded normal motion time! t = {t_sec} seconds")
            return
        
        # temp variables which store the trajectory equation output
        a = 0
        b = 0
        c = 0
        # Reminder to the reader that self.traj_coeffs are the coefficients for a quintic trajectory
        for i in range(6):
            a += self.traj_coeffs[0][i] * t_sec**i
            b += self.traj_coeffs[1][i] * t_sec**i
            c += self.traj_coeffs[2][i] * t_sec**i

        #if task mode, calculates the joint angles using inverse kinematics
        if self.trajMode == TrajectoryModes.TASK_SPACE:
            joints = self.kinematics.ik(a,b,c)
        else:
            # Both TrajectoryModes.JOINT_SPACE and TrajectoryModes.STOW use joint space motions
            joints = np.array([a,b,c])
            
        # print("ABC: ",a,b,c)
        print("Current Joints", self.arm.read_arm_position())
        print("SetPoint Joints: ", joints)
        print("Current Traj Coeffs: ", self.traj_coeffs)
        print("Current Traj Time: ", t_sec)
        
        if joints is not None and self.kinematics.check_move_safe(joints):
            self.arm.write_arm_joints(joints)

    def is_in_position(self, setpoint: tuple[TrajectoryModes, float, float, float]):
        """
            Basic position checking with tolerances variables that don't currently exist :)
            Function takes in tuple[TrajectoryModes, goal_x, goal_y, goal_z]
            Returns True if the arm is in position, False otherwise
        """
        trajectory_mode = setpoint[0] 
        target_position = np.array([setpoint[1], setpoint[2], setpoint[3]])  # Extract the target position from the setpoint
        
        if trajectory_mode == None:
            trajectory_mode = self.trajMode
            
        # Read current arm position
        current_joint_positions = self.arm.read_arm_position()
        current_arm_position = self.kinematics.fk(current_joint_positions)
        current_arm_position = [current_arm_position[0][3],current_arm_position[1][3],current_arm_position[2][3]]
        in_position = True
        
        for i in range(len(current_joint_positions)):
            if trajectory_mode == TrajectoryModes.TASK_SPACE:
                # If any of xyz is out of the tolerance, the arm is not in position
                if abs(current_arm_position[i] - target_position[i]) > self.tolerance:
                    return False
            else:
                # Both joint_space and stow modes use joint space motion
                # If *any* joint is out of the tolerance, the arm is not in position
                if abs(current_joint_positions[i] - target_position[i]) > self.tolerance:
                    return False
        return in_position
    
    def setNewTrajectory(self, setpoint: tuple[TrajectoryModes, float, float, float, float, float, Grasps] ):
        """
            Sets a new trajectory based on the provided setpoint.
            This function will clear the current trajectory and set a new one based on the input setpoint.
            Args:
                setpoint: A tuple containing (movement type, val1, val2, val3, movement_time_sec, tolerance)
            Returns:
                None
        """
        print(f"Setpoint: {setpoint}")
        movement_type = setpoint[0]  # Extract the movement type from the setpoint
        move_time_sec = setpoint[4] 
        if setpoint[4] <= 0:            # Default to MOVE_TIME if not provided
            move_time_sec = TIME_PER_TRAJ_STEP
                            
        if len(setpoint) > 5:   # Check if tolerance is provided
            self.tolerance = setpoint[5] 
            
        self.grasp_at_end_move = Grasps.NONE  # Default to no grasp at end of move
        if len(setpoint) > 6 and isinstance(setpoint[6], Grasps):  
            self.grasp_at_end_move = setpoint[6]      
        
        self.start_move_time_sec = self.get_clock().now().nanoseconds*1e-9 - self.init_time_sec
        self.end_move_time_sec = move_time_sec
        
        current_joints_pos = self.arm.read_arm_position()  # Get the current joint positions
        current_joint_vel = self.arm.read_arm_velocity() # Get the current joint velocities
        
        # print(f"Setpoint: {setpoint}")

        if movement_type == TrajectoryModes.LIVE_TRACK:
            if self.kinematics.check_move_safe(setpoint[1:4]):
                self.setpoint_queue = []
                self.get_logger().info(f"Live tracking at: {self.goal}")
                self.arm.write_arm_joints(setpoint[1:4])
            else:
                self.get_logger().info(f"Attempted to send arm to illegal position at: {self.goal}")

        elif movement_type == TrajectoryModes.TASK_SPACE:
            # Generate a task trajectory to the new goal
            self.goal = Point(x=setpoint[1], y=setpoint[2], z=setpoint[3])  # Update the goal from the setpoint
            current_pos = self.kinematics.fk(current_joints_pos)  # Get the current end-effector position
            current_pos = np.array([current_pos[0][3], current_pos[1][3], current_pos[2][3]])  # Extract the position 
            current_jac = self.kinematics.vk(current_joints_pos)  # Get the Jacobian for the current joint positions
            current_ee_vel = (current_jac @ np.vstack(current_joint_vel)).flatten() if current_jac is not None else np.array([0, 0, 0])
            self.traj_coeffs = self.kinematics.generate_trajectory(
                start=current_pos, 
                end=setpoint[1:4], 
                start_vel=current_ee_vel,  # Use current velocity for smoother motion
                start_time=0.0,
                end_time=self.end_move_time_sec
            )
            self.trajMode = TrajectoryModes.TASK_SPACE  # Set the trajectory mode to task space
            self.get_logger().info(f"New task goal set: {self.goal} with movement time: {move_time_sec}")  # Log the new goal and movement time
            
        elif movement_type == TrajectoryModes.JOINT_SPACE:
            goal_joints = np.array([setpoint[1], setpoint[2], setpoint[3]])  # Calculate the joint angles for the goal position
            # fk_pos = self.kinematics.fk(goal_joints)  # Ensure the goal joints are valid by checking FK
            # self.goal = np.array([fk_pos[0][3], fk_pos[1][3], fk_pos[2][3]])  # Update the goal based on FK to ensure it's correct
            # Generate a joint trajectory to the new goal
            self.traj_coeffs = self.kinematics.generate_trajectory(
                start=current_joints_pos, 
                end=goal_joints,
                start_vel=current_joint_vel,  # Use current velocity for smoother motion
                start_time=0.0,
                end_time=self.end_move_time_sec
            )
            self.trajMode = TrajectoryModes.JOINT_SPACE
            self.get_logger().info(f"New joint goal set: {goal_joints} with movement time: {move_time_sec}")
        else:
            self.get_logger().error(f"Invalid trajectory mode: {movement_type}. Cannot set new trajectory.")
            
        self.traj_running = True  # Set trajectory running to true, so the loop will process it

    def execute_grasp_at_end_move(self):
        if self.grasp_at_end_move == Grasps.CLOSE:
            """
                Execute a grasp at the end of the move if requested
            """
            self.arm.close_gripper()
            self.get_logger().info("Gripper closed at end of move.")
        elif self.grasp_at_end_move == Grasps.OPEN:
            """
                Execute an open grasp at the end of the move if requested
            """
            self.arm.open_gripper()
            self.get_logger().info("Gripper opened at end of move.")
        elif self.grasp_at_end_move == Grasps.STOW:
            """
                Execute a stow grasp at the end of the move if requested
                This is a special case for stowing the gripper at the end of a trajectory
            """
            self.arm.stow_gripper()
            self.get_logger().info("Gripper stowed at end of move.")
        

    def loop(self):
        """
            I don't want to fully implement this before I get the okay from the team
            but the idea of this function is to be a primary control loop (instead of the 2? 3? that Sam has)
            which would control all motions (including stow) all in one place
            The above function (update_trajectory_position) is a cleaned up version of run_traj_callback.
            Overall, this node is full of edge cases, sloppy code, and race conditions, and needs help
            I am happy to take charge and do a lot of this myself, I just don't want to completely overstep
            Feel free to ask me if you have any questions
            
            Thanks - Kay
        """
        # Runs at 100Hz, because commanding the arm to move faster than that is silly

        # This queue would store a series of setpoints that the arm would loop through,
        # Set by callbacks asynchronously
        # Normal go_to_point callbacks would clear the queue before setting a new setpoint
        # But stow/unstow/more complex motion could be set using this queue setup

        if len(self.setpoint_queue) > 0:
            
            # Get current setpoint target
            next_setpoint = self.setpoint_queue[0]

            # Check if arm is in next_setpoint
            in_position = self.is_in_position(next_setpoint)
            
            # If the arm is in_position (or no trajectory has begun), iterate to next setpoint
            if in_position:
                self.execute_grasp_at_end_move() # Execute grasp if needed at the end of the move
                # If we've reached a setpoint, pop it from the queue
                self.setpoint_queue.pop(0)
                # Set trajectory to "not running"
                # so it will check if there's a new setpoint for us
                self.traj_running = False

                # Preconditions: 
                # - Trajectory is running
                # - Trajectory variables are set properly from below (they shouldn't be set elsewhere)
            if self.traj_running:
                # Handle all updates to the arm position from the trajectory
                self.update_trajectory_position()
                
            else:
                if len(self.setpoint_queue) > 0:
                    new_next_setpoint = self.setpoint_queue[0]
                    self.setNewTrajectory(new_next_setpoint) # Set a new trajectory based on the next setpoint in the queue
        else:
            # if the queue is empty, then no trajectory is running
            self.traj_running = False
    

def main(args=None):
    # Main entry point
    rclpy.init(args=args)
    node = ArmNode()    
    
    try:
        # import time
        # time.sleep(1)    
    # node.unStowArm()        
        # node.stowArm()
        
        #manually send it to a test position using trajectory:
        # node.setpoint_queue.append(  (TrajectoryModes.JOINT_SPACE, 0, 0, 0, TIME_PER_STOW_STEP, STOW_JOINT_TOLERANCE)   )        
        # node.setpoint_queue.append(  (TrajectoryModes.JOINT_SPACE, -np.pi/4, 0, 0, TIME_PER_STOW_STEP, STOW_JOINT_TOLERANCE)   )

        rclpy.spin(node)
                   
    except Exception as e:
        node.get_logger().error(f"Exception occurred: {str(e)}")
        node.on_shutdown_()    
    
    except KeyboardInterrupt:        
        node.get_logger().info("Keyboard interrupt received, shutting down arm node...")
        node.on_shutdown_()

if __name__ == '__main__':
    main()
