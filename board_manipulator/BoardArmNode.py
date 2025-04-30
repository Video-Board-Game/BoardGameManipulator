import rclpy
import rclpy.context
from rclpy.node import Node
import rclpy.time
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray, Bool
from board_manipulator.DynamixelArm import DynamixelArm  # Import the DynamixelArm class
from board_manipulator.ArmKinematics import ArmKinematics  # Import the ArmKinematics class
from geometry_msgs.msg import PoseStamped, Point, Pose
from arm_interfaces.msg import ArmStatus, ArmCommand  # Import the custom message types
import numpy as np
import time

from enum import Enum
class MoveModes(Enum):
    JOINT_SPACE = "joint"
    ELEVATE = "elevate"  
    GRIP = "grip"  
    

class Grasps(Enum):
    """
    Enum for grasp modes, used to determine how the gripper should behave during a trajectory.
    Each mode is associated with a float value representing the gripper's target position.
    """
    CARD = (-4.7*np.pi)  
    SQUEEZE = (-5.5*np.pi) 
    OPEN = (-2*np.pi)  # Used to open the gripper
    PIECE = (4.061981126323691-np.pi/3)  # Used for piece grasping, not implemented yet
    CLOSE = (4.061981126323691)  # Used to close the gripper
    HALF = (3.5309905631618455)  # Used for half grasping, not implemented yet
    NONE = (0.05)  # Used when no change in grasp is desired

    


class ArmManipulatorNode(Node):
    
    def __init__(self):
        super().__init__('board_arm_node')
        
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

        self.setpoint_queue = []
        self.current_setpoint = None
        self.tolerance = .02

        self.grasp_at_end = Grasps.NONE  # Default grasp mode
        self.move_mode = MoveModes.JOINT_SPACE

        


         # Create a publisher for the arm status
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 10)
        
        # Create a subscriber for arm commands
        self.command_subscriber = self.create_subscription(ArmCommand, 'arm_command', self.receive_arm_command, 10)

        #timers
        self.create_timer(0.01, self.loop)
        self.create_timer(0.01, self.publish_status)  # Publish status every second

        self.armcommand=None
    
    def shutdown(self):
        self.arm.write_gripper(Grasps.NONE.value)  # Ensure the gripper is released on shutdown
        # self.arm.write_elevator_position(0)  # Reset the elevator position
        self.arm.set_arm_torque(False)  # Disable arm torque


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
        status_msg.ee_pos.position.x=float(eepos[0][3])
        status_msg.ee_pos.position.y=float(eepos[1][3])
        status_msg.ee_pos.position.z=float(eepos[2][3])
        status_msg.ee_pos.orientation.z=float(eepos[0][0])      

        status_msg.gripper_position = float(self.arm.read_gripper_position()) # Read the gripper position

        status_msg.move_mode = str(self.move_mode) # Include the trajectory mode (task or joint) in the status message
        # status_msg.grasp_type = str(self.grasp_at_end.label)

        self.status_publisher.publish(status_msg)

    def receive_arm_command(self, msg: ArmCommand):
        moveType = MoveModes(msg.trajectory_mode)
        self.armcommand=msg
        self.get_logger().info(f"Received arm command: {moveType}")
        if moveType == MoveModes.JOINT_SPACE:
            self.move_mode = MoveModes.JOINT_SPACE
            joints = self.kinematics.ik(msg.goal.x, msg.goal.y, msg.alpha,debug=True)
            if not joints is None:
                self.setpoint_queue.append((MoveModes.JOINT_SPACE, joints[0],joints[1],joints[2],msg.movement_time))
        elif moveType == MoveModes.ELEVATE:
            self.move_mode = MoveModes.ELEVATE
            
            self.setpoint_queue.append((MoveModes.ELEVATE,msg.goal.z,0,0, msg.movement_time))
            
        elif moveType == MoveModes.GRIP:
            self.move_mode = MoveModes.GRIP
            self.grasp_at_end = Grasps[msg.grasp_type]
            self.setpoint_queue.append((MoveModes.GRIP, self.grasp_at_end.value, 0, 0, msg.movement_time))
        else:
            self.get_logger().warn(f"Unknown move type: {moveType}")

    def is_in_position(self):
        """
            Basic position checking with tolerances variables that don't currently exist :)
            Function takes in tuple[MoveModes, goal_x, goal_y, goal_z]
            Returns True if the arm is in position, False otherwise
        """
        trajectory_mode = self.current_setpoint[0] 
        target_position = np.array([self.current_setpoint[1], self.current_setpoint[2], self.current_setpoint[3]])  # Extract the target position from the setpoint
        
        if self.current_setpoint[0] == MoveModes.JOINT_SPACE:
            # For joint space mode, we need to check against the current joint positions
            target_position = np.array(self.current_setpoint[1:4])
            current_joint_positions = self.arm.read_arm_position()
            for i in range(len(current_joint_positions)):
                if abs(current_joint_positions[i] - target_position[i]) > self.tolerance:
                    return False
                
            return True  # All joint positions are within tolerance
        elif self.current_setpoint[0] == MoveModes.ELEVATE:
            target_position = self.kinematics.calc_elevator_joint(self.current_setpoint[1])
            current_joint_position = self.arm.read_elevator_position()
            print(f"Checking elevator position: target={target_position}, current={current_joint_position}")
            if abs(current_joint_position - target_position) > self.tolerance/100:
                return False
            else: 
                return True  # Elevator position is within tolerance
        elif self.current_setpoint[0] == MoveModes.GRIP:
            # For grip mode, we check if the gripper is in the desired position
            target_position = self.current_setpoint[1]
            current_gripper_position = self.arm.read_gripper_position()
            if current_gripper_position is None or abs(current_gripper_position - target_position) > self.tolerance:
                return False
            else:
                return True
        elif self.current_setpoint is None:
            self.get_logger().warn("No current setpoint to check position against.")
            return False
        return False
            
        
    
    def loop(self):
        """
        Main loop for the node, processes setpoints and executes movements.
        """
        if not self.setpoint_queue:
            return
        
        if not self.current_setpoint:
            self.current_setpoint = self.setpoint_queue.pop(0)  # Get the next setpoint from the queue

            if self.current_setpoint is not None:
                if self.move_mode == MoveModes.JOINT_SPACE:
                    print(f"Moving arm to joint space setpoint: {self.current_setpoint[1:4]}")
                    self.arm.write_arm_joints(self.current_setpoint[1:4])
                elif self.move_mode == MoveModes.ELEVATE:
                    print(self.armcommand)
                    print("Moving arm to elevator setpoint: ", self.current_setpoint)
                    
                    self.arm.write_elevator_position(self.kinematics.calc_elevator_joint(self.current_setpoint[1]), debug=True)
                elif self.move_mode == MoveModes.GRIP:
                    self.arm.write_gripper(self.current_setpoint[1])
                else:
                    self.get_logger().warn(f"Unknown move mode: {self.move_mode}")

        if self.is_in_position():
            self.get_logger().info(f"Arm is in position for setpoint: {self.current_setpoint}")
            self.current_setpoint=None
            
if __name__ == '__main__':
    rclpy.init()
    node = ArmManipulatorNode()
    # node.arm.write_gripper(0*np.pi+0*Grasps.OPEN.value)  # Open the gripper at the start
    joints = node.arm.read_arm_position()  # Read the current joint positions
    print(f"Current joint positions: {joints}")
    fk=node.kinematics.fk(joints)
    print(f"Forward kinematics result: {fk}")   
    # node.arm.set_elevator_torque(True)  # Enable elevator torque
    armcommand = ArmCommand()
    armcommand.trajectory_mode = MoveModes.GRIP.value
    armcommand.goal.x = fk[0][3]
    armcommand.goal.y = fk[1][3]
    armcommand.goal.z = .027
    armcommand.alpha = 0.0
    armcommand.movement_time = 2.0
    armcommand.grasp_type = "HALF"  # Set the desired grasp type
    node.receive_arm_command(armcommand)  # Simulate receiving a command to move the arm
    # armcommand.grasp_type = Grasps.OPEN.value  # Open the gripper at the start
    
    node.arm.set_arm_torque(False)  # Enable gripper torque
    
   
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.arm.set_arm_torque(False)  # Disable arm torque on shutdown
        node.arm.write_elevator_position(0)
        # node.arm.set_elevator_torque(False)  # Disable elevator torque on shutdown
        node.arm.set_gripper_torque(False)
        pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()