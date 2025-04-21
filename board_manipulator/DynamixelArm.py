import dynamixel_sdk as dxl
"""
DynamixelArm.py
This module provides a Python interface for controlling a robotic arm and gripper powered by Dynamixel motors.
It uses the Dynamixel SDK to communicate with the motors, allowing for operations such as reading and writing
positions, velocities, and other parameters.
Classes:
    DynamixelArm: A class to manage and control a robotic arm and gripper using Dynamixel motors.
Constants:
    ADDR_MX_*: Control table addresses for various motor parameters.
    LEN_MX_*: Data byte lengths for corresponding control table addresses.
    PROTOCOL_VERSION: Protocol version used by the Dynamixel motors.
    BAUDRATE: Communication baud rate for the motors.
    DEVICENAME: Default device name for the communication port.
    TORQUE_ENABLE, TORQUE_DISABLE: Constants to enable or disable motor torque.
    OP_MODE_*: Operating modes for the motors.
    DXL_ZERO_POSITION: Encoder value corresponding to the zero position of the motor.
    DXL_POSITION_FACTOR: Conversion factor from radians to encoder units.
    DXL_MOVING_STATUS_THRESHOLD: Threshold for position error to determine if the motor is moving.
    GRIPPER_OPEN, GRIPPER_CLOSE: Encoder values for the gripper's open and closed positions.
Class Methods:
    __init__(): Initializes the DynamixelArm object, sets up communication, and configures motor IDs.
    bulk_read(address, length, motor_ids): Reads data from multiple motors at a specified address.
    bulk_write(address, length, motor_ids, values): Writes data to multiple motors at a specified address.
    set_torque(enable): Enables or disables torque for the motors.
    write_joints(positions): Writes target positions to the arm joints.
    read_position(): Reads the current positions of the arm joints.
    read_velocity(): Reads the current velocities of the arm joints.
    write_time(time): Sets the profile velocity and acceleration for the motors based on a given time.
    write_gripper(position): Writes a target position to the gripper.
    read_gripper_position(): Reads the current position of the gripper.
    write_gripper_time(time): Sets the profile velocity and acceleration for the gripper based on a given time.
    reboot(): Reboots all motors to reset their state.
    close(): Closes the communication port.
Usage:
    This module is designed to be used as a standalone script or imported into other Python projects.
    Example usage is provided in the `if __name__ == "__main__":` block.
"""
from dynamixel_sdk import COMM_SUCCESS, COMM_TX_FAIL
import functools
import numpy as np
import time
import platform 
import functools
import inspect
# Control table address
ADDR_MX_OPERATING_MODE = 11         # Address for operating mode
ADDR_MX_DRIVER_MODE = 10          # Address for driver mode 
ADDR_MX_TORQUE_ENABLE = 64         # Address for enabling torque
ADDR_MX_GOAL_POSITION = 116         # Address for setting goal position
ADDR_MX_PRESENT_POSITION = 132      # Address for reading current position
ADDR_MX_PRESENT_VELOCITY = 128      # Address for reading current velocity
ADDR_MX_PROFILE_VELOCITY = 112      # Address for setting profile velocity
ADDR_MX_PROFILE_ACCELERATION = 108  # Address for setting profile acceleration
ADDR_MX_PRESENT_CURRENT = 126 # Address for reading current (only available on some models like XL330 and XM430)
ADDR_MX_CURRENT_LIMIT = 38 # Address for setting current limit (only available on some models like XL330 and XM430)
ADDR_MX_PROPORTIONAL_TERM = 84 # Address for setting proportional term
ADDR_MX_INTEGRRAL_TERM = 82 # Address for setting integral term
ADDR_MX_DERIVATIVE_TERM = 80 # Address for setting derivative term

# Data Byte Length
LEN_MX_OPERATING_MODE = 1
LEN_MX_DRIVER_MODE = 1 
LEN_MX_TORQUE_ENABLE = 1
LEN_MX_GOAL_POSITION = 4
LEN_MX_PRESENT_POSITION = 4
LEN_MX_PRESENT_VELOCITY = 4
LEN_MX_PROFILE_VELOCITY = 4
LEN_MX_PROFILE_ACCELERATION = 4
LEN_MX_PRESENT_CURRENT = 2
LEN_MX_CURRENT = 2 
LEN_PID_TERM = 2 


# Protocol version
PROTOCOL_VERSION = 2.0  # Check your Dynamixel model's protocol version

# Default setting
# Dynamixel ID (Change based on your setup)
BAUDRATE = 1000000         # Dynamixel baudrate

DEVICENAME = '/dev/ttyACM0'  # Port (Update according to your system)
WINDOWS_DEVICENAME = 'COM5' # Windows port, if using windows change this to the correct port

TORQUE_ENABLE = 1        # Enable torque
TORQUE_DISABLE = 0       # Disable torque

MOTOR_PROPORTIONAL = 640
MOTOR_INTEGRAL = 50
MOTOR_DERIVATIVE = 2400

OP_MODE_VEL = 1 # Velocity control mode, not used in this case for the arm joints
OP_MODE_POS = 3 # Position control mode, this is used for the arm joints in this case
OP_MODE_EXPOS = 4 # Extended Position Mode, useful if rotating beyond 360 degrees, but not used in this case
OP_MODE_CURPOS = 5 # Current-Based Position mode, this is used for the gripper in this case

DXL_ZERO_POSITION = 2048        # Zero position value
DXL_POSITION_FACTOR = 651.898646904 # Conversion factor from radians to encoder units
DXL_MOVING_STATUS_THRESHOLD = 20  # Threshold for position error
DXL_VELOCITY_FACTOR = 1 / (0.229 * (2 * np.pi) / 60)  # Conversion factor from 1 [rad/s] to 0.229 [rev/min]  
DXL_ACCELERATION_FACTOR =  1 / (214.577 * (2 * np.pi) / 3600)  # Conversion factor from 1 [rad/s^2] to 214.577 [rev/min^2]

DXL_DRIVE_MODE_VELOCITY = 0b00000000 # Velocity profile drive mode
DXL_DRIVE_MODE_TIME = 0b00000100 # Time based profile drive mode, unused in this case

# gripper positions in encoder ticks
GRIPPER_OPEN = 2400
GRIPPER_CLOSE = 300
GRIPPER_STOW = 800 # barely closed for transport, but without risk of overheating

ARM_PROFILE_VELOCITY = 2*np.pi # radians/sec, this is the desired velocity for the arm joints
ARM_PROFILE_ACCELERATION = 2*np.pi # radians/sec^2, this is the desired acceleration for the arm joints
GRIPPER_OPEN_PROFILE_VELOCITY = 2*np.pi # radians/sec, this is the desired velocity for the gripper
GRIPPER_OPEN_PROFILE_ACCELERATION = np.pi # radians/sec^2, this is the desired acceleration for the gripper


class DynamixelArm:
    
    def __init__(self):
        os_name = platform.system()
        deviceName=DEVICENAME
        if os_name == "Windows":
            deviceName = WINDOWS_DEVICENAME
       
        self.port_handler = dxl.PortHandler(deviceName)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.group_bulk_read = dxl.GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = dxl.GroupBulkWrite(self.port_handler, self.packet_handler)

        if not self.port_handler.openPort():
            raise Exception("Failed to open the port")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise Exception("Failed to set the baudrate")

        self.motor_ids = [10, 11, 12]
        self.gripper_ids = [13]

        self.configure_motors()        
        self.apply_reboot_decorator()

    # The following two functions are used to try and avoid TypeErrors which result from an RX call
    # Giving None due to a bad connection with the dynamixel. Rebooting fixes this
    # but the decorator seems to only work sometimes??? Something is slightly off, but its better -K

    def reboot_if_disconnect(self, func):
        """
        Decorator to reboot dynamixels if there is ever a connection error
        """
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except TypeError:
                self.get_logger().error("RX/TX Error with Dynamixels - rebooting and skipping function call...")
                self.reboot()
        return wrapper
        
    def apply_reboot_decorator(self):
        """
        Helper function to apply decorator to all functions in class
        Will reboot arm if there is ever a disconnect
        """
        for k, f in self.__dict__.items():
            if inspect.ismethod(f):
                setattr(self, k, self.reboot_if_disconnect(f))
   
    
    def configure_motors(self, enable_at_end = False):
        """
        Configures the motors by setting their operating modes, driver modes, and profile velocities/accelerations.
        This method is called during initialization to set up the motors for operation.
        """

        # Disable torque to set important register parameters before enabling torque
        self.set_arm_torque(False)
        self.set_gripper_torque(False)

        # Setting operating mode for all motors
        self.bulk_write(ADDR_MX_OPERATING_MODE, LEN_MX_OPERATING_MODE, self.motor_ids, [OP_MODE_POS for i in self.motor_ids]) # regular position control mode for arm joints even though joint 1 can use current based
        self.bulk_write(ADDR_MX_OPERATING_MODE, LEN_MX_OPERATING_MODE, self.gripper_ids, [OP_MODE_CURPOS for i in self.gripper_ids]) # gripper uses current position mode
        
        # Setting driver mode for all motors
        self.bulk_write(ADDR_MX_DRIVER_MODE, LEN_MX_DRIVER_MODE, self.motor_ids, [DXL_DRIVE_MODE_VELOCITY for i in self.motor_ids]) # Time based profile drive mode for arm joints
        self.bulk_write(ADDR_MX_DRIVER_MODE, LEN_MX_DRIVER_MODE, self.gripper_ids, [DXL_DRIVE_MODE_VELOCITY for i in self.gripper_ids]) # Velocity profile drive mode for gripper

        # Set PID control
        self.bulk_write(ADDR_MX_PROPORTIONAL_TERM, LEN_PID_TERM, self.motor_ids, [MOTOR_PROPORTIONAL, MOTOR_PROPORTIONAL, MOTOR_PROPORTIONAL]) 
        self.bulk_write(ADDR_MX_INTEGRRAL_TERM, LEN_PID_TERM, self.motor_ids, [MOTOR_INTEGRAL, MOTOR_INTEGRAL, MOTOR_INTEGRAL])
        self.bulk_write(ADDR_MX_DERIVATIVE_TERM, LEN_PID_TERM, self.motor_ids, [MOTOR_DERIVATIVE, MOTOR_DERIVATIVE, MOTOR_DERIVATIVE])

        # Sets the profile velocity and acceleration for the arm motors 
        self.write_arm_profile(velocity=ARM_PROFILE_VELOCITY, acceleration=ARM_PROFILE_ACCELERATION) # Set profile velocity and acceleration for arm motors, in radians/sec and radians/sec^2 respectively
        # Sets the profile velocity and acceleration for the gripper
        self.write_gripper_profile(velocity=GRIPPER_OPEN_PROFILE_VELOCITY, acceleration=GRIPPER_OPEN_PROFILE_ACCELERATION) # Set profile velocity and acceleration for gripper, in radians/sec and radians/sec^2 respectively

        # write gripper current to medium of the byte
        self.bulk_write(ADDR_MX_CURRENT_LIMIT, LEN_MX_CURRENT, self.gripper_ids, [1200]) # current
        
        self.set_arm_torque(enable_at_end)
        self.set_gripper_torque(enable_at_end)

    # @reboot_if_disconnect
    def bulk_read(self,address,length,motor_ids):
        """
        Reads data from multiple Dynamixel motors in bulk.
        This method reads either 4-byte or 1-byte data from the specified address
        for a list of motor IDs. It handles communication errors and adjusts the 
        read values based on the address and conversion factors.
        Args:
            address (int): The starting address of the data to read.
            length (int): The length of the data to read (1 or 4 bytes).
            motor_ids (list[int]): A list of motor IDs to read data from.
        Returns:
            list[float]: A list of read values from the specified motors, adjusted 
                         based on the address and conversion factor.
                         Returns None if a communication or packet error occurs.
        Raises:
            None: Errors are handled internally with print statements.
        """

        values = []
        for id in motor_ids:
            dxl_present_position, dxl_comm_result, dxl_error = None,None,None
           
            if length == 4:
                dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, id, address)
            elif length == 2:
                dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, id, address)  
                
            else:
                dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, id, address)
            
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Present position error: {self.packet_handler.getRxPacketError(dxl_error)}")
                return
            values.append(dxl_present_position)
        return values
        
    # @reboot_if_disconnect
    def bulk_write(self,address,length,motor_ids,values):
        """
        Performs a bulk write operation to multiple Dynamixel motors.
        This method writes data to the specified address of multiple motors
        using either 1-byte or 4-byte write operations, depending on the length parameter.
        Args:
            address (int): The starting address of the data to write.
            length (int): The length of the data to write (1 or 4 bytes).
            motor_ids (list[int]): A list of motor IDs to which the data will be written.
            values (list[int]): A list of values to write to the corresponding motors.
        Returns:
            None
        Raises:
            Prints an error message if there is a communication error or a Dynamixel error.
        Notes:
            - The method uses the `write4ByteTxRx` function for 4-byte writes and
              the `write1ByteTxRx` function for 1-byte writes.
            - If a communication error or Dynamixel error occurs, the method will
              print the error and terminate further execution.
        """

        for id,value in zip(motor_ids,values):

            if length == 4:
                dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, id, address, value)
            elif length == 2:
                dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, address, value)
            else:
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, id, address, value)

            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
    
    
    ### ARM FUNCTIONS ###
    def set_arm_torque(self, enable: bool):
        """
        Enables or disables the torque for the arm motors.
        Args:
            enable (bool): True to enable torque, False to disable.
        """
        self.bulk_write(ADDR_MX_TORQUE_ENABLE, LEN_MX_TORQUE_ENABLE, self.motor_ids, [enable, enable, enable])

    def read_arm_position(self):
        """"
        Reads the current positions of the arm joints and gripper."
        """
        pos_dxl = self.bulk_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.motor_ids)  
        pos = [(p - DXL_ZERO_POSITION) * direction / (DXL_POSITION_FACTOR) for p, direction in zip(pos_dxl, [1, -1, -1])]
       
        return pos
        
    
    def read_arm_velocity(self):
        """"
        Reads the current velocities of the arm joints."
        """
        vels_dxl = self.bulk_read(ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY, self.motor_ids)  #dynamixel returns 
        vels = [vel * direction / DXL_VELOCITY_FACTOR for vel, direction in zip(vels_dxl, [1, -1, -1])]
        
        return vels
    
    
    def write_arm_joints(self, positions):
        """
        Writes target positions to the arm joints.
        Args:
            positions (list[float]): A list of target positions for the arm joints.
        """
        positions = [int(position*direction*DXL_POSITION_FACTOR+DXL_ZERO_POSITION) for position, direction in zip(positions,[1,-1,-1])]
        self.bulk_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.motor_ids, positions)

    def write_arm_profile(self, velocity, acceleration):
        """
        Sets the profile velocity and acceleration for the motors 
        Args:
            velocity (float): The desired profile velocity in radians per second.
            acceleration (float): The desired profile acceleration in radians per second squared.
        """

        velocity_dxl = int(velocity * DXL_VELOCITY_FACTOR) # Convert velocity to Dynamixel units
        acceleration_dxl = int(acceleration * DXL_ACCELERATION_FACTOR) # Convert acceleration to Dynamixel units

        self.bulk_write(ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY, self.motor_ids, [velocity_dxl, velocity_dxl, velocity_dxl])
        self.bulk_write(ADDR_MX_PROFILE_ACCELERATION, LEN_MX_PROFILE_ACCELERATION, self.motor_ids, [acceleration_dxl, acceleration_dxl, acceleration_dxl])

    ### GRIPPER FUNCTIONS ###
    def set_gripper_torque(self,enable: bool):
        """
        Enables or disables the torque for the gripper.
        Args:
            enable (bool): True to enable torque, False to disable.
        """
        self.bulk_write(ADDR_MX_TORQUE_ENABLE, LEN_MX_TORQUE_ENABLE, self.gripper_ids, [enable])

    def read_gripper_position(self):
        """
        Reads the current position of the gripper."
        """
        pos_dxl = self.bulk_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.gripper_ids)[0]
        return pos_dxl/DXL_POSITION_FACTOR - DXL_ZERO_POSITION # Convert from Dynamixel units to radians
    
    def read_gripper_goal_position(self):
        """
        Reads the goal position of the gripper."
        """
        pos_dxl = self.bulk_read(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.gripper_ids)[0]
        return pos_dxl/DXL_POSITION_FACTOR - DXL_ZERO_POSITION # Convert from Dynamixel units to radians

    def write_gripper(self, position):
        """
        Writes a target position to the gripper. 
        Args:
            position (int): The target position for the gripper.
        """
        self.bulk_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.gripper_ids, [position])

    def read_gripper_current(self):
        """
        Reads the current from the gripper (only available on some models like XL330 and XM430).
        Returns:
            int: The current value in mA if successful, None otherwise.
        """
        current_dxl = self.bulk_read(ADDR_MX_PRESENT_CURRENT, LEN_MX_PRESENT_CURRENT, self.gripper_ids)
        return current_dxl[0]

    def open_gripper(self):
        
        """
        Opens the gripper by writing the open position to the gripper.
        This is a convenience method for writing the gripper's open position.
        """
        self.write_gripper(GRIPPER_OPEN)
    
    def close_gripper(self):

        """
        Closes the gripper by writing the close position to the gripper.
        This is a convenience method for writing the gripper's closed position.
        """
        self.write_gripper(GRIPPER_CLOSE)

    def stow_gripper(self):
        """
        Mover the gripper by writing the stow position to the gripper.
        This is a convenience method for writing the gripper's swot position.
        """
        self.write_gripper(GRIPPER_STOW)
        

    def write_gripper_profile(self, velocity, acceleration):
        """
        Sets the profile velocity and acceleration for the gripper.
        Args:
            velocity (float): The desired profile velocity in radians per second.
            acceleration (float): The desired profile acceleration in radians per second squared.
        """
        velocity_dxl = int(velocity * DXL_VELOCITY_FACTOR) # Convert velocity to Dynamixel units
        acceleration_dxl = int(acceleration * DXL_ACCELERATION_FACTOR) # Convert acceleration to Dynamixel units

        self.bulk_write(ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY, self.gripper_ids, [velocity_dxl])
        self.bulk_write(ADDR_MX_PROFILE_ACCELERATION, LEN_MX_PROFILE_ACCELERATION, self.gripper_ids, [acceleration_dxl])
    
    def reboot(self):
        """
        Reboots all motors to reset their state.
        This method sends a reboot command to all motors in the motor_ids list.
        """
        for id in self.motor_ids:
            dxl.reboot(self.port_handler, PROTOCOL_VERSION, id)
            # self.packet_handler.reboot(self.port_handler,id)
        # self.packet_handler.reboot(self.port_handler,self.gripper_ids[0])
        dxl.reboot(self.port_handler, PROTOCOL_VERSION, self.gripper_ids[0])
        
    def close(self):
        self.port_handler.closePort()

if __name__ == "__main__":
    
    arm = DynamixelArm()
    print(arm.read_arm_position())
    arm.write_arm_joints([0,np.pi/2,-np.pi/2])