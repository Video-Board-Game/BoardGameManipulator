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
    DXL_CONVERTION_FACTOR: Conversion factor from radians to encoder units.
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
import numpy as np
import time
import platform 
# Control table address
ADDR_MX_OPERATING_MODE = 11         # Address for operating mode
ADDR_MX_TORQUE_ENABLE = 64         # Address for enabling torque
ADDR_MX_GOAL_POSITION = 116         # Address for setting goal position
ADDR_MX_PRESENT_POSITION = 132      # Address for reading current position
ADDR_MX_PRESENT_VELOCITY = 128      # Address for reading current velocity
ADDR_MX_PROFILE_VELOCITY = 112      # Address for setting profile velocity
ADDR_MX_PROFILE_ACCELERATION = 108  # Address for setting profile acceleration
ADDR_MX_CURRENT =126

# Data Byte Length
LEN_MX_OPERATING_MODE = 1
LEN_MX_TORQUE_ENABLE = 1
LEN_MX_GOAL_POSITION = 4
LEN_MX_PRESENT_POSITION = 4
LEN_MX_PRESENT_VELOCITY = 4
LEN_MX_PROFILE_VELOCITY = 4
LEN_MX_PROFILE_ACCELERATION = 4
LEN_MX_CURRENT = 2


# Protocol version
PROTOCOL_VERSION = 2.0  # Check your Dynamixel model's protocol version

# Default setting
# Dynamixel ID (Change based on your setup)
BAUDRATE = 1000000         # Dynamixel baudrate

DEVICENAME = '/dev/ttyACM0'  # Port (Update according to your system)

TORQUE_ENABLE = 1        # Enable torque
TORQUE_DISABLE = 0       # Disable torque

OP_MODE_VEL = 1
OP_MODE_POS = 3
OP_MODE_EXPOS = 4

DXL_ZERO_POSITION = 2048        # Zero position value
DXL_CONVERTION_FACTOR = 651.898646904 # Convertion factor from radians to encoder units
DXL_MOVING_STATUS_THRESHOLD = 20  # Threshold for position error

GRIPPER_OPEN = 1024
GRIPPER_CLOSE = 3072

class DynamixelArm:
    def __init__(self):
        os_name = platform.system()
        deviceName="/dev/ttyACM0"
        if os_name == "Windows":
            deviceName = "COM5"
       
        
        self.port_handler = dxl.PortHandler(deviceName)
        self.packet_handler = dxl.PacketHandler(2.0)
        self.group_bulk_read = dxl.GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = dxl.GroupBulkWrite(self.port_handler, self.packet_handler)

        if not self.port_handler.openPort():
            raise Exception("Failed to open the port")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise Exception("Failed to set the baudrate")

        self.motor_ids = [10, 11, 12]
        self.gripper_ids = [13]

        self.bulk_write(108,4,self.gripper_ids,[200])
        self.bulk_write(112,4,self.gripper_ids,[200])
        self.gripper_open = 2950
        self.gripper_close = 1750
        self.gripper_coke = 2572

        # self.bulk_write(ADDR_MX_OPERATING_MODE,LEN_MX_OPERATING_MODE,self.motor_ids,[OP_MODE_EXPOS for i in self.motor_ids])

   

    

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
                
                if address == ADDR_MX_PRESENT_POSITION:
                    dxl_present_position = dxl_present_position - DXL_ZERO_POSITION

                dxl_present_position = dxl_present_position / DXL_CONVERTION_FACTOR
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
            else:
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, id, address, value)

            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
    
    
    
    def read_position(self):
        """"
        Reads the current positions of the arm joints and gripper."
        """
        pos = self.bulk_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.motor_ids)  
        
        pos = [p * factor for p, factor in zip(pos, [1, -1, -1])]
       
        return pos
        
    
    def read_velocity(self):
        """"
        Reads the current velocities of the arm joints."
        """
        vels = self.bulk_read(ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY, self.motor_ids)  
        vels = [vel * factor for vel, factor in zip(vels, [1, -1, -1])]
        return vels
    
    def read_gripper_position(self):
        """
        Reads the current position of the gripper."
        """
        return self.bulk_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.gripper_ids)[0]
    

    def set_torque(self, enable: bool):
        """
        Enables or disables the torque for the arm motors.
        Args:
            enable (bool): True to enable torque, False to disable.
        """
        self.bulk_write(ADDR_MX_TORQUE_ENABLE, LEN_MX_TORQUE_ENABLE, self.motor_ids, [enable, enable, enable])
    
    def write_joints(self, positions):
        """
        Writes target positions to the arm joints.
        Args:
            positions (list[float]): A list of target positions for the arm joints.
        """
        positions = [int(position*factor*DXL_CONVERTION_FACTOR+DXL_ZERO_POSITION) for position, factor in zip(positions,[1,-1,-1])]
        self.bulk_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.motor_ids, positions)

    def write_time(self, time):
        """
        Sets the profile velocity and acceleration for the motors based on a given time profile.
        Args:
            time (float): The time in seconds for the profile velocity and acceleration.
        """

        time_ms = int(time*1000)
        acc_time_ms = int(time_ms/3)

        self.bulk_write(ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY, self.motor_ids,[time_ms, time_ms, time_ms])
        self.bulk_write(ADDR_MX_PROFILE_ACCELERATION, LEN_MX_PROFILE_ACCELERATION, self.motor_ids, [acc_time_ms, acc_time_ms, acc_time_ms])

    def set_gripper_torque(self,enable: bool):
        """
        Enables or disables the torque for the gripper.
        Args:
            enable (bool): True to enable torque, False to disable.
        """
        self.bulk_write(ADDR_MX_TORQUE_ENABLE, LEN_MX_TORQUE_ENABLE, self.gripper_ids, [enable])


    def write_gripper(self, position):
        """
        Writes a target position to the gripper. 
        Args:
            position (int): The target position for the gripper.
        """
        self.bulk_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.gripper_ids, [position])
        

    def write_gripper_time(self, time):
        """
        Sets the profile velocity and acceleration for the gripper based on a given time.
        Args:
            time (float): The time in seconds for the profile velocity and acceleration.
        """
        time_ms = int(time*1000)
        acc_time_ms = int(time_ms/3)

        self.bulk_write(ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY, self.gripper_ids,[time_ms])
        self.bulk_write(ADDR_MX_PROFILE_ACCELERATION, LEN_MX_PROFILE_ACCELERATION, self.gripper_ids, [acc_time_ms])
    
    def reboot(self):
        """
        Reboots all motors to reset their state.
        This method sends a reboot command to all motors in the motor_ids list.
        """
        for id in self.motor_ids:
            self.packet_handler.reboot(self.port_handler,id)
        self.packet_handler.reboot(self.port_handler,self.gripper_ids[0])

    def close(self):
        self.port_handler.closePort()

if __name__ == "__main__":
    
    arm = DynamixelArm()
    arm.set_gripper_torque(1)
    arm.write_gripper(arm.gripper_close)
    # arm.write_gripper(arm.gripper_open)
    # time.sleep(1)
    # arm.write_gripper(arm.gripper_coke)
    # # arm.set_torque(TORQUE_DISABLE)
    # arm.close()
    
    
    



