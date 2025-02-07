import fk
import vk
import numpy as np
import ik
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

# Control table address
ADDR_MX_TORQUE_ENABLE = 64         # Address for enabling torque
ADDR_MX_GOAL_POSITION = 116         # Address for setting goal position
ADDR_MX_PRESENT_POSITION = 132      # Address for reading current position
ADDR_MX_PRESENT_VELOCITY = 128      # Address for reading current velocity

# Protocol version
PROTOCOL_VERSION = 2.0  # Check your Dynamixel model's protocol version

# Default setting
              # Dynamixel ID (Change based on your setup)
BAUDRATE = 1000000         # Dynamixel baudrate
DEVICENAME = 'COM5'  # Port (Update according to your system)

TORQUE_ENABLE = 1        # Enable torque
TORQUE_DISABLE = 0       # Disable torque
DXL_MINIMUM_POSITION = 1023         # Minimum position value
DXL_MAXIMUM_POSITION =  3073     # Maximum position value
DXL_ZERO_POSITION = 2048        # Zero position value
DXL_CONVERTION_FACTOR = 651.898646904 # Convertion factor from radians to encoder units
DXL_MOVING_STATUS_THRESHOLD = 20  # Threshold for position error

class Arm2:
    def __init__(self) -> None:
        # Initialize PortHandler and PacketHandler
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.ids = [10,11,12]

        # Open port
        if not self.port_handler.openPort():
            print("Failed to open the port!")
            return

        # Set port baudrate
        if not self.port_handler.setBaudRate(BAUDRATE):
            print("Failed to set the baudrate!")
            return
        
        for id in self.ids:
            dxl_com_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, id, 11, 4)

    def setTorque(self,enable:bool):
        for id in self.ids:
            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, id, ADDR_MX_TORQUE_ENABLE, enable)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Torque enable error: {self.packet_handler.getRxPacketError(dxl_error)}")
    

    def moveJoints(self,goal_position: list):
        self.setTorque(True)
        for id,position in zip(self.ids,goal_position):
            # Move to position
            print(f"Moving to position {position}")
            
            encoder_position=int(position*DXL_CONVERTION_FACTOR+DXL_ZERO_POSITION)
            # print(f"Encoder position: {encoder_position}")
            if encoder_position < DXL_MINIMUM_POSITION or encoder_position > DXL_MAXIMUM_POSITION:
                print(f"Position {position} out of range")
                return
            
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, id, ADDR_MX_GOAL_POSITION, encoder_position)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Goal position error: {self.packet_handler.getRxPacketError(dxl_error)}")
    
    def getJoints(self):
        joint_positions = []
        for id in self.ids:
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, id, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Present position error: {self.packet_handler.getRxPacketError(dxl_error)}")
                return
            joint_positions.append(dxl_present_position)
        return joint_positions

    def moveArm(self,x,y,z):
        goal_position = ik.arm2ik(x,y,z)
        # goal_position = [int(position*DXL_CONVERTION_FACTOR+DXL_ZERO_POSITION) for position in goal_position]
        self.moveJoints(goal_position)

    def getPosition(self):
        joint_positions = self.getJoints()
        joint_positions = [(position-DXL_ZERO_POSITION)/DXL_CONVERTION_FACTOR for position in joint_positions]
        return fk.arm2fk(joint_positions[0],joint_positions[1],joint_positions[2])[0:3,3]
    


    def kill(self):
        self.setTorque(False)
        self.port_handler.closePort()
    

if __name__ == "__main__":
    arm = Arm2()
    time.sleep(2)
    arm.moveJoints([np.pi/6,np.pi/6,np.pi/6])
    time.sleep(2)
    zeroFk=fk.arm2fk(0,0,0)
    arm.moveArm(zeroFk[0,3],zeroFk[1,3],zeroFk[2,3])
    time.sleep(2)
    print(arm.getPosition())
    arm.kill()
    