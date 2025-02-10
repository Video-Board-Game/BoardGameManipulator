import dynamixel_sdk as dxl
from dynamixel_sdk import COMM_SUCCESS, COMM_TX_FAIL
import numpy as np
import time
# Control table address
ADDR_MX_OPERATING_MODE = 11         # Address for operating mode
ADDR_MX_TORQUE_ENABLE = 64         # Address for enabling torque
ADDR_MX_GOAL_POSITION = 116         # Address for setting goal position
ADDR_MX_PRESENT_POSITION = 132      # Address for reading current position
ADDR_MX_PRESENT_VELOCITY = 128      # Address for reading current velocity
ADDR_MX_PROFILE_VELOCITY = 112      # Address for setting profile velocity
ADDR_MX_PROFILE_ACCELERATION = 108  # Address for setting profile acceleration

# Data Byte Length
LEN_MX_OPERATING_MODE = 1
LEN_MX_TORQUE_ENABLE = 1
LEN_MX_GOAL_POSITION = 4
LEN_MX_PRESENT_POSITION = 4
LEN_MX_PRESENT_VELOCITY = 4
LEN_MX_PROFILE_VELOCITY = 4
LEN_MX_PROFILE_ACCELERATION = 4


# Protocol version
PROTOCOL_VERSION = 2.0  # Check your Dynamixel model's protocol version

# Default setting
# Dynamixel ID (Change based on your setup)
BAUDRATE = 1000000         # Dynamixel baudrate
DEVICENAME = 'COM5'  # Port (Update according to your system)

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
    def __init__(self, port, baudrate):
        self.port_handler = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(2.0)
        self.group_bulk_read = dxl.GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = dxl.GroupBulkWrite(self.port_handler, self.packet_handler)

        if not self.port_handler.openPort():
            raise Exception("Failed to open the port")
        if not self.port_handler.setBaudRate(baudrate):
            raise Exception("Failed to set the baudrate")

        self.motor_ids = [10, 11, 12]
        self.gripper_ids = [13]

   

    

    def bulk_read(self,address,length,motor_ids):
        values = []
        for id in motor_ids:
            dxl_present_position, dxl_comm_result, dxl_error = None,None,None
           
            if length == 4:
                dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, id, address)
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
    

    

    def set_torque(self, enable: bool):
        self.bulk_write(ADDR_MX_TORQUE_ENABLE, LEN_MX_TORQUE_ENABLE, self.motor_ids, [enable, enable, enable])
    
    def write_joints(self, positions):
        positions = [int(position*DXL_CONVERTION_FACTOR+DXL_ZERO_POSITION) for position in positions]
        self.bulk_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.motor_ids, positions)
    
    def read_position(self):
        return self.bulk_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.motor_ids)
    
    def read_velocity(self):
        return self.bulk_read(ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY, self.motor_ids)
    
    def write_time(self, time):

        time_ms = int(time*1000)
        acc_time_ms = int(time_ms/3)

        self.bulk_write(ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY, self.motor_ids,[time_ms, time_ms, time_ms])
        self.bulk_write(ADDR_MX_PROFILE_ACCELERATION, LEN_MX_PROFILE_ACCELERATION, self.motor_ids, [acc_time_ms, acc_time_ms, acc_time_ms])

    def write_gripper(self, position):
        self.bulk_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.gripper_ids, [position])
    
    def get_gripper_position(self):
        return self.bulk_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.gripper_ids)[0]

    def close(self):
        self.port_handler.closePort()

if __name__ == "__main__":
    arm = DynamixelArm(DEVICENAME, BAUDRATE)
    arm.set_torque(TORQUE_ENABLE)
    time.sleep(.5)
    positions = [np.pi/6, -np.pi/6, np.pi/6]  # Example positions
    arm.write_time(2)
    arm.write_joints(positions)
    
    print("Current Positions:", arm.read_position())
    print("Current Velocities:", arm.read_velocity())

    time.sleep(2)
    arm.write_joints([0, 0, 0])  # Move to zero position
    time.sleep(.5)
    # arm.set_torque(TORQUE_DISABLE)
    arm.close()
    
    
    



