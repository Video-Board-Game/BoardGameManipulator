import dynamixel_sdk as dxl

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
DEVICENAME = '/dev/ttyACM0'  # Port (Update according to your system)

TORQUE_ENABLE = 1        # Enable torque
TORQUE_DISABLE = 0       # Disable torque

OP_MODE_VEL = 1
OP_MODE_POS = 3
OP_MODE_EXPOS = 4

DXL_CONVERTION_FACTOR = 651.898646904 # Convertion factor from radians to encoder units
DXL_MOVING_STATUS_THRESHOLD = 20  # Threshold for position error

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

    def add_bulk_read_param(self, motor_id, address, length):
        if not self.group_bulk_read.addParam(motor_id, address, length):
            raise Exception(f"Failed to add parameter for motor ID {motor_id}")

    def add_bulk_write_param(self, motor_id, address, data, length):
        param_data = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(data)), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(data)),
                      dxl.DXL_LOBYTE(dxl.DXL_HIWORD(data)), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(data))]
        if not self.group_bulk_write.addParam(motor_id, address, length, param_data):
            raise Exception(f"Failed to add parameter for motor ID {motor_id}")

    def bulk_read(self):
        if self.group_bulk_read.txRxPacket() != dxl.COMM_SUCCESS:
            raise Exception("Failed to bulk read")
        results = {}
        for motor_id in self.motor_ids:
            if self.group_bulk_read.isAvailable(motor_id, address, length):
                results[motor_id] = self.group_bulk_read.getData(motor_id, address, length)
            else:
                raise Exception(f"Failed to get data for motor ID {motor_id}")
        return results

    def bulk_write(self):
        if self.group_bulk_write.txPacket() != dxl.COMM_SUCCESS:
            raise Exception("Failed to bulk write")
        self.group_bulk_write.clearParam()


    def bulk_read_write(self, addr, len, ids, values = []):
        if(not values): 
            for motor_id in ids:
                self.add_bulk_read_param(motor_id, addr, len)
            return self.bulk_read()
            
        else:
           for motor_id, value in zip(ids, values):
                self.add_bulk_write_param(motor_id, addr, value, len)
           self.bulk_write()

    def set_torque(self, enable: bool):
        self.bulk_read_write(ADDR_MX_TORQUE_ENABLE, LEN_MX_TORQUE_ENABLE, self.motor_ids, [enable, enable, enable])
    
    def write_joints(self, positions):
        self.bulk_read_write(ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, self.motor_ids, positions)
    
    def read_position(self):
        return self.bulk_read_write(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, self.motor_ids)
    
    def read_velocity(self):
        return self.bulk_read_write(ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY, self.motor_ids)
    
    def write_time(self, time):

        time_ms = time*.001
        acc_time_ms = time_ms/3

        self.bulk_read_write(ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY, self.motor_ids,[time_ms, time_ms, time_ms])
        self.bulk_read_write(ADDR_MX_PROFILE_ACCELERATION, LEN_MX_PROFILE_ACCELERATION, self.motor_ids, [acc_time_ms, acc_time_ms, acc_time_ms])

    def close(self):
        self.port_handler.closePort()





