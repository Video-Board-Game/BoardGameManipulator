# conversion_factors.py
from enum import Enum, auto
from .units import *
from importlib import resources as impresources

from enum import Enum, auto
import math
from datetime import datetime 
import time

class Mode(Enum):
    CURRENT = auto()
    VELOCITY = auto()
    POSITION = auto()
    TRAJECTORY = auto()

# Enumeration for different communication protocols, probably for interfacing with devices.
class Protocol(Enum):
    VIRTUAL = auto()
    RS_485 = auto()
    I2C = auto()
    CAN = auto()
    SPI = auto()
    SERIAL = auto()
    DYNAMIXEL = auto()

class waiter:
    def __init__(self):
        self.now = time.time()
        self.till = self.now
        
    def record_now(self):
        self.now = time.time()
        
    def wait(self, seconds):
        self.till = time.time() + seconds
        
    def if_past(self) -> bool:
        return (time.time() > self.till)

### Same For Every Motor
class Address(Enum):
    ### Address Enum Coresponds to Actual Address
    DEFAULT = auto()
    MODEL_NUMBER = 0
    MODEL_INFORMATION = 2
    FIRMWARE_VERSION = 6
    ID = 7
    BAUD_RATE = 8
    RETURN_DELAY_TIME = 9
    DRIVE_MODE = 10
    OPERATING_MODE = 11
    SECONDARY_ID = 12
    PROTOCCOL_TYPE = 13
    HOMING_OFFSET = 20
    MOVING_THRESHOLD = 24
    TEMPERATURE_LIMIT = 31
    MAX_VOlTAGE_LIMIT = 32
    MIN_VOLTAGE_LIMIT = 34
    PWM_LIMIT = 36
    CURRENT_LIMIT = 38
    VELOCITY_LIMIT = 44
    MAX_POSITON_LIMIT = 48
    MIN_POSITION_LIMIT = 52
    EXTERNAL_PORT_MODE_1 = 56
    EXTERNAL_PORT_MODE_2 = 57
    EXTERNAL_PORT_MODE_3 = 58
    STARTUP_CONFIGURATION = 60
    PWM_SLOPE = 62
    SHUTDOWN = 63
    TORQUE_ENABLE = 64
    LED = 65
    STATUS_RETURN_LEVEL = 68
    REGISTERED_INSTRUCTION = 69
    HARDWARE_ERROR_STATUS = 70
    VELOCITY_I_GAIN = 76
    VELOCITY_P_GAIN = 78
    POSITION_D_GAIN = 80
    POSITION_I_GAIN = 82
    POSITION_P_GAIN = 84    
    FEEDFORWARD_2ND_GAIN = 88
    FEEDFORWARD_1ST_GAIN = 90
    BUS_WATCHDOG = 98
    GOAL_PWM = 100
    GOAL_CURRENT = 102
    GOAL_VELOCITY = 104
    PROFILE_ACCELERATION = 108
    PROFILE_VELOCITY = 112
    GOAL_POSITION = 116
    REALTIME_TICK = 120
    MOVING = 122
    MOVING_STATUS = 123
    PRESENT_PWM = 124
    PRESENT_CURRENT = 126
    PRESENT_VELOCITY = 128
    PRESENT_POSITION = 132
    VELOCTIY_TRAJECTORY = 136
    POSITION_TRAJECTORY = 140
    PRESENT_INPUT_VOLTAGE = 144
    PRESENT_TEMPERATURE = 146
    BACKUP_READY = 147
    INDIRECT_ADDRESS_1 = 168
    INDIRECT_ADDRESS_2 = 170
    INDIRECT_ADDRESS_3 = 172
    INDIRECT_ADDRESS_4 = 174
    INDIRECT_ADDRESS_5 = 176
    INDIRECT_ADDRESS_6 = 178
    INDIRECT_ADDRESS_7 = 180
    INDIRECT_ADDRESS_8 = 182
    INDIRECT_ADDRESS_9 = 184
    INDIRECT_ADDRESS_10 = 186
    INDIRECT_ADDRESS_11 = 188
    INDIRECT_ADDRESS_12 = 190
    INDIRECT_ADDRESS_13 = 192
    INDIRECT_ADDRESS_14 = 194
    INDIRECT_ADDRESS_15 = 196
    INDIRECT_ADDRESS_16 = 198
    INDIRECT_ADDRESS_17 = 200
    INDIRECT_ADDRESS_18 = 202
    INDIRECT_ADDRESS_19 = 204
    INDIRECT_ADDRESS_20 = 206
    INDIRECT_ADDRESS_21 = 208
    INDIRECT_ADDRESS_22 = 210
    INDIRECT_ADDRESS_23 = 212
    INDIRECT_ADDRESS_24 = 214
    INDIRECT_ADDRESS_25 = 216
    INDIRECT_ADDRESS_26 = 218
    INDIRECT_ADDRESS_27 = 220
    INDIRECT_ADDRESS_28 = 222
    INDIRECT_DATA_1 = 224
    INDIRECT_DATA_2 = 225
    INDIRECT_DATA_3 = 226
    INDIRECT_DATA_4 = 227
    INDIRECT_DATA_5 = 228
    INDIRECT_DATA_6 = 229
    INDIRECT_DATA_7 = 230
    INDIRECT_DATA_8 = 231
    INDIRECT_DATA_9 = 232
    INDIRECT_DATA_10 = 233
    INDIRECT_DATA_11 = 234
    INDIRECT_DATA_12 = 235
    INDIRECT_DATA_13 = 236
    INDIRECT_DATA_14 = 237
    INDIRECT_DATA_15 = 238
    INDIRECT_DATA_16 = 239
    INDIRECT_DATA_17 = 240
    INDIRECT_DATA_18 = 241
    INDIRECT_DATA_19 = 242
    INDIRECT_DATA_20 = 243
    INDIRECT_DATA_21 = 244
    INDIRECT_DATA_22 = 245
    INDIRECT_DATA_23 = 246
    INDIRECT_DATA_24 = 247
    INDIRECT_DATA_25 = 248
    INDIRECT_DATA_26 = 249
    INDIRECT_DATA_27 = 250
    INDIRECT_DATA_28 = 251
    INDIRECT_ADDRESS_29 = 578
    INDIRECT_ADDRESS_30 = 580
    INDIRECT_ADDRESS_31 = 582
    INDIRECT_ADDRESS_32 = 584
    INDIRECT_ADDRESS_33 = 586
    INDIRECT_ADDRESS_34 = 588
    INDIRECT_ADDRESS_35 = 590
    INDIRECT_ADDRESS_36 = 592
    INDIRECT_ADDRESS_37 = 594
    INDIRECT_ADDRESS_38 = 596
    INDIRECT_ADDRESS_39 = 598
    INDIRECT_ADDRESS_40 = 600
    INDIRECT_ADDRESS_41 = 602
    INDIRECT_ADDRESS_42 = 604
    INDIRECT_ADDRESS_43 = 606
    INDIRECT_ADDRESS_44 = 608
    INDIRECT_ADDRESS_45 = 610
    INDIRECT_ADDRESS_46 = 612
    INDIRECT_ADDRESS_47 = 614
    INDIRECT_ADDRESS_48 = 616
    INDIRECT_ADDRESS_49 = 618
    INDIRECT_ADDRESS_50 = 620
    INDIRECT_ADDRESS_51 = 622
    INDIRECT_ADDRESS_52 = 624
    INDIRECT_ADDRESS_53 = 626
    INDIRECT_ADDRESS_54 = 628
    INDIRECT_ADDRESS_55 = 630
    INDIRECT_ADDRESS_56 = 632
    INDIRECT_DATA_29 = 634
    INDIRECT_DATA_30 = 635
    INDIRECT_DATA_31 = 636
    INDIRECT_DATA_32 = 637
    INDIRECT_DATA_33 = 638
    INDIRECT_DATA_34 = 639
    INDIRECT_DATA_35 = 640
    INDIRECT_DATA_36 = 641
    INDIRECT_DATA_37 = 642
    INDIRECT_DATA_38 = 643
    INDIRECT_DATA_39 = 644
    INDIRECT_DATA_40 = 645
    INDIRECT_DATA_41 = 646
    INDIRECT_DATA_42 = 647
    INDIRECT_DATA_43 = 648
    INDIRECT_DATA_44 = 649
    INDIRECT_DATA_45 = 650
    INDIRECT_DATA_46 = 651
    INDIRECT_DATA_47 = 652
    INDIRECT_DATA_48 = 653
    INDIRECT_DATA_49 = 654
    INDIRECT_DATA_50 = 655
    INDIRECT_DATA_51 = 656
    INDIRECT_DATA_52 = 657
    INDIRECT_DATA_53 = 658
    INDIRECT_DATA_54 = 659
    INDIRECT_DATA_55 = 660
    INDIRECT_DATA_56 = 661

class Data (Enum):
    SIZE = auto()
    ACCESS = auto()
    RANGE = auto()
    UNIT = auto()
    INDIRECT_ADDRESS = auto()
    
class Access (Enum):
    READ = auto()
    READ_WRITE = auto()
    WRITE = auto()
    NOT_AVAILABLE = auto()


modes = {
    Mode.CURRENT : 0,
    Mode.VELOCITY : 1,
    Mode.POSITION : 3,
    Mode.TRAJECTORY : 5,
}
inv_modes = {v: k for k, v in modes.items()}

### All Values are for XH540_V150_R
default_parameters = {
    Address.DEFAULT:{
        ### Typical Data
        Data.SIZE : 1,
        Data.ACCESS : Access.READ_WRITE,
        Data.RANGE : None,
        Data.UNIT : Unitless.UNITLESS,
        Data.INDIRECT_ADDRESS : None,
        },
    
    ### EEPROM Area
      
    Address.MODEL_NUMBER:{
        Data.SIZE : 2, 
        Data.ACCESS: Access.READ
        },
    Address.MODEL_INFORMATION:{
        Data.SIZE : 4,
        Data.ACCESS : Access.READ
        },
    Address.FIRMWARE_VERSION:{
        Data.ACCESS : Access.READ
        },
    Address.ID:{
        Data.RANGE : [0, 252]
        },
    Address.BAUD_RATE:{
        Data.RANGE : [0, 7]
        },
    Address.RETURN_DELAY_TIME:{
        Data.RANGE : [0, 254],
        Data.UNIT : Unit_Time.DYNAMIXEL_MICROSECOND_RESOLUTION
        },
    Address.DRIVE_MODE:{
        Data.RANGE : [0, 5]
        },
    Address.OPERATING_MODE:{
        Data.RANGE : [0, 16]
        },
    Address.SECONDARY_ID :{
        Data.RANGE : [0, 252]
        },
    Address.PROTOCCOL_TYPE:{
        Data.RANGE : [1, 2]
        },
    Address.HOMING_OFFSET:{
        Data.SIZE : 4,
        Data.RANGE : [-1044479, 1044479],
        Data.UNIT : Unit_Rotation.DYNAMIXEL_RESOLUTION
        },
    Address.MOVING_THRESHOLD:{
        Data.SIZE : 4,
        Data.RANGE : [0, 1023],
        Data.UNIT : Unit_Angular_Velocity.DYNAMIXEL_RPM_RESOLUTION
        },
    Address.TEMPERATURE_LIMIT: {
        Data.RANGE : [0, 100],
        Data.UNIT : Unit_Temp.CELSIUS
        },
    Address.MAX_VOlTAGE_LIMIT: { 
        Data.SIZE : 2,
        Data.RANGE : [110, 300], 
        Data.UNIT : Unit_Voltage.STD_VOLT_RESOLUTION
        },
    Address.MIN_VOLTAGE_LIMIT: { 
        Data.SIZE : 2,
        Data.RANGE : [110, 300],
        Data.UNIT : Unit_Voltage.STD_VOLT_RESOLUTION
        },
    Address.PWM_LIMIT: { 
        Data.SIZE : 2,
        Data.RANGE : [0, 885],
        Data.UNIT : Unitless.DYNAMIXEL_PXM
        },
    Address.CURRENT_LIMIT:{ 
        Data.SIZE : 2,
        Data.RANGE : [0, 1188],
        Data.UNIT : Unit_Current.STD_MILLIAMP_RESOLUTION
        },
    Address.VELOCITY_LIMIT: { 
        Data.SIZE : 4,
        Data.RANGE : [0, 1023],
        Data.UNIT : Unit_Angular_Velocity.DYNAMIXEL_RPM_RESOLUTION
        },
    Address.MAX_POSITON_LIMIT: {
        Data.SIZE : 4,
        Data.RANGE : [0, 4095],
        Data.UNIT : Unit_Rotation.DYNAMIXEL_RESOLUTION
        },
    Address.MIN_POSITION_LIMIT:{
        Data.SIZE : 4,
        Data.RANGE : [0, 4095],
        Data.UNIT : Unit_Rotation.DYNAMIXEL_RESOLUTION
        },
    Address.EXTERNAL_PORT_MODE_1:{
        Data.RANGE : [0, 3],
        },
    Address.EXTERNAL_PORT_MODE_2:{
        Data.RANGE : [0, 3],
        },
    Address.EXTERNAL_PORT_MODE_3:{
        Data.RANGE : [0, 3],
        },
    Address.STARTUP_CONFIGURATION: {
        Data.RANGE : [3, 3]
        },
    Address.PWM_SLOPE : {
        Data.ACCESS : Access.NOT_AVAILABLE,
        Data.RANGE : [1, 255],
        Data.UNIT : Unitless.DYNAMIXEL_PWM_SLOPE
    },
    Address.SHUTDOWN: {
        Data.ACCESS : Access.READ
        },
    
    ### RAM Area
    
    Address.TORQUE_ENABLE:{
        Data.RANGE : [0, 1]
    },
    Address.LED:{
        Data.RANGE : [0, 1]
    },
    Address.STATUS_RETURN_LEVEL:{
        Data.RANGE : [0, 2]
    },
    Address.REGISTERED_INSTRUCTION:{
        Data.RANGE : [0, 1],
        Data.ACCESS : Access.READ
    },
    Address.HARDWARE_ERROR_STATUS:{
        Data.ACCESS : Access.READ
    },
    Address.VELOCITY_I_GAIN:{
        Data.SIZE : 2,
        Data.RANGE : [0, 16383]
    },
    Address.VELOCITY_P_GAIN:{
        Data.SIZE : 2,
        Data.RANGE : [0, 16383]
    },
    Address.POSITION_D_GAIN:{
        Data.SIZE : 2,
        Data.RANGE : [0, 16383]
    },
    Address.POSITION_I_GAIN:{
        Data.SIZE : 2,
        Data.RANGE : [0, 16383]
    },
    Address.POSITION_P_GAIN:{
        Data.SIZE : 2,
        Data.RANGE : [0, 16383]
    },
    Address.FEEDFORWARD_2ND_GAIN:{
        Data.SIZE: 2,
        Data.RANGE : [0, 16383]
    },
    Address.FEEDFORWARD_1ST_GAIN:{
        Data.SIZE: 2,
        Data.RANGE : [0, 16383]
    },
    Address.BUS_WATCHDOG:{
        Data.RANGE: [1, 127],
        Data.UNIT: Unit_Time.DYNAMIXEL_WATCHDOG_MILLISECOND
    },
    Address.GOAL_PWM:{
        Data.SIZE: 2,
        Data.UNIT: Unitless.DYNAMIXEL_PXM
    },
    Address.GOAL_CURRENT:{
        Data.SIZE: 2,
        Data.UNIT: Unit_Current.STD_MILLIAMP_RESOLUTION
    },
    Address.GOAL_VELOCITY:{
        Data.SIZE: 4,
        Data.UNIT: Unit_Angular_Velocity.DYNAMIXEL_RPM_RESOLUTION
    },
    Address.PROFILE_ACCELERATION:{
        Data.SIZE: 4,
        Data.UNIT: Unit_Angular_Acceleration.DYNAMIXEL_ACCEL_RESOLUTION
    },
    Address.PROFILE_VELOCITY:{
        Data.SIZE: 4,
        Data.UNIT: Unit_Angular_Velocity.DYNAMIXEL_RPM_RESOLUTION
    },
    Address.GOAL_POSITION:{
        Data.SIZE: 4,
        Data.UNIT: Unit_Rotation.DYNAMIXEL_RESOLUTION
    },
    Address.REALTIME_TICK:{
        Data.SIZE: 2,
        Data.UNIT: Unit_Time.MILLISECOND
    },
    Address.MOVING:{
        Data.RANGE: [0, 1],
        Data.ACCESS: Access.READ
    },
    Address.MOVING_STATUS:{
        Data.ACCESS: Access.READ
    },
    Address.PRESENT_PWM:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 2,
        Data.UNIT: Unitless.DYNAMIXEL_PXM
    },
    Address.PRESENT_CURRENT:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 2,
        Data.UNIT: Unit_Current.STD_MILLIAMP_RESOLUTION
    },
    Address.PRESENT_VELOCITY:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 4,
        Data.UNIT: Unit_Angular_Velocity.DYNAMIXEL_RPM_RESOLUTION 
    },
    Address.PRESENT_POSITION:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 4,
        Data.UNIT: Unit_Rotation.DYNAMIXEL_RESOLUTION
    },
    Address.VELOCTIY_TRAJECTORY:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 4,
        Data.UNIT: Unit_Angular_Velocity.DYNAMIXEL_RPM_RESOLUTION
    },
    Address.POSITION_TRAJECTORY:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 4,
        Data.UNIT: Unit_Rotation.DYNAMIXEL_RESOLUTION
    },
    Address.PRESENT_INPUT_VOLTAGE:{
        Data.ACCESS: Access.READ,
        Data.SIZE: 2,
        Data.UNIT: Unit_Voltage.STD_VOLT_RESOLUTION
    },
    Address.PRESENT_TEMPERATURE:{
        Data.ACCESS: Access.READ,
        Data.UNIT: Unit_Temp.CELSIUS
    },
    Address.BACKUP_READY:{
        Data.ACCESS: Access.READ,
        Data.RANGE: [0, 1] 
    },
}

### Add Indirect Addresses to default_parameters

# Iterate through the enum members and collect instances of "INDIRECT"
indirect_data = [member for member in Address if "INDIRECT_DATA" in member.name]
indirect_address = [member for member in Address if "INDIRECT_ADDRESS" in member.name]

# print("INDIRECT_DATA")
# Print the collected instances
for indirect_data_address in indirect_data:
    # print(indirect_data_address)
    data_number = int(indirect_data_address.name.split("_")[-1])
    address = [member for member in indirect_address if str(data_number) in member.name]
    
    default_parameters[indirect_data_address] = {
        Data.SIZE : 1,
        Data.ACCESS : Access.READ_WRITE,
        Data.RANGE : [0, 255],
        Data.UNIT : Unitless.UNITLESS,
        Data.INDIRECT_ADDRESS : address
    }

### Models Conversion Tables and Special Stuff
class Model(Enum):
    DEFAULT = 0
    XL330_M077_T = 1190
    XL330_M288_T = 1200
    XM430_W210_R = 1030
    XH430_V210_R = 1050
    XH540_V270_R = 1140
    XH540_V150_R = 1150
    XL430_W250_T = 1060

motor_override = {
    ### To add motors you have to log the differences between the new motor and the XH540_V150_R
    Model.DEFAULT:{},
    Model.XL330_M077_T:{
        Address.BAUD_RATE : {
            Data.RANGE : [0, 6]
        },
        Address.PROTOCCOL_TYPE:{
            Data.RANGE : [2, 22]
        },
        Address.MAX_VOlTAGE_LIMIT : {
            Data.RANGE : [31, 70]
        },
        Address.MIN_VOLTAGE_LIMIT : {
            Data.RANGE : [31, 70]
        },
        Address.CURRENT_LIMIT : {
            Data.RANGE : [0, 1750],
            Data.UNIT : Unit_Current.MILLIAMP
        },
        Address.VELOCITY_LIMIT : {
            Data.RANGE : [0, 2047]
        },
        Address.PWM_SLOPE : {
            Data.ACCESS : Access.READ_WRITE
        }, 
        Address.GOAL_CURRENT:{ 
            Data.UNIT : Unit_Current.MILLIAMP
        },
        Address.PRESENT_CURRENT:{ 
            Data.UNIT : Unit_Current.MILLIAMP
        },
    },
    Model.XL330_M288_T:{
        Address.BAUD_RATE : {
            Data.RANGE : [0, 6]
        },
        Address.PROTOCCOL_TYPE:{
            Data.RANGE : [2, 22]
        },
        Address.MAX_VOlTAGE_LIMIT : {
            Data.RANGE : [31, 70]
        },
        Address.MIN_VOLTAGE_LIMIT : {
            Data.RANGE : [31, 70]
        },
        Address.CURRENT_LIMIT : {
            Data.RANGE : [0, 1750],
            Data.UNIT : Unit_Current.MILLIAMP
        },
        Address.VELOCITY_LIMIT : {
            Data.RANGE : [0, 2047]
        },
        Address.PWM_SLOPE : {
            Data.ACCESS : Access.READ_WRITE
        },
        Address.GOAL_CURRENT:{ 
            Data.UNIT : Unit_Current.MILLIAMP
        },
        Address.PRESENT_CURRENT:{ 
            Data.UNIT : Unit_Current.MILLIAMP
        },
    },
    Model.XM430_W210_R:{
        Address.MAX_VOlTAGE_LIMIT : {
            Data.RANGE : [95, 160]
        },
        Address.MIN_VOLTAGE_LIMIT : {
            Data.RANGE : [95, 160]
        },
        Address.CURRENT_LIMIT : {
            Data.RANGE : [0, 1193]
        }
    },
    Model.XH430_V210_R:{
        Address.CURRENT_LIMIT : {
            Data.RANGE : [0, 689],
            Data.UNIT : Unit_Current.XH430_V210_R_MILLIAMP
        },
        Address.GOAL_CURRENT:{ 
            Data.UNIT : Unit_Current.XH430_V210_R_MILLIAMP
        },
        Address.PRESENT_CURRENT:{ 
            Data.UNIT : Unit_Current.XH430_V210_R_MILLIAMP
        },
    },
    Model.XH540_V270_R:{},
    Model.XH540_V150_R:{},
    Model.XL430_W250_T: {
        # The XL430_W250_T does not have a current sensor
        # This address for the XL430 is the 'Present_Load'
        # Which is the % torque of the maximum being applied.
        # Units are 0.1% per LSB, but are being kept in terms of current
        # as to not break consistency with all other dynamixels.
        Address.PRESENT_CURRENT:{ 
            Data.RANGE: [-1000,1000],
            Data.UNIT : Unit_Current.MILLIAMP
        },
    }
}

def get_unit(address: Address):
    return get(Model.DEFAULT, address, Data.UNIT)

def get_bytes(address: Address):
    return get(Model.DEFAULT, address, Data.SIZE)

def get_range_ticks(model_number: Model, address: Address):
    return get(model_number, address, Data.RANGE)

def get_range(model_number: Model, address: Address):
    return [element * get_conversion(model_number, address) for element in get_range_ticks(model_number, address)] 

def get_conversion(model_number: Model, address: Address):
    return get(model_number, address, get_unit(address))

def get_access(model_number: Model, address: Address):
    return get(model_number, address, Data.ACCESS)

def get(model_number: Model, address: Address, data):
    ### Acts a filtered lookup table and pulls prioritized reference
    if address in motor_override[model_number] and data in motor_override[model_number][address]:
        return motor_override[model_number][address][data]
    elif address in default_parameters and data in default_parameters[address]:
        return default_parameters[address][data]
    elif data in motor_override[model_number]:
        return motor_override[model_number][data]
    elif data in default_parameters[Address.DEFAULT]:
        return default_parameters[Address.DEFAULT][data]
    
if __name__  == '__main__':
    model_number = Model.XL330_M288_T
    address = Address.MAX_VOlTAGE_LIMIT
    print("Model :          ", model_number.name)
    print("Address :        ", address.name)
    print("Conversion :     ", get_conversion(model_number, address), get_unit(address).name)
    print("Range :          ", get_range(model_number, address), get_unit(address).name)
    print("Bytes :          ", Data.SIZE.name, get_bytes(address))
    print("Access :         ", get_access(model_number, address).name)
