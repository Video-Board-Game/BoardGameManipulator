import dynamixel_variables as dv
import os
import time
from enum import Enum, auto
import math
from datetime import datetime
from units import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

      
class ValueOutOfRangeError(Exception):
    def __init__(self, value, min_val, max_val, range_name: Enum):
        self.value = value
        self.min_val = round(min_val, 4)
        self.max_val = round(max_val, 4)
        self.range_name = range_name
        super().__init__(f"Value {value} is out of range for {range_name.name}. Range: [{self.min_val}, {self.max_val}]")


from dynamixel_sdk import * # Uses Dynamixel SDK library

class Dynamixel(): 
    def __init__(self, id, port, protocol_version = 2.0, baudrate = 57600, units = UnitSystem(), name = "DYNAMIXEL_MOTOR"):
        ### Motor
        self.name = name
        self.id = id
        self.units = units
        self.port=port
        ### Address and Model Specific Info
        self.mv = dv
        ### Protoccol Specific
        self.protocol_version = protocol_version
        self.baudrate = baudrate
        ### Packet Handler
        self.port_handler = None
        self.packet_handler = None
        self._init_packet_handler()
        ### Store Model Number
        self.model = self.mv.Model.DEFAULT
        self.model = self.get_model()
        # self._log_bounds()
        # self.off()
        
    def _tick_range_check(self, read, address_enum):
        ###
        range_readable = self.mv.get_range_ticks(self.model, address_enum)
        # print(address_enum, "   ", range_readable)
        if range_readable is None:
            return True
        min_val = range_readable[0]
        max_val = range_readable[1]
        if not min_val <= read <= max_val:
            raise ValueOutOfRangeError(read, min_val, max_val, address_enum) 
        
    def _init_packet_handler(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_handler = PortHandler(self.port)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packet_handler = PacketHandler(self.protocol_version)

        # Open port
        if self.port_handler.openPort():
            pass
            # print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.port_handler.setBaudRate(self.baudrate):
            pass
            # print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
            
    def _read_address(self, address_enum, signed = False): 
        read = self._read(self.mv.get_bytes(address_enum), address_enum.value)
        # Middle 
        if signed:
            size = (1 << (self.mv.get_bytes(address_enum) * 8))
            if read > size / 2:
                read -= size
                
        # Find the unit of the data
        unit_internal = self.mv.get(self.model, address_enum, self.mv.Data.UNIT)  

        return self.units.convert(read, unit_internal) #Converts to external units
      
    def _read(self, size, address):
        # Computer to Dynamixel
        if (size == 1):
            data, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, self.id, address)
        elif(size == 2):
            data, result, error = self.packet_handler.read2ByteTxRx(self.port_handler, self.id, address)
        elif(size == 4):
            data, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.id, address)
        else: 
            raise Exception
        ### check stuff
        return data
        
    def _write_address(self, address_enum, data):
        # print("Conversion Write :   ", self.mv.get_conversion(self.model, address_enum))
        
        #conversion stuffs
        unit_internal = self.mv.get(self.model, address_enum, self.mv.Data.UNIT) 
        unit_external = self.units.get_unit_for(unit_internal)

        data_in_internal_unit = convert_units(data, unit_external, unit_internal)
        data_in_tick = int(data_in_internal_unit)

        # up this
        self._tick_range_check(data_in_tick, address_enum)
        self._write(self.mv.get_bytes(address_enum), address_enum.value, data_in_tick)
        
    def _write(self, size, address, data):
        # Computer to Dynamixel
        if (size == 1):
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.id, address, data)
        elif(size == 2):
            result, error = self.packet_handler.write2ByteTxRx(self.port_handler, self.id, address, data)
        elif(size == 4):
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.id, address, data)
        else: 
            raise Exception
        ### check stuff

    def reset(self):
        # Sents a reboot signal to the dynamixel, which resets errors
        self.packet_handler.reboot(self.port_handler, self.id)

    def has_warnings(self):
        # No warnings in dynamixel - only errors
        return False

    def has_errors(self):
        # Check for hardware errors
        return bool(self.get_hardware_error_status())
    
    def check_connection(self):
        # Check connection with the motor by turning the LED on and off
        self.set_LED(True)
        if self.get_LED():
            self.set_LED(False)
            if not self.get_LED():
                return True
        return False
        
    def get_model(self):
        return self.mv.Model(self._read_address(self.mv.Address.MODEL_NUMBER))
           
    def get_firmware_version(self):
        return self._read_address(self.mv.Address.FIRMWARE_VERSION)
    
    def get_id(self):
        return self._read_address(self.mv.Address.ID)
        
    def set_id(self, id):
        self._write_address(self.mv.Address.ID, id)
       
    def get_baud_rate(self):
        return self._read_address(self.mv.Address.BAUD_RATE)
    
    def set_baud_rate(self, baud_rate): 
        return self._write_address(self.mv.Address.BAUD_RATE, baud_rate)

    def get_return_delay_time(self):
        return self._read_address(self.mv.Address.RETURN_DELAY_TIME)
    
    def set_return_delay_time(self, value):
        self._write_address(self.mv.Address.RETURN_DELAY_TIME, value)

    def get_mode(self):
        return self.mv.inv_modes[self._read_address(self.mv.Address.OPERATING_MODE)]
 
    def set_mode(self, mode):
        self._write_address(self.mv.Address.OPERATING_MODE, self.mv.modes[mode])

    def get_drive_mode(self):
        return self._read_address(self.mv.Address.DRIVE_MODE)
        
    def set_drive_mode(self, value):
        self._write_address(self.mv.Address.DRIVE_MODE, value)
        
    def get_secondary_id(self):
        return self._read_address(self.mv.Address.SECONDARY_ID)
    
    def set_secondary_id(self, value):
        self._write_address(self.mv.Address.SECONDARY_ID, value)
        
    def get_protocol_type(self):
        return self._read_address(self.mv.Address.PROTOCCOL_TYPE)
    
    def set_protocol_type(self, value):
        self._write_address(self.mv.Address.PROTOCCOL_TYPE, value)
        
    def get_position_offset(self):
        return self._read_address(self.mv.Address.HOMING_OFFSET)
    
    def set_position_offset(self, value):
        self._write_address(self.mv.Address.HOMING_OFFSET, value)
        
    def get_moving_threshold(self):
        return self._read_address(self.mv.Address.MOVING_THRESHOLD)
    
    def set_moving_threshold(self, value):
        self._write_address(self.mv.Address.MOVING_THRESHOLD, value)
        
    def get_temperature_limit(self):
        return self._read_address(self.mv.Address.TEMPERATURE_LIMIT)
    
    def set_temperature_limit(self, value):
        self._write_address(self.mv.Address.TEMPERATURE_LIMIT, value)
        
    def get_max_voltage_limit(self):
        return self._read_address(self.mv.Address.MAX_VOlTAGE_LIMIT)
    
    def set_max_voltage_limit(self, value):
        self._write_address(self.mv.Address.MAX_VOlTAGE_LIMIT, value)
        
    def get_min_voltage_limit(self):
        return self._read_address(self.mv.Address.MIN_VOLTAGE_LIMIT)
    
    def set_min_voltage_limit(self, value):
        self._write_address(self.mv.Address.MIN_VOLTAGE_LIMIT, value)
        
    def get_current_limit(self):
        return self._read_address(self.mv.Address.CURRENT_LIMIT)
    
    def set_current_limit(self, current):
        self._write_address(self.mv.Address.CURRENT_LIMIT, current)
        self._log_bounds()
        
    def get_velocity_table_limit(self):
        return self.mv.get_range(self.mv.Address.VELOCITY_LIMIT)
    
    def get_velocity_limit(self):
        return self._read_address(self.mv.Address.VELOCITY_LIMIT)
    
    def set_velocity_limit(self, velocity):
        self._write_address(self.mv.Address.VELOCITY_LIMIT, velocity)
        self._log_bounds()

    def get_max_position_limit(self):
        return self._read_address(self.mv.Address.MAX_POSITON_LIMIT)
    
    def set_max_position_limit(self, value):
        self._write_address(self.mv.Address.MAX_POSITON_LIMIT, value)
        
    def get_min_position_limit(self):
        return self._read_address(self.mv.Address.MIN_POSITION_LIMIT)
    
    def set_min_position_limit(self, value):
        self._write_address(self.mv.Address.MIN_POSITION_LIMIT, value)
        
    def get_external_port_mode_1(self):
        return self._read_address(self.mv.Address.EXTERNAL_PORT_MODE_1)
     
    def set_external_port_mode_1(self, value):
        self._write_address(self.mv.Address.EXTERNAL_PORT_MODE_1, value)
        
    def get_external_port_mode_2(self):
        return self._read_address(self.mv.Address.EXTERNAL_PORT_MODE_2)
     
    def set_external_port_mode_2(self, value):
        self._write_address(self.mv.Address.EXTERNAL_PORT_MODE_2, value)
        
    def get_external_port_mode_3(self):
        return self._read_address(self.mv.Address.EXTERNAL_PORT_MODE_3)
     
    def set_external_port_mode_3(self, value):
        self._write_address(self.mv.Address.EXTERNAL_PORT_MODE_3, value)
        
    def get_startup_configuration(self):
        return self._read_address(self.mv.Address.STARTUP_CONFIGURATION)
    
    def set_startup_configuration(self, value):
        self._write_address(self.mv.Address.STARTUP_CONFIGURATION, value)
        
    def get_shutdown(self):
        return self._read_address(self, self.mv.Address.SHUTDOWN)
    
    def set_shutdown(self, value):
        self._write_address(self.mv.Address.SHUTDOWN, value)
    
    ### RAM Area
    
    def get_enable(self):
        return bool(self._read_address(self.mv.Address.TORQUE_ENABLE))
    
    def set_enable(self, state):
        self._write_address(self.mv.Address.TORQUE_ENABLE, float(state))
     
    def get_LED(self):
        return bool(self._read_address(self.mv.Address.LED))
    
    def set_LED(self, on):
        self._write_address(self.mv.Address.LED, float(on))
        
    def get_status_return_level(self):
        return self._read_address(self.mv.Address.STATUS_RETURN_LEVEL)
    
    def set_status_return_level(self, value):
        self._write_address(self.mv.Address.STATUS_RETURN_LEVEL, value)
        
    def get_registered_instruction(self):
        return bool(self._read_address(self.mv.Address.REGISTERED_INSTRUCTION))
    
    def get_hardware_error_status(self):
        return self._read_address(self.mv.Address.HARDWARE_ERROR_STATUS)
    
    def get_velocity_I_gain(self):
        return self._read_address(self.mv.Address.VELOCITY_I_GAIN)
    
    def set_velocity_I_gain(self, value):
        self._write_address(self.mv.Address.VELOCITY_I_GAIN, value)
        
    def get_velocity_P_gain(self):
        return self._read_address(self.mv.Address.VELOCITY_P_GAIN)
    
    def set_velocity_P_gain(self, value):
        self._write_address(self.mv.Address.VELOCITY_P_GAIN, value)
        
    def get_position_D_gain(self):
        return self._read_address(self.mv.Address.POSITION_D_GAIN)
    
    def set_position_D_gain(self, value):
        self._write_address(self.mv.Address.POSITION_D_GAIN, value)
        
    def get_position_I_gain(self):
        return self._read_address(self.mv.Address.POSITION_I_GAIN)
    
    def set_position_I_gain(self, value):
        self._write_address(self.mv.Address.POSITION_I_GAIN, value)
        
    def get_position_P_gain(self):
        return self._read_address(self.mv.Address.POSITION_P_GAIN)
    
    def set_position_P_gain(self, value):
        self._write_address(self.mv.Address.POSITION_P_GAIN, value)
        
    def get_feedforward_2nd_gain(self):
        return self._read_address(self.mv.Address.FEEDFORWARD_2ND_GAIN)
    
    def set_feedforward_2nd_gain(self, value):
        self._write_address(self.mv.Address.FEEDFORWARD_2ND_GAIN, value)
        
    def get_feedforward_1st_gain(self):
        return self._read_address(self.mv.Address.FEEDFORWARD_1ST_GAIN)
    
    def set_feedforward_1st_gain(self, value):
        self._write_address(self.mv.Address.FEEDFORWARD_1ST_GAIN, value)
        
    def get_bus_watchdog(self):
        return self._read_address(self.mv.Address.BUS_WATCHDOG)
    
    def set_bus_watchdog(self, value):
        self._write_address(self.mv.Address.BUS_WATCHDOG, value)
     
    def get_goal_effort(self):
        return self._read_address(self.mv.Address.GOAL_PWM, signed=True)
              
    def set_goal_effort(self, value):
        self._write_address(self.mv.Address.GOAL_PWM, value)
    
    def get_goal_current(self):
        return self._read_address(self.mv.Address.GOAL_CURRENT, signed=True)

    def set_goal_current(self, value):
        if not -self.current_limit <= value <= self.current_limit:
            raise ValueOutOfRangeError(value, -self.current_limit, self.current_limit, self.mv.Address.GOAL_CURRENT)
        self._write_address(self.mv.Address.GOAL_CURRENT, value)
        
    def get_goal_velocity(self):
        return self._read_address(self.mv.Address.GOAL_VELOCITY, signed=True)

    def set_goal_velocity(self, value):
        # if not -self.velocity_limit <= value <= self.velocity_limit:
        #     raise ValueOutOfRangeError(value, -self.velocity_limit, self.velocity_limit, self.mv.Address.GOAL_VELOCITY)
        self._write_address(self.mv.Address.GOAL_VELOCITY, value)
             
    def get_profile_acceleration(self):
        return self._read_address(self.mv.Address.PROFILE_ACCELERATION)
             
    def set_profile_acceleration(self, value):
        self._write_address(self.mv.Address.PROFILE_ACCELERATION, value)
        
    def get_profile_max_velocity(self):
        return self._read_address(self.mv.Address.PROFILE_VELOCITY)
             
    def set_profile_max_velocity(self, value):
        self._write_address(self.mv.Address.PROFILE_VELOCITY, value)   
             
    def get_goal_position(self):
        return self._read_address(self.mv.Address.GOAL_POSITION, signed=True)
    
    def set_goal_position(self, value):
        self._write_address(self.mv.Address.GOAL_POSITION, value)

    def get_realtime_tick(self):
        return self._read_address(self.mv.Address.REALTIME_TICK)
        
    def get_moving(self):
        return self._read_address(self.mv.Address.MOVING)
        
    def get_moving_status(self):
        return self._read_address(self.mv.Address.MOVING_STATUS)
        
    def get_pwm(self):
        return self._read_address(self.mv.Address.PRESENT_PWM, signed=True)
        
    def get_current(self):
        return self._read_address(self.mv.Address.PRESENT_CURRENT, signed=True)
        
    def get_velocity(self):
        return self._read_address(self.mv.Address.PRESENT_VELOCITY, signed=True)  
        
    def get_position(self):
        return self._read_address(self.mv.Address.PRESENT_POSITION, signed=True)
        
    def get_velocity_trajectory(self):
        return self._read_address(self.mv.Address.VELOCTIY_TRAJECTORY)
        
    def get_position_trajectory(self):
        return self._read_address(self.mv.Address.POSITION_TRAJECTORY)
        
    def get_input_voltage(self):
        return self._read_address(self.mv.Address.PRESENT_INPUT_VOLTAGE)
        
    def get_temperature(self):
        return self._read_address(self.mv.Address.PRESENT_TEMPERATURE)
        
    def get_backup_ready(self):
        return self._read_address(self.mv.Address.BACKUP_READY)

    def get_power_draw(self):
        return (self.get_input_voltage() * self.get_current())
    
    def print(self, depth = 0):
        prefix = "\t" * depth
        print(f"{prefix}Dynamixel: {self.name} (Model: {self.get_model()}) (ID: {self.get_id()})")
  

# Very simple test
if __name__  == '__main__':
    bus = '/dev/ttyACM0'
    motor = Dynamixel(3, bus)
    # motor.set_mode(Mode.EXTENDED_POSITION)
    motor.set_mode(dv.Mode.EXTENDED_POSITION)
    motor.off()
    motor.set_goal_velocity(2)
    print(motor.model)
    print(motor.get_mode())
    while(True):
        print(motor.get_velocity())
        time.sleep(2)