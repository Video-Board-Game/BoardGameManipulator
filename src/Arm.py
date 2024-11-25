import fk
import vk
import numpy as np
import ik
from MotorInterfaces.dynamixel import Dynamixel
from MotorInterfaces.dynamixel_variables import *
from MotorInterfaces.units import *
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time


# Default setting
              # Dynamixel ID (Change based on your setup)
BAUDRATE = 1000000         # Dynamixel baudrate
PORT = '/dev/ttyACM0'  # Port (Update according to your system)


class Arm2:
    def __init__(self) -> None:
        self.motor_zero = Dynamixel(10, PORT, baudrate=BAUDRATE)
        self.motor_one = Dynamixel(11, PORT, baudrate=BAUDRATE)
        self.motor_two = Dynamixel(12, PORT, baudrate=BAUDRATE)
        self.motors = [self.motor_zero, self.motor_one, self.motor_two]

        self.setup()

    def setup(self):
        for motor in self.motors:
            motor.off()
            motor.set_mode(Mode.POSITION)
            motor.on()

            # motor.set_profile_max_velocity(10)
            # motor.set_profile_acceleration(50)

            motor.set_goal_position(0)

    def set_enable(self, enable:bool):
        for motor in self.motors:
            motor.set_enable(enable)

    def set_goal_positions(self, goal_positions: list):
        for i in range(len(self.motors)):
            self.motors[i].set_goal_position(goal_positions[i])
    
    def get_positions(self):
        positions = []
        for motor in self.motors:
            positions.append(motor.get_position())

    def move_to(self,x,y,z):
        goal_position = ik.arm2ik(x,y,z)
        # goal_position = [int(position*DXL_CONVERTION_FACTOR+DXL_ZERO_POSITION) for position in goal_position]
        self.set_goal_positions(goal_position)

    def get_position_xyz(self):
        joint_positions = self.get_positions() # In radians
        return fk.arm2fk(joint_positions[0],joint_positions[1],joint_positions[2])[0:3,3]
    
    def on(self):
        self.set_enable(True)

    def off(self):
        self.set_enable(False)
    

if __name__ == "__main__":
    arm = Arm2()
    time.sleep(2)
    arm.set_goal_positions([-np.pi/2,np.pi/2,-np.pi/2])
    time.sleep(5)
    zeroFk=fk.arm2fk(0,0,0)
    arm.move_to(zeroFk[0,3],zeroFk[1,3],zeroFk[2,3])
    time.sleep(5)
    print(arm.get_positions())
    arm.off()
    