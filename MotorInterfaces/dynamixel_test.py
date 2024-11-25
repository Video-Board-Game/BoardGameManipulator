from dynamixel import Dynamixel
from dynamixel_variables import *
from typing import Type
import sys
import time
import signal



unsigned = (1<<32) - 90
signed = unsigned - (1<<32)

print("unsigned :   ", unsigned)
print("signed :     ", signed)

bus = '/dev/ttyACM0'
motor = Dynamixel(12, bus, baudrate=1000000)

def signal_handler(sig, frame):
    motor.set_goal_velocity(0)
    motor.off()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

print("Model ID :   ", motor.get_id())
print("Model :      ", motor.get_model())
print("Firmware :   ", motor.get_firmware_version())
print("Baud Rate :  ", motor.get_baud_rate())
print("Operating :  ", motor.get_mode())

motor.off()
print("Enable :     ", motor.get_enable())
motor.set_mode(Mode.CURRENT)
motor.on()

print("Max Vel :    ", motor.get_velocity_limit())
motor.set_velocity_limit(15)

print("Max Vel :    ", motor.get_velocity_limit())
motor.set_goal_velocity(1)

init_time = time.time()
timeout = 10 # seconds
while time.time() < init_time + timeout:
    time.sleep(1)
    print()
    print("Cur Vel:    ", motor.get_velocity())
    print("Goal Vel:   ", motor.get_goal_velocity())
    print("Position:   ", motor.get_position())
    print("Current:    ",motor.get_current())
    print("Enable:     ",motor.get_enable())
    print("Temp:       ", motor.get_temperature())
    print("Position:   ", motor.get_position())
    print()

motor.reset()

init_time = time.time()
timeout = 10 # seconds
while time.time() < init_time + timeout:
    time.sleep(1)
    print()
    print("Cur Vel:    ", motor.get_velocity())
    print("Goal Vel:   ", motor.get_goal_velocity())
    print("Position:   ", motor.get_position())
    print("Current:    ",motor.get_current())
    print("Enable:     ",motor.get_enable())
    print("Temp:       ", motor.get_temperature())
    print("Position:   ", motor.get_position())
    print()