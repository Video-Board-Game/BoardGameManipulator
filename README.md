# TerrawardenMansplain
This is the repo for our Manipulator code (Hence Mansplain, Manipulate, Malewife).

### Install Instructions
1. `sudo apt install python3-build`
2. `./rebuild.sh`
Rebuild needs to be performed whenever a change is made to the MotorInterfaces

## Usage
```python
from motor_interfaces.dynamixel import Dynamixel
from motor_interfaces.dynamixel_variables import *

bus = '/dev/ttyACM0'
motor = Dynamixel(1, bus, baudrate=1000000)
```