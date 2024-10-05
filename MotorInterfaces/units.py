from enum import Enum, auto
import math

# Enumeration for different time units with their conversions to seconds.
class Unit_Time(Enum):
    HOUR = 1/3600
    MINUTE = 1/60
    SECOND = 1
    MILLISECOND = 10**3
    MICROSECOND = 10**6
    DYNAMIXEL_MICROSECOND_RESOLUTION = 0.5 / MICROSECOND
    DYNAMIXEL_WATCHDOG_MILLISECOND = 0.05 / MILLISECOND

# Enumeration for revolution units, including a custom unit for a specific device (DYNAMIXEL servos).

class Unit_Distance(Enum):
    MILLIMETER = 25.4
    INCH = 1
    FOOT = 1/12
    METER = 25.4 / 1000
    MILE = 1/(12 * 5280)
    
class Unit_Speed(Enum):
    MILES_PER_HOUR = Unit_Distance.MILE.value / Unit_Time.HOUR.value
    FEET_PER_SECOND = Unit_Distance.FOOT.value / Unit_Time.SECOND.value
    METERS_PER_SECOND = Unit_Distance.METER.value / Unit_Time.SECOND.value
    INCHES_PER_SECOND = Unit_Distance.INCH.value / Unit_Time.SECOND.value
    MILLIMETERS_PER_SECOND = Unit_Distance.MILLIMETER.value / Unit_Time.SECOND.value
    # Add more combinations as necessary

class Unit_Acceleration(Enum):
    MILES_PER_HOUR_PER_HOUR = Unit_Distance.MILE.value / (Unit_Time.HOUR.value ** 2)
    FEET_PER_SECOND_SQUARED = Unit_Distance.FOOT.value / (Unit_Time.SECOND.value ** 2)
    METERS_PER_SECOND_SQUARED = Unit_Distance.METER.value / (Unit_Time.SECOND.value ** 2)
    INCHES_PER_SECOND_SQUARED = Unit_Distance.INCH.value / (Unit_Time.SECOND.value ** 2)
    MILLIMETERS_PER_SECOND_SQUARED = Unit_Distance.MILLIMETER.value / (Unit_Time.SECOND.value ** 2)

    
class Unit_Rotation(Enum):
    REVOLUTION = 1
    DEGREE = 360
    RADIAN = math.tau
    DYNAMIXEL_RESOLUTION = 4096 / REVOLUTION # Position in Ticks
    TINYMOVR_TICKS = 8192 / REVOLUTION

# Enumeration for speed units, combining revolutions with time units.
class Unit_Angular_Velocity(Enum):
    REV_PER_SEC = Unit_Rotation.REVOLUTION.value / Unit_Time.SECOND.value
    RAD_PER_SEC = Unit_Rotation.RADIAN.value / Unit_Time.SECOND.value
    REV_PER_MIN = Unit_Rotation.REVOLUTION.value / Unit_Time.MINUTE.value
    DYNAMIXEL_RPM_RESOLUTION = 4.367 * REV_PER_MIN
    TINYMOVR_TICKS_PER_SECOND = Unit_Rotation.TINYMOVR_TICKS.value / Unit_Time.SECOND.value

# Enumeration for acceleration units, further combining speed units with time.
class Unit_Angular_Acceleration(Enum):
    REV_PER_SEC_SQUARED = Unit_Rotation.REVOLUTION.value / Unit_Time.SECOND.value**2
    RAD_PER_SEC_SQUARED = Unit_Rotation.RADIAN.value / Unit_Time.SECOND.value**2
    REV_PER_MIN_SQUARED = Unit_Rotation.REVOLUTION.value / Unit_Time.MINUTE.value**2
    DYNAMIXEL_ACCEL_RESOLUTION = 0.004660332 * REV_PER_MIN_SQUARED
    TINYMOVR_TICKS_PER_SECOND_SQUARED = Unit_Rotation.TINYMOVR_TICKS.value / Unit_Time.SECOND.value**2

# Enumeration for voltage units, straightforward as most systems use volts directly.
class Unit_Voltage(Enum):
    VOLT = 1
    MILLIVOLT = 1000
    MICROVOLT = 10**6
    STD_VOLT_RESOLUTION = 10 / VOLT

# Enumeration for current units.
class Unit_Current(Enum):
    AMP = 1
    MILLIAMP = 10**3
    STD_MILLIAMP_RESOLUTION = 0.3717 / MILLIAMP
    XH430_V210_R_MILLIAMP = 0.746 / MILLIAMP

# Enumeration for temperature units, leveraging `auto()` for automatic value assignment.
class Unit_Temp(Enum):
    CELSIUS = auto()
    FAHRENHEIT = auto()

# Enumeration for power units, combining voltage and current units.
class Unit_Power(Enum):
    WATT = Unit_Voltage.VOLT.value * Unit_Current.AMP.value
    KILOWATT = 10**-3

class Unitless(Enum):
    # For values without units, such as ratios.
    # Also used for specific conversions with units not handled in this library.
    UNITLESS = 1
    DYNAMIXEL_PXM = 8.849 / UNITLESS
    DYNAMIXEL_PWM_SLOPE = 0.5058 # mV/msec

# Default units class for setting default units across various measurement categories.
class DefaultUnits:
    TIME = Unit_Time.SECOND
    CURRENT = Unit_Current.MILLIAMP
    REVOLUTION = Unit_Rotation.RADIAN
    REVOLUTION_SPEED = Unit_Angular_Velocity(REVOLUTION.value / TIME.value)
    REVOLUTION_ACCLERATION = Unit_Angular_Acceleration(REVOLUTION_SPEED.value / TIME.value)
    DISTANCE = Unit_Distance.MILLIMETER
    SPEED = Unit_Speed(DISTANCE.value / TIME.value)
    ACCELERATION = Unit_Acceleration(SPEED.value / TIME.value)
    VOLTAGE = Unit_Voltage.VOLT
    TEMPERATURE = Unit_Temp.CELSIUS
    POWER = Unit_Power.WATT

class UnitSystem(object):
    def __init__(self,
            time = DefaultUnits.TIME, 
            current = DefaultUnits.CURRENT,
            distance = DefaultUnits.DISTANCE,
            speed = DefaultUnits.SPEED,
            acceleration = DefaultUnits.ACCELERATION,
            rotation = DefaultUnits.REVOLUTION, 
            angular_velocity = DefaultUnits.REVOLUTION_SPEED, 
            angular_acceleration = DefaultUnits.REVOLUTION_ACCLERATION,
            voltage = DefaultUnits.VOLTAGE,
            temperature = DefaultUnits.TEMPERATURE,
            power = DefaultUnits.POWER):
        self.time = time
        self.current = current
        self.rotation = rotation
        self.angular_velocity = angular_velocity
        self.angular_acceleration = angular_acceleration
        self.voltage = voltage
        self.temperature = temperature
        self.distance = distance
        self.speed = speed
        self.acceleration = acceleration
        self.power = power
        self.unitless = Unitless.UNITLESS
        
        # Function to convert between units within the same category, with special handling for temperature conversions.
        
    def get_unit_for(self, unit : Enum) -> Enum:
        """
        Filter objects in the input_list that are instances of the target_class.

        Args:
        - input_list: A list of objects.
        - target_class: The class to filter by.

        Returns:
        - A list of objects that are instances of the target_class.
        """
        # Determine the external unit used for a given unit type, based on instance configuration
        override_units = [
            self.time, 
            self.current, 
            self.rotation, 
            self.angular_velocity, 
            self.angular_acceleration,
            self.distance,
            self.speed,
            self.acceleration, 
            self.voltage, 
            self.temperature,
            self.power,
            self.unitless]
        units_same_kind = [obj for obj in override_units if isinstance(obj, unit.__class__)]
        if len(units_same_kind) == 0:
            return unit
        else:
            return units_same_kind[0]
        
    def convert(self, input, unit_input) -> float:
        # Convert a value from one unit to another, based on configured defaults
        unit_output = self.get_unit_for(unit_input)
        return convert_units(input, unit_input, unit_output)

# Custom exception for handling mismatches in enum types during conversions.
class EnumTypeMismatchError(Exception):
    def __init__(self, enum1, enum2):
        self.enum1 = enum1
        self.enum2 = enum2
        super().__init__(f"Enums must be of the same class: {enum1.__class__.__name__} != {enum2.__class__.__name__}")

# Function to convert between units within the same category, with special handling for temperature conversions.
def convert_units(input, unit_input, unit_output) -> float:
    if unit_input == unit_output:
        return input
    if unit_input.__class__ is not unit_output.__class__:
        raise EnumTypeMismatchError(unit_input, unit_output)
    
    if unit_input.__class__ == Unit_Temp:
        if unit_input is Unit_Temp.CELSIUS and unit_output is Unit_Temp.FAHRENHEIT:
            return input * 1.8 + 32
        elif unit_input is Unit_Temp.FAHRENHEIT and unit_output is Unit_Temp.CELSIUS:
            return (input - 32.0) / 1.8

    return input * unit_output.value / unit_input.value


# Main block for testing conversions between different units.
if __name__  == '__main__':
    degrees = 270
    print("revolution : ", convert_units(degrees, Unit_Rotation.DEGREE, Unit_Rotation.REVOLUTION))
    print("radians :    ", convert_units(degrees, Unit_Rotation.DEGREE, Unit_Rotation.RADIAN))
    temperature = 100
    print("F :          ", convert_units(temperature, Unit_Temp.CELSIUS, Unit_Temp.FAHRENHEIT))
    
    # Demonstrates conversion from revolutions per minute to radians per second.
    print("rad_per_sec : ", convert_units(100, Unit_Angular_Velocity.REV_PER_MIN, Unit_Angular_Velocity.RAD_PER_SEC))
    print("mile :        ", convert_units(2, Unit_Distance.MILE, Unit_Distance.MILLIMETER))
    
    u = UnitSystem()
    print(u.convert(360, Unit_Rotation.DEGREE))
    print(u.convert(212, Unit_Temp.FAHRENHEIT))