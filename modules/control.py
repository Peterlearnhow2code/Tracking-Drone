import sys
sys.path.append(r"D:\Python Project\ai-drone0")

from logging import debug
from modules import drone
from simple_pid import PID
import time

USE_PID_YAW = True
USE_PID_PITCH = True

MAX_SPEED = 1.7       # M / s
MAX_YAW = 15        # Degrees / s 

P_YAW = 0.041       # 2m = 15 degree
I_YAW = 0
D_YAW = 0

P_PITCH = 0.004      # 1.5m = 373 pixels
I_PITCH = 0
D_PITCH = 0

control_loop_active = True
pidYaw = None
pidPitch = None
movementYawAngle = 0
movementPitchAngle = 0
inputValueYaw = 0
inputValueVelocityX = 0
control_loop_active = True
flight_altitude = 5

debug_yaw = None
debug_velocity = None



def configure_PID(control):
    global pidPitch,pidYaw

    """ Creates a new PID object depending on whether or not the PID or P is used """ 

    print("Configuring control")

    if control == 'PID':
        pidYaw = PID(P_YAW, I_YAW, D_YAW, setpoint=0)       # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidPitch = PID(P_PITCH, I_PITCH, D_PITCH, setpoint=0)   # I = 0.001
        pidPitch.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring PID")
    else:
        pidYaw = PID(P_YAW, 0, 0, setpoint=0)               # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidPitch = PID(P_PITCH, 0, 0, setpoint=0)             # I = 0.001
        pidPitch.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring P")

def connect_drone(drone_location):
    drone.connect_drone(drone_location) #'/dev/ttyTHS1'

def getMovementYawAngle():
    return movementYawAngle

def setXdelta(XDelta):
    global inputValueYaw
    inputValueYaw = XDelta

def getMovementVelocityXCommand():
    return movementPitchAngle

def setZDelta(ZDelta):
    global inputValueVelocityX
    inputValueVelocityX = ZDelta

def set_system_state(current_state):
    global state
    state = current_state

def set_flight_altitude(alt):
    global flight_altitude
    flight_altitude = alt
# end control functions

#drone functions
def arm_and_takeoff(max_height):
    drone.arm_and_takeoff(max_height)

def land():
    drone.land()

def print_drone_report():
    # print(drone.get_EKF_status())
    # print(drone.get_battery_info())
    # print(drone.get_version())
    print("cool")
#end drone functions

def initialize_debug_logs(DEBUG_FILEPATH):
    global debug_yaw, debug_velocity, debug_altitude
    debug_yaw = open(DEBUG_FILEPATH + "_yaw.txt", "a")
    debug_yaw.write("P: I: D: Error: command:\n")

    debug_velocity = open(DEBUG_FILEPATH + "_velocity.txt", "a")
    debug_velocity.write("P: I: D: Error: command:\n")

    debug_altitude = open(DEBUG_FILEPATH + "_altitude.txt", "a")
    debug_altitude.write("P: I: D: Error: command:\n")

def debug_writer_YAW(value):
    global debug_yaw
    debug_yaw.write(str(0) + "," + str(0) + "," + str(0) + "," + str(inputValueYaw) + "," + str(value) + "\n")

def debug_writer_PITCH(value):
    global debug_velocity
    debug_velocity.write(str(0) + "," + str(0) + "," + str(0) + "," + str(inputValueYaw) + "," + str(value) + "\n")

def debug_writer_ALTITUDE(value):
    global debug_altitude
    debug_altitude.write(str(0) + "," + str(0) + "," + str(0) + "," + str(inputValueYaw) + "," + str(value) + "\n")

def control_drone(flag_movement):
    global movementYawAngle, movementPitchAngle
    if flag_movement == 0:
        if inputValueYaw == 0:
            drone.set_velocity_xyYaw(0, 0, 0, 0)
        else:
            movementYawAngle = (pidYaw(inputValueYaw) * -1)
            drone.set_velocity_xyYaw(0, 0, movementYawAngle, 10)
            debug_writer_YAW(movementYawAngle)
    else:
        if inputValueVelocityX == 0:
            drone.set_velocity_xyYaw(0, 0, 0, 0)
        else:
            print(inputValueVelocityX)
            movementPitchAngle = (pidPitch(inputValueVelocityX) * -1)
            print(movementPitchAngle)
            drone.set_velocity_xyYaw(movementPitchAngle, 0, 0, 0)
            debug_writer_PITCH(movementPitchAngle)

def stop_drone():
    drone.set_velocity_xyYaw(0, 0, 0, 0)
    