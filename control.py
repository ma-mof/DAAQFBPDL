from logging import debug
import drone
from simple_pid import PID
import time
import datetime
import os

USE_PID_YAW = True
USE_PID_Pitch = False

MAX_SPEED = 1       # M / s
MAX_YAW = 3        # Degrees / s 

P_YAW = 0.035 #orginal 0.01
I_YAW = 0
D_YAW = 0

P_Pitch = 0.95 #original =0.02
I_Pitch = 0
D_Pitch = 0

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
debug_general=None
debug_position=None

coef_dist_shoulders = 3.9
dist_human_b = 552 #y=a*X+b
dist_human_a = 7.2 #y=a*X+b



def configure_PID(control):
    global pidPitch,pidYaw

    """ Creates a new PID object depending on whether or not the PID or P is used """ 

    print("Configuring control")

    if control == 'PID':
        pidYaw = PID(P_YAW, I_YAW, D_YAW, setpoint=0)       # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidPitch = PID(P_Pitch, I_Pitch, D_Pitch, setpoint=0)   # I = 0.001
        pidPitch.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring PID")
    else:
        pidYaw = PID(P_YAW, 0, 0, setpoint=0)               # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidPitch = PID(P_Pitch, 0, 0, setpoint=0)             # I = 0.001
        pidPitch.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring P")

def setXdelta(XDelta):
    global inputValueYaw
    inputValueYaw = XDelta

def setZDelta(ZDelta):
    global inputValueVelocityX
    inputValueVelocityX = ZDelta

def set_flight_altitude(alt):
    global flight_altitude #default altitude = 5m
    flight_altitude = alt

## Following functions are UNUSED:
def connect_drone(drone_location):
    drone.connect_drone(drone_location) #'/dev/ttyACM0'

def getMovementYawAngle():
    return movementYawAngle

def getMovementVelocityXCommand():
    return movementPitchAngle


def set_system_state(current_state):
    global state
    state = current_state

def arm_and_takeoff(max_height):
    drone.arm_and_takeoff(max_height)

def land():
    drone.land()

def print_drone_report():
    print(drone.get_EKF_status())
    print(drone.get_battery_info())
    print(drone.get_version())
#########################

def initialize_debug_logs(DEBUG_FILEPATH):
    global debug_yaw, debug_velocity, debug_general, debug_position
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    debug_yaw = open(f"{DEBUG_FILEPATH}_yaw_{timestamp}.txt", "a")
    debug_yaw.write(f"P:	I:	D:	Error:	Command:	{DEBUG_FILEPATH}_yaw_{timestamp}.txt\n")

    debug_velocity = open(f"{DEBUG_FILEPATH}_velocity_{timestamp}.txt", "a")
    debug_velocity.write(f"P:	I:	D:	Error:	Command:	{DEBUG_FILEPATH}_velocity_{timestamp}\n")

    debug_general = open(f"{DEBUG_FILEPATH}_general_{timestamp}.txt", "a")
    debug_general.write(f"TIME:	MESSAGE:	{DEBUG_FILEPATH}_general_{timestamp}\n")

    debug_position = open(f"{DEBUG_FILEPATH}_position_{timestamp}.txt", "a")
    debug_position.write(f"TIME:	LOCATION:	ALTITUDE:	VELOCITY:	HEADING:	POSES:	HAND_CONTROL:	HEADING_ERROR:	HEADING_COMMAND:	PITCH_ERROR:	PITCH_COMMAND:	{DEBUG_FILEPATH}_position_{timestamp}\n")

def debug_writer_YAW(value):
    global debug_yaw
    debug_yaw.write(str(time.ctime())+ "	" + str(0) + "	" + str(0) + "	" + str(0) + "	" + str(inputValueYaw) + "	" + str(value) + "\n")

def debug_writer_Pitch(value):
    global debug_velocity
    debug_velocity.write(str(time.ctime())+ "	" + str(0) + "	" + str(0) + "	" + str(0) + "	" + str(inputValueYaw) + "	" + str(value) + "\n")

def debug_writer_general(message):
    global debug_general
    debug_general.write(str(time.ctime()) + "	" + message + "\n")
    debug_general.flush()

def debug_writer_position(get_location_w, get_altitude_w, get_velocity_w, get_heading_w, poses_no, hand_up, heading_rel, pitch_rel):
    global debug_position
    debug_position.write(str(time.ctime()) + "	" + str(get_location_w) + "	" +  str(get_altitude_w) + "	" + str(get_velocity_w) + "	" + str(get_heading_w) + "	" + str(poses_no) +"	" + str(hand_up) +"	" + str(heading_rel) + "	" + str(pidYaw(heading_rel)* -1) + "	" + str(pitch_rel) + "	" + str(pidPitch(pitch_rel)*-1) +   "\n")
    debug_position.flush()



def control_drone():
    global movementYawAngle, movementPitchAngle
    if (inputValueYaw == 0) and (inputValueVelocityX == 0):
        drone.send_movement_command_XYAH(0,0,flight_altitude,0)
        debug_writer_YAW(0)
        debug_writer_Pitch(0)

    elif (inputValueYaw != 0) and (inputValueVelocityX == 0):
        movementYawAngle = (pidYaw(inputValueYaw) * -1)
        drone.send_movement_command_XYAH(0,0,flight_altitude,movementYawAngle)
        debug_writer_YAW(movementYawAngle)
        debug_writer_Pitch(0)

    elif (inputValueYaw == 0) and (inputValueVelocityX != 0):
        movementPitchAngle = (pidPitch(inputValueVelocityX) * -1)
        drone.send_movement_command_XYAH(movementPitchAngle, 0,flight_altitude,0)
        debug_writer_YAW(0)
        debug_writer_Pitch(movementPitchAngle)

    elif(inputValueYaw != 0) and (inputValueVelocityX != 0):
        movementYawAngle = (pidYaw(inputValueYaw) * -1)
        movementPitchAngle = (pidPitch(inputValueVelocityX) * -1)
        drone.send_movement_command_XYAH(movementPitchAngle, 0,flight_altitude,movementYawAngle)
        debug_writer_YAW(movementYawAngle)
        debug_writer_Pitch(movementPitchAngle)

## ne koristi se:
def stop_drone():
    drone.send_movement_command_YAW(0) #nepotrebno
    drone.send_movement_command_XYA(0, 0,flight_altitude)
    
