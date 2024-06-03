from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import RPi.GPIO as GPIO
import math
from pymavlink import mavutil
import csv

from realsenseOps.arucoTrack import arucoTrack

GPIO.setmode(GPIO.BOARD)
button_pin = 16 # GPIO pin we set the switch 
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

arucoTrack()

def arm_and_takeoff_death(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not Death.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    Death.mode = VehicleMode("GUIDED")
    Death.armed = True

    while not Death.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    Death.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", Death.location.global_relative_frame.alt)      
        if Death.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def send_global_velocity_death(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = Death.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        Death.send_mavlink(msg)
        time.sleep(1)
        
def condition_yaw_death(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = Death.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    Death.send_mavlink(msg)

def arm_and_takeoff_taxes(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not Taxes.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    Taxes.mode = VehicleMode("GUIDED")
    Taxes.armed = True

    while not Taxes.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    Taxes.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", Taxes.location.global_relative_frame.alt)      
        if Taxes.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def send_global_velocity_taxes(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = Taxes.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        Taxes.send_mavlink(msg)
        time.sleep(1)
        
def condition_yaw_taxes(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = Taxes.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    Taxes.send_mavlink(msg)

Death = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
Taxes = connect('/dev/ttyUSB0', baud=57600)

rows1 =[] #Rows for storing coarse positioning data
rows2 =[] #Rows for storiing fine positioning data
lz = (0,0,0) #Landing zone for the drones and payload
Taxes = open('Taxes_Location.csv')
type(Taxes)
RealSense = open('RealSense_Data.csv')
type(RealSense)

arm_and_takeoff_taxes(1)
arm_and_takeoff_death(4)
Taxes.mode = VehicleMode("STABILIZE")
Death.mode = VehicleMode("GUIDED")
time.sleep(1)

attachment = 0 #starts unattached

while True:
    button_state = GPIO.input(button_pin)
    if button_state == False:
        # what to do when button is pressed
        break
    else:
        if arucoTrack.detection == 0: #Aruco marker not detected
            csvreader1 = csv.reader(Taxes) #Open the csv file containing Taxes' Location Data
            for row in csvreader1:
                rows1.append(row)

            location1 = (rows1[-1])
            point = LocationGlobalRelative(float(location1[0]), float(location1[1]), float(location1[2]))
            Death.simple_goto(point, groundspeed=1)
            print("Locating Taxes")
            time.sleep(1)
        elif arucoTrack.detection == 1: #Aruco marker detected
            print("RealSense Stepping")
            condition_yaw_death(0)
            condition_yaw_taxes(0)
            send_global_velocity_death(float(arucoTrack.depth_point_in_meters_camera_coords[0]),float(arucoTrack.depth_point_in_meters_camera_coords[2]),float(arucoTrack.depth_point_in_meters_camera_coords[2]),1)

print("Attachment Successful. Moving to Landing Zone")
#goto(5,5)
print("Landing")
Death.mode = VehicleMode("LAND")
Taxes.mode = VehicleMode("LAND")
