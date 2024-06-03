from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import RPi.GPIO as GPIO
import math
from pymavlink import mavutil
import csv

from realsenseOps.arucoTrack import arucoTrack
from finalScriptHelpers import arm_and_takeoff, send_global_velocity, condition_yaw

# # not doing limit switch :(
# GPIO.setmode(GPIO.BOARD)
# button_pin = 16 # GPIO pin we set the switch 
# GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

Death = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
Taxes = connect('/dev/ttyUSB0', baud=57600)

rows1 =[] # Rows for storing coarse positioning data
rows2 =[] # Rows for storiing fine positioning data
lz = (0,0,0) # Landing zone for the drones and payload
Taxes = open('Taxes_Location.csv')
type(Taxes)
RealSense = open('RealSense_Data.csv')
type(RealSense)

arm_and_takeoff(1, Taxes)
arm_and_takeoff(4, Death)
Taxes.mode = VehicleMode("STABILIZE")
Death.mode = VehicleMode("GUIDED")
time.sleep(1)

attachment = 0 #starts unattached

while True:
    user_input = input("Press any key to continue, or 'q' to quit: ")
    if user_input.lower() == 'exit':
        break
    elif user_input.lower() == 'connected':
        print("Death Connected")
        print("Taxes Connected")
    else:
        if arucoTrack.detection == 0: # Aruco marker not detected
            csvreader1 = csv.reader(Taxes) # Open the csv file containing Taxes' Location Data
            for row in csvreader1:
                rows1.append(row)

            location1 = (rows1[-1])
            point = LocationGlobalRelative(float(location1[0]), float(location1[1]), float(location1[2]))
            Death.simple_goto(point, groundspeed=1)
            print("Locating Taxes")
            time.sleep(1)
        elif arucoTrack.detection == 1: #Aruco marker detected
            print("RealSense Stepping")
            condition_yaw(0, droneID = Death)
            condition_yaw(0, droneID = Taxes)
            send_global_velocity(float(arucoTrack.depth_point_in_meters_camera_coords[0]), float(arucoTrack.depth_point_in_meters_camera_coords[2]), float(arucoTrack.depth_point_in_meters_camera_coords[2]), 1, Death)

print("Attachment Successful. Moving to Landing Zone")
# goto(5,5)
print("Landing")
Death.mode = VehicleMode("LAND")
Taxes.mode = VehicleMode("LAND")
