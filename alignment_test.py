from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import csv

from realsenseOps.arucoTrack import arucoTrack
from finalScriptHelpers import arm_and_takeoff, send_global_velocity, condition_yaw

Death = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

rows1 =[] # Rows for storing coarse positioning data

# Set the drone to take off
#arm_and_takeoff(2, Death) # set to target altitude of 2 [m]
#Death.mode = VehicleMode("GUIDED")
#time.sleep(1)

while True:
    user_input = input("Type `connected` when the drones are connected, or type `exit` to quit the script: ")
    if user_input.lower() == 'exit':
        # Prematurely exit (instruct both drones to land immediately)
        break
    elif user_input.lower() == 'connected':
        print("Death Connected")
        break
    else:
        if arucoTrack.detection == 1: #Aruco marker detected
            print("RealSense Stepping")
            #condition_yaw(0, droneID = Death)
            #send_global_velocity(float(arucoTrack.depth_point[0]), float(arucoTrack.depth_point[1]), float(arucoTrack.depth_point[2]), 1, Death)

print("Attachment Successful. Moving to Landing Zone")
# goto(5,5)
print("Landing")
#Death.mode = VehicleMode("LAND")
