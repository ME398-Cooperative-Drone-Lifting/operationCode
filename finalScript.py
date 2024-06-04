from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import csv

from realsenseOps.arucoTrack import arucoTrack
from finalScriptHelpers import arm_and_takeoff, send_global_velocity, condition_yaw

Death = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
Taxes = connect('/dev/ttyUSB0', baud=57600)

rows1 =[] # Rows for storing coarse positioning data
Taxes = open('Taxes_Location.csv')
type(Taxes)

arm_and_takeoff(1, Taxes) # set to target altitude of 1 [m]
arm_and_takeoff(4, Death) # set to target altitude of 4 [m]
Taxes.mode = VehicleMode("STABILIZE")
Death.mode = VehicleMode("GUIDED")
time.sleep(1)

while True:
    user_input = input("Type `connected` when the drones are connected, or type `exit` to quit the script: ")
    if user_input.lower() == 'exit':
        # Prematurely exit (instruct both drones to land immediately)
        break
    elif user_input.lower() == 'connected':
        print("Death Connected")
        print("Taxes Connected")
        break
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
            send_global_velocity(float(arucoTrack.depth_point[0]), float(arucoTrack.depth_point[2]), float(arucoTrack.depth_point[2]), 1, Death)

print("Attachment Successful. Moving to Landing Zone")
# goto(5,5)
print("Landing")
Death.mode = VehicleMode("LAND")
Taxes.mode = VehicleMode("LAND")
