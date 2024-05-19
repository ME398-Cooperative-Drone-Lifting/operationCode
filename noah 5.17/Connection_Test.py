from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import csv

#vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

vehicle1 = connect('com7', baud=115200)
vehicle = connect('com4', baud=57600)
print(vehicle1.mode)
print(vehicle.mode)
#vehicle1.armed = True
vehicle.armed = True
while not vehicle.armed:
    print("waiting")
    time.sleep(1)
time.sleep(2)
print("die")
vehicle1.close
vehicle.close