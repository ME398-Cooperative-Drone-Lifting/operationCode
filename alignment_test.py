from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import csv
import cv2
import numpy as np

from realsenseOps.arucoHelpers_mult import CreateDetector, GetRelativeYaw
from realsenseOps.realsenseStartup_mult import StartRealSense
from realsenseOps.vectorHelpers_mult import Center
from realsenseOps.arucoTrack import get_frames, align_frames, get_images, apply_colormap, resize_ir_image, detect_markers, process_markers
from finalScriptHelpers import arm_and_takeoff, send_global_velocity, condition_yaw

# arucoTrack() # Activate RealSense and Aruco tracking

Death = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
print(type(Death))

rows1 = [] # Rows for storing coarse positioning data

(arucoDict, arucoParams, detector) = CreateDetector()
(pipeline,align) = StartRealSense()
print('RealSense camera activated, waiting for pipeline...')
global detection
imageResized = False
stepCounter = 0

# Set the drone to take off
arm_and_takeoff(1.5, Death) # set to target altitude of 1.5 [m]
Death.mode = VehicleMode("GUIDED")
time.sleep(1)

try:
    while True:
        frames, depth_frame, ir_frame = get_frames(pipeline)
        if not depth_frame or not ir_frame:
            continue

        depth_intrin = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()

        if stepCounter == 0:
            print('Frame pipeline successfully opened...')

        aligned_depth_frame = align_frames(frames, align)
        if aligned_depth_frame is None:
            continue

        depth_image, ir_image = get_images(depth_frame, ir_frame)
        depth_colormap = apply_colormap(depth_image)

        depth_colormap_dim = depth_colormap.shape
        ir_colormap_dim = ir_image.shape

        if depth_colormap_dim != ir_colormap_dim:
            resized_ir_image = resize_ir_image(ir_image, depth_colormap_dim)
            images = np.hstack((resized_ir_image, depth_colormap))
            imageResized = True
        else:
            ir_image = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)
            images = np.hstack((ir_image, depth_colormap))
            imageResized = False

        if imageResized:
            arucoimage = resized_ir_image
        else:
            arucoimage = ir_image

        if len(arucoimage.shape) == 3:
            arucoimage = cv2.cvtColor(arucoimage, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detect_markers(arucoimage, arucoDict, arucoParams)

        if ids is not None and len(ids) > 0:
            markedImage, markerInfoList, depth_point = process_markers(corners, ids, aligned_depth_frame, depth_intrin, arucoimage)
            detection = 1
            
            # Print marker information to console for testing/status checking
            # Structure: [ids[i], marker_center, depth_point_in_meters_camera_coords, angle]
            print('ID: {}, Center: {}, Depth: {}, Angle: {}'.format(markerInfoList[0][0], markerInfoList[0][1], markerInfoList[0][2], markerInfoList[0][3]))

            markedImage = cv2.cvtColor(markedImage, cv2.COLOR_GRAY2BGR)
            disp_image = np.hstack((markedImage, depth_colormap))
        else:
            detection = 0

            if imageResized:
                disp_image = np.hstack((resized_ir_image, depth_colormap))
            else:
                disp_image = np.hstack((ir_image, depth_colormap))

        # display_image(disp_image)

        stepCounter += 1
        
        # user_input = input("Type `connected` when the drones are connected, or type `exit` to quit the script: ")
        if detection == 0:
            send_global_velocity(1, 0, 0, 1, Death) # 1 metre steps
        elif detection == 1: #Aruco marker detected
                print("RealSense Stepping")
                condition_yaw(0, droneID = Death)
                print('Sending drone vectors {} {} {}', float(depth_point[0]), float(depth_point[1]), float(depth_point[2]))
                send_global_velocity(float(depth_point[0]), float(depth_point[1]), float(depth_point[2]), 1, Death)

finally:
    pipeline.stop()
    print('\nRealSense camera deactivated, pipeline stopped...')
    print("Landing")
    Death.mode = VehicleMode("LAND")
