import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from arucoHelpers_mult import CreateDetector, GetRelativeYaw
from realsenseStartup_mult import StartRealSense
from vectorHelpers_mult import Center

from typing import Tuple, List, Optional, Union

# MARK:- Activate camera, align frames
def activate_camera_and_align_frames() -> Tuple[rs.pipeline, Optional[rs.depth_frame], Optional[rs.video_frame], Optional[rs.depth_frame], Optional[rs.intrinsics], cv2.aruco_Dictionary, cv2.aruco_DetectorParameters, cv2.SimpleBlobDetector]:
    (arucoDict, arucoParams, detector) = CreateDetector()
    (pipeline,align) = StartRealSense()

    # Wait for a coherent pair of frames: depth and infrared
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    ir_frame = frames.get_infrared_frame(1)  # 1 for the first infrared sensor

    if not depth_frame or not ir_frame:
        return None, None, None, None

    # Initialize aligned_frames to None
    aligned_frames = None

    # Initialize aligned_depth_frame to None
    aligned_depth_frame = None

    try:
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
    except Exception as e:
        print("Error during alignment:", str(e))

    # Only proceed if aligned_depth_frame is not None and we have a valid aligned depth frame
    if aligned_depth_frame is not None and aligned_depth_frame:
        # Now we can safely get the intrinsic parameters
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    else:
        return None, None, None, None

    return pipeline, depth_frame, ir_frame, aligned_depth_frame, depth_intrin, arucoDict, arucoParams, detector

# MARK:- Process images and IDs
def process_images_and_ids(depth_frame: rs.depth_frame, ir_frame: rs.video_frame, aligned_depth_frame: rs.depth_frame, depth_intrin: rs.intrinsics) -> Tuple[List[np.ndarray], List[int], List[np.ndarray], np.ndarray, np.ndarray, bool, np.ndarray, np.ndarray]:
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    ir_image = np.asanyarray(ir_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    ir_colormap_dim = ir_image.shape

    # If depth and infrared resolutions are different, resize infrared image to match depth image for display
    if depth_colormap_dim != ir_colormap_dim:
        resized_ir_image = cv2.resize(ir_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        # Convert the resized infrared image to 3D by adding an extra dimension
        resized_ir_image = cv2.cvtColor(resized_ir_image, cv2.COLOR_GRAY2BGR)
        images = np.hstack((resized_ir_image, depth_colormap))
        imageResized = True
    else:
        # Convert the infrared image to 3D by adding an extra dimension
        ir_image = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)
        images = np.hstack((ir_image, depth_colormap))
        imageResized = False

    if imageResized:
        arucoimage = resized_ir_image
    else:
        arucoimage = ir_image

    # Convert arucoimage to grayscale if it's not already
    if len(arucoimage.shape) == 3:
        arucoimage = cv2.cvtColor(arucoimage, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(arucoimage, arucoDict, parameters=arucoParams)

    return corners, ids, rejected, arucoimage, depth_colormap, imageResized, ir_image, resized_ir_image

# MARK:- Identify markers
def process_markers(corners: List[np.ndarray], ids: List[int], arucoimage: np.ndarray, aligned_depth_frame: rs.depth_frame, depth_intrin: rs.intrinsics, imageResized: bool, ir_image: np.ndarray, resized_ir_image: np.ndarray) -> None:
    # Check if at least one marker was detected
    if ids is not None and len(ids) > 0:
        # Iterate over each detected marker
        for i in range(len(ids)):
            # Get the corners of the current marker
            markerCorners = corners[i][0]

            # Check if markerCorners is not empty
            if markerCorners.size > 0:
                # Calculate the center of the current marker
                marker_center = Center(markerCorners)

                # Draw the corners and center of the current marker
                markedImage = cv2.aruco.drawDetectedMarkers(arucoimage, [corners[i]], np.array([ids[i]]))                    
                cv2.circle(markedImage, tuple(marker_center), 5, (0, 0, 255), cv2.FILLED)

                # Get the depth of the current marker
                depth = aligned_depth_frame.get_distance(*marker_center)

                # Get the 3D coordinates of the current marker
                depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, marker_center, depth)

                # Get the yaw of the current marker
                angle = GetRelativeYaw(markerCorners)

                print("\nMarker ID:", ids[i])
                print("Corners:", markerCorners)
                print("Center:", marker_center)
                print("Coordinate in camera frame:", depth_point_in_meters_camera_coords)
                print("Angle:", angle)

        # Display the images
        cv2.imshow('ArUco Image', markedImage)
        cv2.imshow('Depth Colormap', depth_colormap)

        # Save the images
        cv2.imwrite('arucoimage.png', markedImage)
        cv2.imwrite('depth_colormap.png', depth_colormap)
    else:
        # Convert the infrared image to 3D by adding an extra dimension
        if imageResized:
            disp_image = np.hstack((resized_ir_image, depth_colormap))
        else:
            disp_image = np.hstack((ir_image, depth_colormap))

# Mark:- Main loop
try:
    while True:
        pipeline, depth_frame, ir_frame, aligned_depth_frame, depth_intrin, arucoDict, arucoParams, detector = activate_camera_and_align_frames()
        if depth_frame is None:
            continue
        corners, ids, rejected, arucoimage, depth_colormap, imageResized, ir_image, resized_ir_image = process_images_and_ids(depth_frame, ir_frame, aligned_depth_frame, depth_intrin)
        process_markers(corners, ids, arucoimage, aligned_depth_frame, depth_intrin, imageResized, ir_image, resized_ir_image)
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop streaming
    pipeline.stop()
