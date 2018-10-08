import pyrealsense2 as rs
import numpy as np
import cv2
import logging
import time

# Configure depth and color streams...
# Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('822512060625'), # Copy the serial number from the viewer or device manager
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline_1.start(config_1)

frame_id=1
try:
    while True:
        #start=time.time()
        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        if not depth_frame_1 or not color_frame_1:
            continue
        # Convert images to numpy arrays
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)
        #
        images = np.hstack((color_image_1, depth_colormap_1))
        cv2.namedWindow('Intel Realsense D415', cv2.WINDOW_NORMAL)
        cv2.imshow('Intel Realsense D415', images)
        cv2.waitKey(1),
        frame_id = frame_id + 1
finally:

    # Stop streaming
    pipeline_1.stop()