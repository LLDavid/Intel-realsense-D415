import pyrealsense2 as rs
import numpy as np
import cv2
import logging

# Configure depth and color streams...
# Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('821312060330'), # Copy the serial number from the viewer or device manager
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# start
pipeline_1.start(config_1)

align_to = rs.stream.color
align = rs.align(align_to)
try:
    while True:

        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()

        # aligned frames
        aligned_frames = align.process(frames_1)
        aligned_color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame=aligned_frames.get_depth_frame()

        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        if not depth_frame_1 or not color_frame_1:
            continue
        # Convert images to numpy arrays
        aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
        aligned_color_image = np.asanyarray(aligned_color_frame.get_data())
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())

        a=aligned_depth_image-depth_image_1
        # for i in range(480):
        #     for j in range(640):
        #         if a[i,j] is not 0:
        #             print([i,j])
        # print(np.sum(aligned_color_image-color_image_1))
        # print(color_image_1.shape)
        # print(depth_image_1.shape)
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_image_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.05), cv2.COLORMAP_HSV)
        aligned_depth_image = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth_image , alpha=0.05), cv2.COLORMAP_HSV)

        # Stack all images horizontally
        images = np.hstack((color_image_1, depth_image_1,aligned_color_image,aligned_depth_image))


        # Show images from both cameras
        cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)




finally:

    # Stop streaming
    pipeline_1.stop()
