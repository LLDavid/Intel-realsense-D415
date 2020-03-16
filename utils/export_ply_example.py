import pyrealsense2 as rs
import numpy as np
import cv2
# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('822512060625'), # Copy the serial number from the viewer or device manager
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# Start streaming
profile_1 = pipeline_1.start(config_1)

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline_1.wait_for_frames()

        # Fetch color and depth frames
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        ## Enable viewing stream to make sure it is streaming
        # if not depth or not color:
        #     continue
        #
        # depth_np=np.asanyarray(depth.get_data())
        # color_np=np.asanyarray(color.get_data())
        # depth_np_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_np, alpha=0.5), cv2.COLORMAP_JET)
        # images = np.hstack((color_np, depth_np_color))
        # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('Align Example', images)
        # cv2.waitKey(1)

        # Tell pointcloud object to map to this color frame
        pc.map_to(color)
        # Generate the pointcloud and texture mappings
        points = pc.calculate(depth)
        print("Saving to 1.ply...")
        points.export_to_ply("1.ply", color)
        print("Done")

finally:
    pipeline_1.stop()