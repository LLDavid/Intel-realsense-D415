import pyrealsense2 as rs
import numpy as np
# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('821312060330'), # Copy the serial number from the viewer or device manager
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# Start streaming
profile_1 = pipeline_1.start(config_1)

frame_id=0
try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline_1.wait_for_frames()

        # Fetch color and depth frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Enable viewing stream to make sure it is streaming
        if not depth_frame or not color_frame:
            continue
        frame_id+=1
        print(type(depth_frame))
        print(type(color_frame))
        # print intrin and extrin parameters
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
        color_to_depth_extrin = color_frame.profile.get_extrinsics_to(depth_frame.profile)
        print("\n Depth intrinsics: " + str(depth_intrin))
        print("\n Color intrinsics: " + str(color_intrin))
        print("\n Depth to color extrinsics: " + str(depth_to_color_extrin))

        # Scale the depth value to meters
        depth_sensor = profile_1.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("\n\t depth_scale: " + str(depth_scale))
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_pixel = [200, 200]  # Random pixel
        depth_value = depth_image[200][200] * depth_scale
        print("\n\t depth_pixel@" + str(depth_pixel) + " value: " + str(depth_value) + " meter")

        # convert depth image to 3-d points
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_value)
        print("\n\t 3D depth_point: " + str(depth_point))

        #
        color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
        print("\n\t 3D color_point: " + str(color_point))

        #
        color_pixel = rs.rs2_project_point_to_pixel(color_intrin, [1.,2.,3.])
        print("\n\t color_pixel: " + str(color_pixel))

        print(type(color_pixel))


        if frame_id == 1:
            break
finally:
    pipeline_1.stop()