import pyrealsense2 as rs


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

        if not depth or not color:
            continue

        # Tell pointcloud object to map to this color frame
        pc.map_to(color)

        # Generate the pointcloud and texture mappings
        points = pc.calculate(depth)
        #print(type(points))
        print("Saving to 1.ply...")
        points.export_to_ply("1.ply", color)
        print("Done")
finally:
    pipeline_1.stop()