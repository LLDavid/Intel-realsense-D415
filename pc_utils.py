from open3d import *
import pyrealsense2 as rs
import numpy as np

def get_profile(serial_number):
    if not isinstance(serial_number, str):
        serial_number=str(serial_number)
    pc = rs.pointcloud()
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipeline = rs.pipeline()
    config = rs.config()
    # config_1.enable_device('822512060625'), # Copy the serial number from the viewer or device manager
    config.enable_device(serial_number),  # Copy the serial number from the viewer or device manager
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    return profile, pipeline, depth_scale

def get_RGBDImage(pipeline, depth_scale, clipping_distance_in_meters):
    align_to = rs.stream.color
    align = rs.align(align_to)
    # Wait for the next set of frames from the camera
    frames= pipeline.wait_for_frames()
    # Align depth and color frame
    aligned_frames = align.process(frames)
    depth = aligned_frames.get_depth_frame()
    color = aligned_frames.get_color_frame()
    if not depth or not color:  # or not depth_2 or not color_2:
        return False
    depth_raw = Image(np.array(depth.get_data()))
    color_raw = Image(np.array(color.get_data()))
    rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw, depth_scale=1.0 / depth_scale \
                                                        , depth_trunc=clipping_distance_in_meters \
                                                        , convert_rgb_to_intensity=False)
    return rgbd_image, color




