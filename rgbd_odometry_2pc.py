import open3d as o3d
import pyrealsense2 as rs
import numpy as np
import cv2
from register_utils import *
from pc_utils import *
# SID_list=[822512060625, 821312060330, 821212061385, 822512060979]
SID_list=[822512060625, 821312060330]

for i in range(4):
    SID = SID_list[i]
    profile, pipeline, depth_scale = get_profile(SID)

    # align session
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        frame_id = 0
        while True:
            # Get aligned frame
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            if not color_frame or not aligned_depth_frame:
                continue

            # Transform color and depth images to np array
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())

            # Get intrinsic parameters
            pinhole_camera_intrinsic = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))

            if frame_id  == 0:
                source_rgbd_image = geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image)
            else:
                target_rgbd_image = geometry.RGBDImage.create_from_color_and_depth(target_color, target_depth)
            frame_id+=1
    finally:
        pipeline.stop()

target_pcd = geometry.PointCloud.create_from_rgbd_image(target_rgbd_image, pinhole_camera_intrinsic)
option = odometry.OdometryOption()
odo_init = np.identity(4)
print(option)