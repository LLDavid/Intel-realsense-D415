from open3d import *
import pyrealsense2 as rs
import numpy as np
from register_utils import *
from pc_utils import *

def GetPC(SID, clipping_distance_in_meters):
    profile, pipeline, depth_scale = get_profile(SID)
    try:
        frame_id = 0
        vis = Visualizer()
        vis.create_window('PCD', width=640, height=480)
        pointcloud = PointCloud()
        while True:
            pointcloud.clear()
            #dt0 = datetime.now()
            # Create RGBD
            rgbd_image = get_RGBDImage(pipeline, depth_scale, clipping_distance_in_meters)
            if not rgbd_image:
                continue
            frame_id += 1
            # Get intrinsic
            if frame_id == 1:
                pinhole_camera_intrinsic= get_pinhole_camera_intrinsic(profile)
            # Create Point cloud from rgbd
            pcd = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
            #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            # pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.05)
            # draw_geometries([pcd_1])
            pointcloud = pcd
            if frame_id == 1:
                break
    finally:
        pipeline.stop()
    return pcd