from open3d import *
import pyrealsense2 as rs
import numpy as np
from datetime import datetime
from utils import *

clipping_distance_in_meters = 1  # 1 meter
profile_1, pipeline_1, depth_scale= get_profile(822512060625)

try:
    frame_id = 0
    vis = Visualizer()
    vis.create_window('PCD', width=640, height=480)
    pointcloud = PointCloud()

    while True:
        pointcloud.clear()
        dt0=datetime.now()
        # Create RGBD
        rgbd_image = get_RGBDImage(pipeline_1, depth_scale, clipping_distance_in_meters)
        if not rgbd_image:
            continue
        # Get intrinsic
        frame_id+=1
        if frame_id == 1:
            pinhole_camera_intrinsic = get_pinhole_camera_intrinsic(profile_1)
        # Create Point cloud from rgbd
        pcd_1 = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
        pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        #pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.05)
        # draw_geometries([pcd_1])
        pointcloud = pcd_1
        vis.add_geometry(pointcloud)
        vis.update_geometry()
        vis.reset_view_point(False)
        vis.poll_events()
        vis.update_renderer()
        process_time = datetime.now() - dt0
        print("FPS: "+str(1/process_time.total_seconds()))
finally:
    pipeline_1.stop()
