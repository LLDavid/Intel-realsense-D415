from open3d import *
import pyrealsense2 as rs
import numpy as np
from datetime import datetime
from utils import *

SID_1=822512060625
SID_2=821312060330
SID_3=821212061385
SID_4=822512060979
clipping_distance_in_meters = 2  # 1 meter
profile_1, pipeline_1, depth_scale_1= get_profile(SID_1)
profile_2, pipeline_2, depth_scale_2= get_profile(SID_4)

try:
    frame_id = 0
    vis = Visualizer()
    vis.create_window('PCD', width=640, height=480)
    pointcloud=PointCloud()
    while True:
        pointcloud.clear()
        dt0=datetime.now()
        # Create RGBD
        rgbd_image_1 = get_RGBDImage(pipeline_1, depth_scale_1, clipping_distance_in_meters)
        rgbd_image_2 = get_RGBDImage(pipeline_2, depth_scale_2, clipping_distance_in_meters)
        if not rgbd_image_1 or not rgbd_image_2:
            continue
        frame_id+=1
        # Get intrinsic
        if frame_id == 1:
            pinhole_camera_intrinsic_1 = get_pinhole_camera_intrinsic(profile_1)
            pinhole_camera_intrinsic_2 = get_pinhole_camera_intrinsic(profile_2)
        # Create Point cloud from rgbd
        pcd_1 = create_point_cloud_from_rgbd_image(rgbd_image_1, pinhole_camera_intrinsic_1)
        pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd_2 = create_point_cloud_from_rgbd_image(rgbd_image_2, pinhole_camera_intrinsic_2)
        pcd_2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        #pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.05)
        # draw_geometries([pcd_1])
        pointcloud = pcd_1+pcd_2
        vis.add_geometry(pointcloud)
        vis.update_geometry()
        vis.reset_view_point(False)
        vis.poll_events()
        vis.update_renderer()
        process_time = datetime.now() - dt0
        print("FPS: "+str(1/process_time.total_seconds()))
        # if frame_id==1:
        #     break
finally:
    pipeline_1.stop()
    draw_geometries([pcd_1+pcd_2])
    file_path_1 = '.\\ply\\' + 'test_1' + '.pcd'
    file_path_2 = '.\\ply\\' + 'test_2' + '.pcd'
    write_point_cloud(file_path_1, pcd_1)
    write_point_cloud(file_path_2, pcd_2)

