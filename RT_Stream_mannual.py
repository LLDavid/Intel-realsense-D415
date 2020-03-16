from open3d import *
import pyrealsense2 as rs
import numpy as np
from datetime import datetime
from pc_utils import *
from register_utils import *

SID_1=822512060625
SID_2=821312060330
SID_3=821212061385
SID_4=822512060979
RT_Matrix_1to0=[[1, 0,  0,  0],
 [0, 1, 0, 0],
 [0, 0, 1, 0],
 [ 0, 0, 0, 1]]
RT_Matrix_2to0=[[ 0.84124972, -0.19507302,  0.50422756, -0.91832208],
 [ 0.54016768,  0.26401256, -0.79907211,  2.17686131],
 [ 0.022755,    0.94458662,  0.32747263,  1.05325517],
 [ 0.,          0.,          0.,          1.,        ]]
RT_Matrix_3to0=[[ 0.78180166, -0.32345985,  0.53306649, -0.92801616],
 [ 0.62307246,  0.37262571, -0.68769963,  1.96557416],
 [ 0.02380894,  0.86978377,  0.49285833,  1.22283516],
 [ 0.,          0. ,         0. ,         1.        ]]
RT_Matrix_4to0=[[ 0.97516934,  0.03811139,  0.21815655, -0.43917217],
 [-0.10534722,  0.94631832,  0.30558732, -0.2496694 ],
 [-0.19479918, -0.32098157,  0.92683554,  0.49607633],
 [ 0. ,         0.,          0.,          1.        ]]
profile_1, pipeline_1, depth_scale_1= get_profile(SID_1)
profile_2, pipeline_2, depth_scale_2= get_profile(SID_2)
profile_3, pipeline_3, depth_scale_3= get_profile(SID_3)
profile_4, pipeline_4, depth_scale_4= get_profile(SID_4)
clipping_distance_in_meters=2

try:
    frame_id = 0
    vis = Visualizer()
    vis.create_window('PCD', width=640, height=480)
    pointcloud=PointCloud()
    while True:
        pointcloud.clear()
        dt0=datetime.now()
        # Create RGBD
        rgbd_image_1, _ = get_RGBDImage(pipeline_1, depth_scale_1, clipping_distance_in_meters)
        rgbd_image_2, _ = get_RGBDImage(pipeline_2, depth_scale_2, clipping_distance_in_meters)
        rgbd_image_3, _ = get_RGBDImage(pipeline_3, depth_scale_3, clipping_distance_in_meters)
        rgbd_image_4, _ = get_RGBDImage(pipeline_4, depth_scale_4, clipping_distance_in_meters)
        if not rgbd_image_1 or not rgbd_image_2 or not rgbd_image_3 or not rgbd_image_4:
            continue
        frame_id+=1
        # Get intrinsic
        if frame_id == 1:
            pinhole_camera_intrinsic_1 = get_pinhole_camera_intrinsic(pipeline_1)
            pinhole_camera_intrinsic_2 = get_pinhole_camera_intrinsic(pipeline_2)
            pinhole_camera_intrinsic_3 = get_pinhole_camera_intrinsic(pipeline_3)
            pinhole_camera_intrinsic_4 = get_pinhole_camera_intrinsic(pipeline_4)
        # Create Point cloud from rgbd
        pcd_1 = create_point_cloud_from_rgbd_image(rgbd_image_1, pinhole_camera_intrinsic_1)
        #pcd_1.transform(RT_Matrix_1to0)
        #pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd_2 = create_point_cloud_from_rgbd_image(rgbd_image_2, pinhole_camera_intrinsic_2)
        pcd_2.transform(RT_Matrix_2to0)
        pcd_3 = create_point_cloud_from_rgbd_image(rgbd_image_3, pinhole_camera_intrinsic_3)
        pcd_3.transform(RT_Matrix_3to0)
        pcd_4 = create_point_cloud_from_rgbd_image(rgbd_image_4, pinhole_camera_intrinsic_4)
        pcd_4.transform(RT_Matrix_4to0)
        #pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.05)
        # draw_geometries([pcd_1])
        pointcloud= pcd_1+pcd_2+pcd_3+pcd_4
        pointcloud.transform([[1,0 , 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        vis.add_geometry(pointcloud)
        vis.update_geometry()
        vis.reset_view_point(False)
        vis.poll_events()
        vis.update_renderer()
        process_time = datetime.now() - dt0
        print("FPS: "+str(1/process_time.total_seconds()))
        # if frame_id==1:
        #      break
        file_path_1 = '.\\pcd\\' + str(frame_id) + '.pcd'
        write_point_cloud(file_path_1, pointcloud)
finally:
    pipeline_1.stop()



