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
RT_Matrix_1to0, RT_Matrix_2to0, RT_Matrix_3to0, RT_Matrix_4to0=checkerboard_cali()
profile_1, pipeline_1, depth_scale_1= get_profile(SID_1)
profile_2, pipeline_2, depth_scale_2= get_profile(SID_2)
profile_3, pipeline_3, depth_scale_3= get_profile(SID_3)
profile_4, pipeline_4, depth_scale_4= get_profile(SID_4)
clipping_distance_in_meters=5

try:
    frame_id = 0
    vis = Visualizer()
    vis.create_window('PCD', width=640, height=480)
    pointcloud=PointCloud()
    while True:
        pointcloud.clear()
        dt0=datetime.now()
        # Create RGBD
        rgbd_image_1, color_frame_1 = get_RGBDImage(pipeline_1, depth_scale_1, clipping_distance_in_meters)
        rgbd_image_2, color_frame_2 = get_RGBDImage(pipeline_2, depth_scale_2, clipping_distance_in_meters)
        rgbd_image_3, color_frame_3 = get_RGBDImage(pipeline_3, depth_scale_3, clipping_distance_in_meters)
        rgbd_image_4, color_frame_4 = get_RGBDImage(pipeline_4, depth_scale_4, clipping_distance_in_meters)
        if not rgbd_image_1 or not rgbd_image_2 or not rgbd_image_3 or not rgbd_image_4:
            continue
        frame_id+=1
        # Get intrinsic
        if frame_id == 1:
            pinhole_camera_intrinsic_1 = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame_1))
            pinhole_camera_intrinsic_2 = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame_2))
            pinhole_camera_intrinsic_3 = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame_3))
            pinhole_camera_intrinsic_4 = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame_4))
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
        #draw_geometries([ pcd_1,pcd_2,pcd_3,pcd_4])
        print(np.asarray(pointcloud.points).shape)
        pointcloud.transform([[1, 0 , 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
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



