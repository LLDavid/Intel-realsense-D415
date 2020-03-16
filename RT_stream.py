from register_utils import *
from datetime import datetime
from pc_utils import *

import numpy as np

RT_Matrix_2to0 = np.load("./RTMatrix/RT20.npy")
RT_Matrix_3to0 = np.load("./RTMatrix/RT30.npy")
RT_Matrix_4to0 = np.load("./RTMatrix/RT40.npy")

SID_1=822512060625
SID_2=821312060330
SID_3=821212061385
SID_4=822512060979

profile_1, pipeline_1, depth_scale_1= get_profile(SID_1)
profile_2, pipeline_2, depth_scale_2= get_profile(SID_2)
profile_3, pipeline_3, depth_scale_3= get_profile(SID_3)
profile_4, pipeline_4, depth_scale_4= get_profile(SID_4)
clipping_distance_in_meters=5

pinhole_camera_intrinsic = get_pinhole_camera_intrinsic(profile_1)
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

        # Create Point cloud from rgbd
        pcd_1 = create_point_cloud_from_rgbd_image(rgbd_image_1, pinhole_camera_intrinsic)
        pcd_1_s = pcd_1
        # pcd_1, _ = radius_outlier_removal(pcd, nb_points=15, radius=0.05)
        # pcd_1, _ = statistical_outlier_removal(pcd_1, nb_neighbors=20, std_ratio=2.0)
        # pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.01)
        #pcd_1.transform(RT_Matrix_1to0)
        #pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd_2 = create_point_cloud_from_rgbd_image(rgbd_image_2, pinhole_camera_intrinsic)
        pcd_2_s = pcd_2
        #pcd_2, _ = statistical_outlier_removal(pcd_2, nb_neighbors=20, std_ratio=2.0)
        # pcd_2 = voxel_down_sample(pcd_2, voxel_size=0.01)
        pcd_2.transform(RT_Matrix_2to0)
        pcd_3 = create_point_cloud_from_rgbd_image(rgbd_image_3, pinhole_camera_intrinsic)
        pcd_3_s = pcd_3
        #pcd_3, _ = statistical_outlier_removal(pcd_3, nb_neighbors=20, std_ratio=2.0)
        # pcd_3 = voxel_down_sample(pcd_3, voxel_size=0.01)
        pcd_3.transform(RT_Matrix_3to0)
        pcd_4 = create_point_cloud_from_rgbd_image(rgbd_image_4, pinhole_camera_intrinsic)
        pcd_4_s = pcd_4
        #pcd_4, _ = statistical_outlier_removal(pcd_4, nb_neighbors=20, std_ratio=2.0)
        # pcd_4 = voxel_down_sample(pcd_4, voxel_size=0.01)
        pcd_4.transform(RT_Matrix_4to0)
        #pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.05)
        # draw_geometries([pcd_1])
        pointcloud= pcd_1+pcd_2+pcd_3+pcd_4
        pointcloud, ind = statistical_outlier_removal(pointcloud, nb_neighbors=5, std_ratio=2)
        # pointcloud, ind = radius_outlier_removal(pointcloud, nb_points=5, radius=2)
        pointcloud.transform([[1, 0 , 0, 0], [0, 0, -1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        vis.add_geometry(pointcloud)
        vis.update_geometry()
        vis.reset_view_point(False)
        vis.poll_events()
        vis.update_renderer()
        process_time = datetime.now() - dt0
        print("FPS: "+str(1/process_time.total_seconds()))
        # if frame_id==1:
        #      break

        file_path_1 = '.\\pcd\\' + 'ck_body_' + str(frame_id) + '.pcd'
        write_point_cloud(file_path_1, pointcloud)

        # write seperate pc
        write_point_cloud('.\\pcd\\' + 'ck_body_' + str(frame_id) + '_pc01.pcd', pcd_1_s)
        write_point_cloud('.\\pcd\\' + 'ck_body_' + str(frame_id) + '_pc02.pcd', pcd_2_s)
        write_point_cloud('.\\pcd\\' + 'ck_body_' + str(frame_id) + '_pc03.pcd', pcd_3_s)
        write_point_cloud('.\\pcd\\' + 'ck_body_' + str(frame_id) + '_pc04.pcd', pcd_4_s)




finally:
    pipeline_1.stop()