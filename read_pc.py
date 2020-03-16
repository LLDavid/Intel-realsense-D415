from open3d import *
import pyrealsense2 as rs
import numpy as np
import cv2

file_dir=r"C:\Users\Owner\Li\PyCode\MyRepo\Intel-realsense-D415\pcd\\"

for i in range(50, 200, 1):
    file_path = file_dir+'ck_body_'+str(i+1)+".pcd"
    print(file_path)
    pcd = read_point_cloud(file_path)
    # pcd = voxel_down_sample(pcd, voxel_size=0.02)
    pcd, _ = radius_outlier_removal(pcd, nb_points=3, radius=0.05)
    draw_geometries([pcd])

    # io.write_point_cloud("a.pcd", pcd)
    # print(file_path)
    # exit(0)


    if cv2.waitKey(1) == ord('q'):
        break
    else:
        print(i)
        continue


