from open3d import *
import pyrealsense2 as rs
import numpy as np

file_path=r"C:\Users\Owner\Li\PyCode\MyRepo\Intel-realsense-D415\pcd\1.pcd"
pcd = read_point_cloud(file_path)

draw_geometries([pcd ])