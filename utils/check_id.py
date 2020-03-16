from open3d import *
import pyrealsense2 as rs
import numpy as np

def pick_points(pcd):
    print("")
    print("1) Please pick at least three correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run() # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()
pc=rs.pointcloud()
# Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('822512060625'), # Copy the serial number from the viewer or device manager
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('821312060330'), # Copy the serial number from the viewer or device manager
config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# Start streaming
profile_1 = pipeline_1.start(config_1)
profile_2 = pipeline_2.start(config_2)
frame_id=0
try:
    while True:
        # Wait for the next set of frames from the camera
        frames_1 = pipeline_1.wait_for_frames()
        frames_2 = pipeline_2.wait_for_frames()
        # Fetch color and depth frames
        depth_1 = frames_1.get_depth_frame()
        color_1 = frames_1.get_color_frame()
        depth_2 = frames_2.get_depth_frame()
        color_2 = frames_2.get_color_frame()
        print(type(depth_1))
        if not depth_1 or not color_1 or not depth_2 or not color_2:
            continue
        frame_id=frame_id+1
        # if frame_id ==2:
        #     break
        # pc_1.map_to(color_1)
        # pc_2.map_to(color_2)

        # Generate the pointcloud and texture mappings
        points_1 = pc.calculate(depth_1)
        points_2 = pc.calculate(depth_2)
        vtx_1 = np.asanyarray(points_1.get_vertices())
        vtx_2 = np.asanyarray(points_2.get_vertices())
        vtx_1 = vtx_1.tolist()
        vtx_2 = vtx_2.tolist()

        tempv=[0,0,0,0,0,0,0,0,0,0]
        for ii in range(10):
            tempv[ii]=(ii**2,ii**2,ii**2)

        print(tempv)
        pcd = PointCloud()

        # Combine point cloud
        pcd.points = Vector3dVector(tempv)
        # # Down Sample
        # downpcd = voxel_down_sample(pcd, voxel_size=0.05)
        # downpcd.paint_uniform_color([1, 0.706, 0])
        # # Remove Outliers
        # cl, ind = statistical_outlier_removal(downpcd, nb_neighbors=20, std_ratio=2.0)

        # Visualize PLY
        # draw_geometries([pcd])
        print(pick_points(pcd))
        exit()
finally:
    pipeline_1.stop()



