import open3d
import pyrealsense2 as rs
import numpy as np


# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('821312060330'), # Copy the serial number from the viewer or device manager
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# Start streaming
profile_1 = pipeline_1.start(config_1)
frame_id=0
try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline_1.wait_for_frames()

        # Fetch color and depth frames
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            continue
        frame_id=frame_id+1
        # if frame_id ==2:
        #     break
        pc.map_to(color)

        # Generate the pointcloud and texture mappings
        np_depth = np.asanyarray(depth.get_data())
        np_color = np.asanyarray(color.get_data())
        points = pc.calculate(depth)

        vtx = np.asanyarray(points.get_vertices())
        tex = np.asanyarray(points.get_texture_coordinates())
        np_vtx=np.zeros((len(vtx),3))
        np_color_v=np.zeros((len(vtx),3))
        print(np_color.shape)
        print(max(tex))
        for i in range(len(vtx)):
            cx= min(480-1-int(tex[i][0]*(480-1)),480-1)
            #print(tex[i][0])
            cy= min(640-1-int(tex[i][1]*(640-1)), 640-1)
            #print(cx,cy)
            #print(cx)
            np_color_v[i,:] = np_color[cx, cy,:]/255.
            a=vtx[i]
            np_vtx[i, 0] = a[0]
            np_vtx[i,1] = a[1]
            np_vtx[i, 2] = a[2]
            # print(np_vtx[i,:])
            if (np.absolute(np_vtx[i, 0]>1)) or np.absolute((np_vtx[i, 1]>1)) or np.absolute((np_vtx[i, 2]>1)):
                #print(np_vtx[i, :])
                np_vtx[i, :]=np.array([0,0,0])
        # Export point cloud to ply
        pcd = open3d.PointCloud()
        pcd.points = open3d.Vector3dVector(np_vtx)
        pcd.colors = open3d.Vector3dVector(np_color_v)
        file_path = '.\\ply\\' + str(frame_id) + '.ply'
        open3d.write_point_cloud(file_path, pcd)

        # Read PLY
        pcd1 = open3d.read_point_cloud(file_path)
        # Visualize PLY
        open3d.draw_geometries([pcd1])

finally:
    pipeline_1.stop()



