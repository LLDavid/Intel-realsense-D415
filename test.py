import pyrealsense2 as rs
import numpy as np
from open3d import *
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
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
        points = pc.calculate(depth)

        vtx = np.asanyarray(points.get_vertices())
        tex = np.asanyarray(points.get_texture_coordinates())
        a = vtx[1]
        np_vtx=np.zeros((len(vtx),3))
        for i in range(len(vtx)):
            a=vtx[i]
            np_vtx[i, 0] = a[0]
            np_vtx[i,1] = a[1]
            np_vtx[i, 2] = a[2]
            print(np_vtx[i,:])
            if (np.absolute(np_vtx[i, 0]>17)) | np.absolute((np_vtx[i, 1]>17)) | np.absolute((np_vtx[i, 2]>17)):
                #print(np_vtx[i, :])
                np_vtx[i, :]=np.array([0,0,0])

        # discard_index+=[i]
        # np.delete(vtx,  discard_index, 0)
        # np.delete(tex, discard_index, 0)
        fig = pyplot.figure()
        ax = Axes3D(fig)
        #ax.scatter(vtx[:,0], vtx[:,1], vtx[:,2])
        print(np_vtx[:, 0])
        ax.scatter(np_vtx[:, 0], np_vtx[:, 1], np_vtx[:, 2],s=1)
        pyplot.show()
        # print("Saving to i.ply...")
        file_path='.\\ply\\'+str(frame_id)+'.ply'
        # points.export_to_ply(file_path, color)
        # print("Done")
        # Read PLY
        pcd = read_point_cloud(file_path)
        # Visualize PLY
        draw_geometries([pcd])

finally:
    pipeline_1.stop()



