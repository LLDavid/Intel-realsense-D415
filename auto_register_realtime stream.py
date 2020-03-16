from open3d import *
import pyrealsense2 as rs
import numpy as np
from register_utils import *
from datetime import datetime
from pc_utils import *

PATTERN_SIZE=(5, 7)
corners_pc=np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1],3,4))
## get corner points on checkerboard
SID_list=[822512060625, 821312060330, 821212061385, 822512060979]
pcd_demo=[]
pcd_register=[]
picked_id_sequence = []
id_pc=np.zeros((35, 4))
id_pc_1 = np.zeros((35, 4))
id_pc_2 = np.zeros((35, 4))

for i in range(4):

    # for the i th camera
    SID=SID_list[i]

    # get session
    profile, pipeline, depth_scale = get_profile(SID)
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        frame_id = 0
        while True:

            # Get aligned frame
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            depth_intrin=aligned_depth_frame.profile.as_video_stream_profile().intrinsics # get depth intrin

            color_frame = aligned_frames.get_color_frame()
            if not color_frame or not aligned_depth_frame:
                continue
            # Transform color and depth images to np array
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_raw=Image(color_image)

            # Count frame
            frame_id+=1

            # Get intrinsic parameters
            pinhole_camera_intrinsic = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))
            found, corners, start_id = find_checkerboard(color_image, PATTERN_SIZE)



            if found:
                pcd_dummy = PointCloud()
                pcd_demo.append(PointCloud())
                pcd_register.append(PointCloud())
                tempv=[] # dummy list

                # dummy depth image
                # this is for test


                for j in range(PATTERN_SIZE[0]*PATTERN_SIZE[1]):
                    ## 1st method: calculate from intrinsic parameters
                    ## 2nd method: use the deproject funciton
                    ## 3rd method: create from rgbd image

                    depth_image_dummy = np.ones((480, 640)) * float('Inf')
                    depth_image_dummy[int(round(corners[j, 0, 1])), int(round(corners[j, 0, 0]))] = depth_image[int(round(corners[j, 0, 1])), int(round(corners[j, 0, 0]))]
                    depth_image_dummy = np.uint16(depth_image_dummy)
                    depth_raw_dummy = Image(depth_image_dummy)

                    # create for rbgd
                    rgbd_image_dummy = create_rgbd_image_from_color_and_depth(color_raw, depth_raw_dummy,
                                                                        depth_scale=1 / depth_scale
                                                                        , depth_trunc=float('Inf')
                                                                        , convert_rgb_to_intensity=False)
                    pcd_dummy = create_point_cloud_from_rgbd_image(rgbd_image_dummy, pinhole_camera_intrinsic)

                    # pick_points(pcd_dummy)
                    depth_point=np.array(pcd_dummy.points)
                    # print("depth point: ",  depth_point)
                    # exit()

                    # create list of tuple
                    tempv.append((depth_point[0, 0], depth_point[0, 1], depth_point[0,2]))

                # convert selected point to point cloud
                pcd_register[i].points=Vector3dVector(tempv)
                # draw_geometries([pcd_register[i]])
                # print(pick_points(pcd_register[i]))
                # draw_geometries(pcd)

                ## create point cloud for demo
                # create for rbgd
                depth_raw = Image(depth_image)
                # color_raw = Image(color_image)
                rgbd_image_demo = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                                    depth_scale=1 / depth_scale
                                                                    , depth_trunc=float('Inf')
                                                                    , convert_rgb_to_intensity=False)
                pcd_demo[i] = create_point_cloud_from_rgbd_image(rgbd_image_demo, pinhole_camera_intrinsic)

            else:
                frame_id-=1

            if frame_id == 1:
                break
    finally:
        pipeline.stop()

cv2.destroyAllWindows()

## create id sequence
temps=np.arange(35).reshape((35,1))
id_seq=np.hstack((temps, temps))

RT1 = [ [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
RT2 = auto_registration(pcd_register[1], pcd_register[0], id_seq)
RT3 = auto_registration(pcd_register[2], pcd_register[0], id_seq)
RT4 = auto_registration(pcd_register[3], pcd_register[0], id_seq)

SID_1=822512060625
SID_2=821312060330
SID_3=821212061385
SID_4=822512060979
RT_Matrix_2to0=RT2
RT_Matrix_3to0=RT3
RT_Matrix_4to0=RT4
profile_1, pipeline_1, depth_scale_1= get_profile(SID_1)
profile_2, pipeline_2, depth_scale_2= get_profile(SID_2)
profile_3, pipeline_3, depth_scale_3= get_profile(SID_3)
profile_4, pipeline_4, depth_scale_4= get_profile(SID_4)
clipping_distance_in_meters=5
np.save("./RTMatrix/RT20.npy", np.array(RT_Matrix_2to0))
np.save("./RTMatrix/RT30.npy", np.array(RT_Matrix_3to0))
np.save("./RTMatrix/RT40.npy", np.array(RT_Matrix_4to0))

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
        # pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.01)
        #pcd_1.transform(RT_Matrix_1to0)
        #pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd_2 = create_point_cloud_from_rgbd_image(rgbd_image_2, pinhole_camera_intrinsic)
        # pcd_2 = voxel_down_sample(pcd_2, voxel_size=0.01)
        pcd_2.transform(RT_Matrix_2to0)
        pcd_3 = create_point_cloud_from_rgbd_image(rgbd_image_3, pinhole_camera_intrinsic)
        # pcd_3 = voxel_down_sample(pcd_3, voxel_size=0.01)
        pcd_3.transform(RT_Matrix_3to0)
        pcd_4 = create_point_cloud_from_rgbd_image(rgbd_image_4, pinhole_camera_intrinsic)
        # pcd_4 = voxel_down_sample(pcd_4, voxel_size=0.01)
        pcd_4.transform(RT_Matrix_4to0)
        #pcd_1 = voxel_down_sample(pcd_1, voxel_size=0.05)
        # draw_geometries([pcd_1])
        pointcloud= pcd_1+pcd_2+pcd_3+pcd_4
        pointcloud, ind = statistical_outlier_removal(pointcloud, nb_neighbors=5, std_ratio=2)
        # pointcloud, ind = radius_outlier_removal(pointcloud, nb_points=5, radius=2)
        pointcloud.transform([[1, 0 , 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
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