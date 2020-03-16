from open3d import *
import pyrealsense2 as rs
import numpy as np
from register_utils import *
from ManualRegister_2PC import *

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

                    depth_image_dummy = np.ones((480, 640)) * 10000/depth_scale
                    depth_image_dummy[int(round(corners[j, 0, 1])), int(round(corners[j, 0, 0]))] = depth_image[int(round(corners[j, 0, 1])), int(round(corners[j, 0, 0]))]
                    depth_image_dummy = np.uint16(depth_image_dummy)
                    depth_raw_dummy = Image(depth_image_dummy)

                    # create for rbgd
                    rgbd_image_dummy = create_rgbd_image_from_color_and_depth(color_raw, depth_raw_dummy,
                                                                        depth_scale=1 / depth_scale
                                                                        , depth_trunc=10
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

## create id sequence
temps=np.arange(10).reshape((10,1))
id_seq=np.hstack((temps, temps))

RT1 = [ [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
RT2 = auto_registration(pcd_register[0], pcd_register[1], id_seq)
RT3 = auto_registration(pcd_register[0], pcd_register[2], id_seq)
RT4 = auto_registration(pcd_register[0], pcd_register[3], id_seq)

# pcd_demo[0].transform(RT1)
pt1 = copy.deepcopy(pcd_register[0])
pt2 = copy.deepcopy(pcd_register[1])
pt3 = copy.deepcopy(pcd_register[2])
pt4 = copy.deepcopy(pcd_register[3])

#
demo_manual_registration(pt2, pt1)