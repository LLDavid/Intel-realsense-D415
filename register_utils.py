from open3d import *
import pyrealsense2 as rs
import numpy as np
import cv2
from ManualRegister_2PC import *
from datetime import datetime
from register_utils import *
from pc_utils import *


def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = PinholeCameraIntrinsic(640, 480,
            intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    return out

def get_pinhole_camera_intrinsic(profile):
    tt = profile.get_stream(rs.stream.depth)
    intr = tt.as_video_stream_profile().get_intrinsics()
    pinhole_camera_intrinsic = PinholeCameraIntrinsic(intr.width, intr.height, intr.fx,
                                                      intr.fy, intr.ppx, intr.ppy)
    return pinhole_camera_intrinsic

def reorder_corners(start_id, corners):
    '''
    start_id: start from which point
    corners: 2D numpy array
    '''
    order_1=list(range(35))
    order_2=[4,3,2,1,0,9,8,7,6,5,14,13,12,11,10,19,18,17,16,15,24,23,22,21,20,29,28,27,26,25,
             34,33,32,31,30]
    if start_id==2 or start_id==3:
        if start_id == 3:
            order_2.reverse()
        global_corners = corners[order_2, :]
    else:
        if start_id == 4:
            order_1.reverse()
        global_corners = corners[order_1, :]
    return global_corners

def find_checkerboard(color_image, PATTERN_SIZE):
    '''
    :param color_image: np array
    :param PATTERN_SIZE: (# of points on row, # of points in a column)
    :return:
    '''
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray_image, PATTERN_SIZE, None)
    #print(corners)
    id_range = [ord('1'), ord('2'), ord('3'), ord('4')]
    if found:
        cv2.drawChessboardCorners(color_image, PATTERN_SIZE, corners, found)
        cv2.imshow('result', color_image)
        print("## Select start point ID ##")
        start_id = int(cv2.waitKey(0))
        if start_id in id_range:
            print("Your selected id: " + str(chr(start_id)))
            corners=reorder_corners(start_id, corners)
            print("## press space to continue ##")
            if cv2.waitKey(0) == ord(' '):
                return found, corners, chr(start_id)
        else:
            print("wrong id")
    else:
        print("nor corners found")
    return 0

# def checkerboard_cali_1():
#     PATTERN_SIZE=(5, 7)
#     corners_pc=np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1],3,4))
#     ## get corner points on checkerboard
#     SID_list=[822512060625, 821312060330, 821212061385, 822512060979]
#     for i in range(4):
#         SID=SID_list[i]
#         profile, pipeline, depth_scale = get_profile(SID)
#         align_to = rs.stream.color
#         align = rs.align(align_to)
#         try:
#             frame_id = 0
#             while True:
#                 # Get aligned frame
#                 frames = pipeline.wait_for_frames()
#                 #aligned_frames = align.process(frames)
#                 aligned_frames = frames
#                 aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
#                 color_frame = aligned_frames.get_color_frame()
#                 if not color_frame or not aligned_frames:
#                     continue
#                 # Transform color and depth images to np array
#                 color_image = np.asanyarray(color_frame.get_data())
#                 depth_image = np.asanyarray(aligned_depth_frame.get_data())
#                 print(color_image.shape)
#                 # Count frame
#                 frame_id+=1
#                 # Get intrinsic parameters
#                 tt = profile.get_stream(rs.stream.depth)
#                 depth_intr = tt.as_video_stream_profile().get_intrinsics()
#                 depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
#                 # Get ordered corner points
#                 points=np.zeros((3,1))
#
#                 found, corners, start_id = find_checkerboard(color_image, PATTERN_SIZE)
#                 print(corners)
#                 # Create a buffer depth image
#                 depth_image_part=np.ones(depth_image.shape)*100
#                 if found:
#                     for j in range(PATTERN_SIZE[0]*PATTERN_SIZE[1]):
#                         ## 1st method: calculate from intrinsic parameters
#                         ## 2nd method: use the deproject funciton
#                         ## 3rd method: create from rgbd image
#                         pixel=[corners[j, 0, 0], corners[j, 0, 1]]
#                         pixel_x = int(np.round(pixel[0]))
#                         pixel_y = int(np.round(pixel[1]))
#                         depth_value=depth_image[pixel_y, pixel_x]
#                         points = rs.rs2_deproject_pixel_to_point(depth_intr, pixel, depth_value*depth_scale)
#                         points = rs.rs2_transform_point_to_point(depth_to_color_extrin, points)
#                         points[2]= depth_value
#                         corners_pc[j, 0, i] = points[0]
#                         corners_pc[j, 1, i] = points[1]
#                         corners_pc[j, 2, i] = points[2]
#                 else:
#                     frame_id-=1
#                 # from points
#                 if frame_id == 1:
#                     break
#         finally:
#             pipeline .stop()
#     #pcd_0 = PointCloud()
# #     pcd_1 = PointCloud()
# #     pcd_2 = PointCloud()
# #     pcd_3 = PointCloud()
# #     pcd_4 = PointCloud()
# #     # target_np=np.zeros((35, 3))
# #     # for ii in range(7):
# #     #     for jj in range(5):
# #     #         target_np[ii * 5 + jj, 0] = ii
# #     #         target_np[ii * 5 + jj, 1] = jj
# #     # pcd_0.points = Vector3dVector(target_np)
# #     pcd_1.points = Vector3dVector(corners_pc[:, :, 0])
# #     pcd_2.points = Vector3dVector(corners_pc[:, :, 1])
# #     pcd_3.points = Vector3dVector(corners_pc[:, :, 2])
# #     pcd_4.points = Vector3dVector(corners_pc[:, :, 3])
# #     picked_id_sequence=np.repeat(np.arange(35).reshape(35, 1), 2, axis=1)
# #     #print(picked_id_sequence)
# #     RT1 = auto_registration(pcd_1, pcd_1, picked_id_sequence)
# #     RT2 = auto_registration(pcd_2, pcd_2, picked_id_sequence)
# #     RT3 = auto_registration(pcd_3, pcd_3, picked_id_sequence)
# #     RT4 = auto_registration(pcd_4, pcd_4, picked_id_sequence)
# #
# #     return RT1, RT2, RT3, RT4

def checkerboard_cali():
    PATTERN_SIZE=(5, 7)
    corners_pc=np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1],3,4))
    ## get corner points on checkerboard
    SID_list=[822512060625, 821312060330, 821212061385, 822512060979]
    pcd=[]
    pcd_demo=[]
    picked_id_sequence = []
    id_pc=np.zeros((35, 4))
    for i in range(4):
        SID=SID_list[i]
        profile, pipeline, depth_scale = get_profile(SID)
        align_to = rs.stream.color
        align = rs.align(align_to)
        try:
            frame_id = 0
            while True:
                # Get aligned frame
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                #aligned_frames = frames
                aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()
                if not color_frame or not aligned_frames:
                    continue
                # Transform color and depth images to np array
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                print(color_image.shape)
                # Count frame
                frame_id+=1
                # Get intrinsic parameters
                pinhole_camera_intrinsic = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))
                found, corners, start_id = find_checkerboard(color_image, PATTERN_SIZE)
                print(corners)
                # dummy id sequence array
                # Create a buffer depth image
                picked_id_sequence.append([])
                depth_rgbd=np.zeros((480, 640), dtype=np.float32)+100/depth_scale

                if found:
                    for j in range(PATTERN_SIZE[0]*PATTERN_SIZE[1]):
                        ## 1st method: calculate from intrinsic parameters
                        ## 2nd method: use the deproject funciton
                        ## 3rd method: create from rgbd image
                        pixel=[corners[j, 0, 0], corners[j, 0, 1]]
                        pixel_x = int(np.round(pixel[0]))
                        pixel_y = int(np.round(pixel[1]))
                        picked_id_sequence[i].append(pixel_y*640+pixel_x)
                        depth_rgbd[pixel_y,  pixel_x] = depth_image[pixel_y,  pixel_x]
                        id_pc[j, i]=pixel_y*640+pixel_x
                else:
                    frame_id-=1
                pcd.append(PointCloud())
                pcd_demo.append(PointCloud())
                # from points
                depth_raw = Image(depth_image)
                color_raw = Image(color_image)
                rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                                    depth_scale=1/depth_scale\
                                                                    , depth_trunc=100000000 \
                                                                    , convert_rgb_to_intensity=False)
                pcd[i] = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                # for demo
                depth_raw = Image(depth_image)
                rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                                    depth_scale=1 / depth_scale \
                                                                    , depth_trunc=1000000000 \
                                                                    , convert_rgb_to_intensity=False)
                pcd_demo[i] = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                if frame_id == 1:
                    break
        finally:
            pipeline .stop()

    # #print(picked_id_sequence)
    picked_id_sequence=np.array(picked_id_sequence)
    picked_id_sequence=picked_id_sequence.transpose()
    #print(np.hstack((picked_id_sequence[:,0], picked_id_sequence[:,0])))
    print("pcd:")
    print(np.asarray(pcd[0].points).shape)
    print("id: ")
    print(id_pc)
    id1 = np.zeros((35, 1))
    id2 = np.zeros((35, 1))
    id3 = np.zeros((35, 1))
    id4 = np.zeros((35, 1))
    dummy_seq=np.sort(id_pc, axis=0)
    for k in range(35):
        id1[k] = np.searchsorted(dummy_seq[:, 0], id_pc[k, 0])
        id2[k] = np.searchsorted(dummy_seq[:, 1], id_pc[k, 1])
        id3[k] = np.searchsorted(dummy_seq[:, 2], id_pc[k, 2])
        id4[k] = np.searchsorted(dummy_seq[:, 3], id_pc[k, 3])
    print(id1)

    id1 = id_pc[:, 0].reshape((35 ,1))
    id2 = id_pc[:, 1].reshape((35 ,1))
    id3 = id_pc[:, 2].reshape((35 ,1))
    id4 = id_pc[:, 2].reshape((35 ,1))
    print("id1")
    print(id1)
    #RT1 = auto_registration(pcd[0], pcd[0], np.hstack((id1, id1)))
    RT1 = [ [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
    RT2 = auto_registration(pcd[0], pcd[1], np.hstack((id1, id2)))
    RT3 = auto_registration(pcd[0], pcd[2], np.hstack((id1, id3)))
    RT4 = auto_registration(pcd[0], pcd[3], np.hstack((id1, id4)))


    #pcd_demo[0].transform(RT1)
    pcd_demo[1].transform(RT2)
    pcd_demo[2].transform(RT3)
    pcd_demo[3].transform(RT4)
    draw_geometries([pcd_demo[0], pcd_demo[1], pcd_demo[2], pcd_demo[3]])

    return RT1, RT2, RT3, RT4

if __name__ == "__main__":
    help(cv2.findChessboardCorners)
    checkerboard_cali()