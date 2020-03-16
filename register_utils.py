from open3d import *
import pyrealsense2 as rs
import numpy as np
import cv2
from ManualRegister_2PC import *
from datetime import datetime
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
    # use start_id=1 as reference
    order_1=list(range(35))
    order_2=[4,3,2,1,0,9,8,7,6,5,14,13,12,11,10,19,18,17,16,15,24,23,22,21,20,29,28,27,26,25,
             34,33,32,31,30]
    global_corners=0
    print(start_id)
    print(global_corners)
    if start_id == 4:
        order_2.reverse()
        global_corners = corners[order_2,:, :]
    if start_id == 3:
        order_1.reverse()
        global_corners = corners[order_1,:, :]
    if start_id == 2:
        global_corners = corners[order_2, :,:]
    if start_id == 1:
        global_corners = corners[order_1,:, :]
    print(global_corners)
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
    id_range = [1, 2, 3, 4]
    if found:
        cv2.drawChessboardCorners(color_image, PATTERN_SIZE, corners, found)
        cv2.imshow('result', color_image)
        # print(corners)
        print("## Select start point ID ##")
        start_id = int(chr(cv2.waitKey(0)))
        if start_id in id_range:
            print("Your selected id: " + str(start_id))
            corners=reorder_corners(start_id, corners)
            # print(corners)
            print("## press space to continue ##")
            if cv2.waitKey(0) == ord(' '):
                return found, corners, chr(start_id)
        else:
            print("wrong id")
    else:
        print("nor corners found")
        exit(0)
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
    id_pc_1 = np.zeros((35, 4))
    id_pc_2 = np.zeros((35, 4))
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
                aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()
                if not color_frame or not aligned_depth_frame:
                    continue

                # Transform color and depth images to np array
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(aligned_depth_frame.get_data())

                # Count frame
                frame_id+=1

                # Get intrinsic parameters
                pinhole_camera_intrinsic = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))
                found, corners, start_id = find_checkerboard(color_image, PATTERN_SIZE)
                print([corners[0, 0, 0], corners[0, 0, 1]])
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

                        # get each line in corners
                        pixel=[corners[j, 0, 0], corners[j, 0, 1]]
                        pixel_x = int(np.round(pixel[0]))
                        pixel_y = int(np.round(pixel[1]))

                        # create sequence ((640, 480))
                        id_pc[j, i]=pixel_y*640+pixel_x
                        id_pc_1[j, i] = pixel_x * 480 + pixel_y
                        id_pc_2[j,i]=depth_image[pixel_y, pixel_x]
                else:
                    frame_id-=1
                pcd.append(PointCloud())
                pcd_demo.append(PointCloud())

                # print sequence
                # print("Seq 0: ", id_pc[:,i])
                # print("Seq 1: ", id_pc_1[:, i])
                # print("Seq 2: ", id_pc_2[:, i])

                # this is for test
                depth_image_dummy = np.ones((480, 640)) * 10000
                for ii in range(35):
                    depth_image_dummy[int(corners[ii, 0, 1]), int(corners[ii, 0, 0])] = 1
                depth_image_dummy=np.uint16(depth_image_dummy)

                # end of test

                # convert to Image object
                depth_raw = Image(depth_image)
                depth_raw_dummy = Image(depth_image_dummy)
                color_raw = Image(color_image)

                # create for rbgd
                rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw_dummy,
                                                                    depth_scale=1/depth_scale
                                                                    , depth_trunc=10
                                                                    , convert_rgb_to_intensity=False)
                rgbd_image_demo = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                                    depth_scale=1 / depth_scale
                                                                    , convert_rgb_to_intensity=False)

                # create point cloud for the ith camera
                pcd[i] = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                # for demo
                pcd_demo[i] = create_point_cloud_from_rgbd_image(rgbd_image_demo, pinhole_camera_intrinsic)

                # select points
                # id_pc_2[:, i] = pick_points(pcd[i])

                print("id pc 2 shape: ", id_pc_2.shape)
                # print("picked id: ")
                # print(np.argsort(picked_id_source))
                # print("detect id-depth: ")
                # print(np.argsort(id_pc_2[:,i]))




                if frame_id == 1:
                    break
                # [25L, 27L, 29L, 30L, 32L, 33L, 34L, 17L, 19L, 21L, 23L, 26L, 28L, 31L, 8L, 11L, 14L, 16L, 20L, 22L, 24L, 3L, 5L, 7L, 10L, 13L, 15L, 18L, 0L, 1L, 2L, 4L, 6L, 9L, 12L]
        finally:
            pipeline .stop()

    print("picked ID: ")
    print(id_pc_2)
    print("pcd:")
    print(np.asarray(pcd[0].points).shape)

    id1 = np.zeros((35, 1))
    id2 = np.zeros((35, 1))
    id3 = np.zeros((35, 1))
    id4 = np.zeros((35, 1))

    # dummy_seq=np.sort(id_pc_2, axis=0)
    # for k in range(35):
    #     id1[k] = np.searchsorted(dummy_seq[:,0], id_pc_2[k, 0])
    #     id2[k] = np.searchsorted(dummy_seq[:,1], id_pc_2[k, 1])
    #     id3[k] = np.searchsorted(dummy_seq[:,2], id_pc_2[k, 2])
    #     id4[k] = np.searchsorted(dummy_seq[:,3], id_pc_2[k, 3])
    # print(id1)

    ## all manual
    for k in range(35):
        id1[k] = id_pc_2[k,0]
        id2[k] = id_pc_2[k,1]
        id3[k] = id_pc_2[k,2]
        id4[k] = id_pc_2[k,3]
    print(id1)

    # id1 = id_pc_2[:, 0].reshape((35 ,1))
    # id2 = id_pc_2[:, 1].reshape((35 ,1))
    # id3 = id_pc_2[:, 2].reshape((35 ,1))
    # id4 = id_pc_2[:, 2].reshape((35 ,1))
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

# def test():
#     a=np.array([34L, 27L, 20L, 13L, 6L, 33L, 26L, 19L, 12L, 5L, 32L, 25L, 18L, 11L, 4L, 31L, 24L, 17L, 10L, 3L, 30L, 23L, 16L, 9L, 2L, 29L, 22L, 15L, 8L, 1L, 28L, 21L, 14L, 7L, 0L])
#
#     b1=np.array([2213., 2103., 1965., 1808., 1666., 2195., 2076., 1928., 1790., 1654., 2178., 2038.,
#  1911., 1751., 1606., 2148., 2012., 1872., 1737., 1592., 2128., 1986., 1838., 1700.,
#  1561., 2076., 1938., 1805., 1654., 1518., 2049., 1904., 1745., 1622., 1503.])


    print(np.argsort(a))
    print(np.argsort(b1))
if __name__ == "__main__":
    # help(cv2.findChessboardCorners)
    checkerboard_cali()
    test()