from open3d import *
import pyrealsense2 as rs
import numpy as np
import cv2
from ManualRegister_2PC import *
from datetime import datetime
from register_utils import *
from pc_utils import *




def checkerboard_cali_1():
    PATTERN_SIZE=(5, 7)
    corners_pc=np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1],3,4))
    ## get corner points on checkerboard
    SID_list = [822512060625, 821312060330, 821212061385, 822512060979]
    #SID_list = [822512060979, 821312060330]
    for i in range(len(SID_list)):
        SID=SID_list[i]
        profile, pipeline, depth_scale = get_profile(SID)
        profile, pipeline, depth_scale = get_profile(SID)
        print("depth scale:"+str(depth_scale))
        align_to = rs.stream.color
        align = rs.align(align_to)
        try:
            frame_id = 0
            while True:
                # Get aligned frame
                frames = pipeline.wait_for_frames()
                aligned_frames_rgbd = align.process(frames)
                # not use the aligned depth frame for mannual deprojection
                aligned_frames = frames
                aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()
                if not color_frame or not aligned_frames:
                    continue
                # Transform color and depth images to np array
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                # Count frame
                frame_id+=1
                # Get intrinsic parameters
                tt = profile.get_stream(rs.stream.depth)
                depth_intr = tt.as_video_stream_profile().get_intrinsics()
                depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
                pinhole_camera_intrinsic = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))

                found, corners, start_id = find_checkerboard(color_image, PATTERN_SIZE)

                # Create a buffer depth image
                depth_image_rgbd=np.ones(depth_image.shape)*100

                if found:
                    for j in range(PATTERN_SIZE[0]*PATTERN_SIZE[1]):
                        ## 1st method: calculate from intrinsic parameters
                        ## 2nd method: use the deproject funciton
                        ## 3rd method: create from rgbd image

                        pixel=[corners[j, 0, 0], corners[j, 0, 1]]
                        pixel_x = int(np.round(pixel[0]))
                        pixel_y = int(np.round(pixel[1]))
                        depth_value=depth_image[pixel_y, pixel_x]
                        points = rs.rs2_deproject_pixel_to_point(depth_intr, pixel, depth_scale)
                        points = rs.rs2_transform_point_to_point(depth_to_color_extrin, points)
                        points[2]= depth_value
                        corners_pc[j, 0, i] = points[0]
                        corners_pc[j, 1, i] = points[1]
                        corners_pc[j, 2, i] = points[2]
                        depth_image_rgbd[pixel_y, pixel_x]=depth_value

                    # print("here")
                    for ii in range(480):
                        for jj in range(640):
                            depth_image_rgbd[ii,jj]=(jj+1+ii*640)

                    print("finished")
                    ## create pcd from rgbd image
                    ## Get RGBD image
                    depth_image_rgbd=depth_image_rgbd.astype(np.float32)
                    # print(depth_image_rgbd.shape)
                    # print(depth_image_rgbd)
                    depth_raw = Image(depth_image_rgbd)
                    color_raw = Image(np.array(color_frame.get_data()))
                    rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                                        depth_scale=1\
                                                                        , depth_trunc=10\
                                                                        , convert_rgb_to_intensity=False)
                    # print("rgbd.images")
                    # #print(rgbd.image)
                    # print("rbgd.depth:")
                    # aaa=rgbd_image.depth
                    # print(np.asarray(rgbd_image.depth))
                    pcd_rgbd = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                else:
                    frame_id-=1
                # from points


                if frame_id == 1:
                    break
        finally:
            pipeline .stop()
    pcd_1 = PointCloud()
    pcd_1.points = Vector3dVector(corners_pc[:, :, 0])
    print("mannual:")
    print(np.asarray(pcd_1.points))
    print("rgbd:")
    rgbd_points=np.asarray(pcd_rgbd.points)
    print(rgbd_points.shape)
    print(np.asarray(pcd_rgbd.points))
    picked_id_sequence=np.repeat(np.arange(35).reshape(35, 1), 2, axis=1)
    #print(picked_id_sequence)
    RT1 = auto_registration(pcd_1, pcd_1, picked_id_sequence)

    return RT1

if __name__ == "__main__":
    help(create_rgbd_image_from_color_and_depth)
    checkerboard_cali_1()
