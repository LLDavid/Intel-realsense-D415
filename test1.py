from open3d import *
import pyrealsense2 as rs
import numpy as np
import cv2
from ManualRegister_2PC import *
from datetime import datetime
from register_utils import *
from pc_utils import *
from global_register import *

def test_debug():
    PATTERN_SIZE=(5, 7)
    corners_pc=np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1],3,4))
    ## get corner points on checkerboard
    #SID_list = [822512060625, 821312060330, 821212061385, 822512060979]
    SID_list = [22512060625]
    pcd=list(range(2))
    for i in range(len(SID_list)):
        SID=SID_list[i]
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
                pinhole_camera_intrinsic = PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))

                ## create pcd from rgbd image
                ## Get RGBD image
                depth_raw = Image(np.array(aligned_depth_frame.get_data()))
                color_raw = Image(np.array(color_frame.get_data()))
                rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                                     depth_scale=1/depth_scale\
                                                                    , depth_trunc=2\
                                                                    , convert_rgb_to_intensity=False)
                pcd_temp = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                #cl,pcd[i]=statistical_outlier_removal(pcd[i],
                                           # nb_neighbors=20, std_ratio=2.0)
                pcd[i]=voxel_down_sample(pcd_temp, voxel_size = 0.02)
                # from
                frame_id+=1
                if frame_id == 1:
                    break
        finally:
            pipeline.stop()
    demo_manual_registration_global(pcd[0], pcd[1])
    return 0

if __name__ == "__main__":
    help(auto_registration)
    test_debug()
