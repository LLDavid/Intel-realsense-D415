from open3d import *
import pyrealsense2 as rs
import numpy as np
from datetime import datetime
from pc_utils import *
import copy
import time
import inspect


file_path_1 = '.\\ply\\' + 'test_1' + '.pcd'
file_path_2 = '.\\ply\\' + 'test_2' + '.pcd'
target= read_point_cloud(file_path_1)
source= read_point_cloud(file_path_2)


def demo_crop_geometry():
    print("Demo for manual geometry cropping")
    print("1) Press 'Y' twice to align geometry with negative direction of y-axis")
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    #pcd = read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    #draw_geometries_with_editing([pcd])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

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

def demo_manual_registration(source, target):
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    print(type(picked_id_source))
    print(picked_id_source)
    assert(len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert(len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target
    print(type(corr))
    print(corr)
    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
             Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03 # 3cm distance threshold
    reg_p2p = registration_icp(source, target, threshold, trans_init,
            TransformationEstimationPointToPoint())
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)
    print("")

def auto_registration(source, target, corr):
    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            Vector2iVector(corr))
    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = registration_icp(source, target, threshold, trans_init,
                               TransformationEstimationPointToPoint())
    print(reg_p2p.transformation)
    return reg_p2p.transformation

def test():
    color_raw=Image(np.uint8(np.ones((480,640,3))))
    depth_raw=Image(np.uint8(np.ones((480,640))*100))
    rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw,
                                                        depth_scale=1 / 0.001 \
                                                        , depth_trunc=100000000 \
                                                        , convert_rgb_to_intensity=False)
    pcd = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
    picked_id_source = pick_points(pcd)
    print(picked_id_source)

if __name__ == "__main__":
    # demo_crop_geometry(source, target)
    demo_manual_registration(source, target)
    # # test()
    # inspect.getsourcelines(pick_points)