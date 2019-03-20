from open3d import *
import pyrealsense2 as rs
import numpy as np
from datetime import datetime
from pc_utils import *
import copy
import time
from PC_functions import *


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

def demo_manual_registration_global(source, target):
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))
    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    #picked_id_target = np.asarray(target.points)
    picked_id_target = pick_points(source)
    assert(len(picked_id_source)>=3 and len(picked_id_target)>=3)
    assert(len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source),2))
    corr[:,0] = picked_id_source
    corr[:,1] = picked_id_target
    print(np.asarray(source.points).shape)
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

if __name__ == "__main__":
    ## Global Points for registration
    no_points = 12
    xyz = np.zeros((no_points, 3))
    xyz[0, :] = np.array([0, 0, 0])
    xyz[1, :] = np.array([0, 1, 0])
    xyz[2, :] = np.array([0, 2, 0])
    xyz[3, :] = np.array([0, 3, 0])
    xyz[4, :] = np.array([0, 4, 0])
    xyz[5, :] = np.array([0, 5, 0])
    xyz[6, :] = np.array([0, 6, 0])
    xyz[7, :] = np.array([0, 7, 0])
    xyz[8, :] = np.array([1, 0, 0])
    xyz[9, :] = np.array([1, 1, 0])
    xyz[10, :] = np.array([1, 2, 0])
    xyz[11, :] = np.array([1, 3, 0])
    # Pass xyz to Open3D.PointCloud and visualize
    pcd = PointCloud()
    pcd.points = Vector3dVector(xyz)
    target = pcd

    clipping_distance_in_meters=5
    cam_id=0
    for SID in [822512060625, 821312060330, 821212061385, 822512060979]:
        source=GetPC(SID, clipping_distance_in_meters)
        demo_crop_geometry()
        cam_id+=1
        print("Cam:"+str(cam_id))
        demo_manual_registration_global(source, target)
