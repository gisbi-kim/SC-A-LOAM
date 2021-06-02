import os 
import sys
import time 
from io import StringIO
import pypcd # for the install, use this command: python3.x (use your python ver) -m pip install --user git+https://github.com/DanielPollithy/pypcd.git
from pypcd import pypcd
import numpy as np
import open3d as o3d

##########################
# User only consider this block
##########################
# data_dir = "/home/user/Documents/catkin2021/catkin_scaloam_util/data_tuto/" # should end with / 
data_dir = "sample_data/KAIST03/"
scan_idx_range_to_stack = [0, 20] # if you want a whole map, use [0, len(scan_files)]
node_skip = 1

is_live_vis = False
is_o3d_vis = False
##########################


#
scan_dir = data_dir + "Scans"
scan_files = os.listdir(scan_dir) 
scan_files.sort()

poses = []
f = open(data_dir+"optimized_poses.txt", 'r')
while True:
    line = f.readline()
    if not line: break
    pose_SE3 = np.asarray([float(i) for i in line.split()])
    pose_SE3 = np.vstack( (np.reshape(pose_SE3, (3, 4)), np.asarray([0,0,0,1])) )
    poses.append(pose_SE3)
f.close()


#
assert (scan_idx_range_to_stack[1] > scan_idx_range_to_stack[0])
print("Merging scans from", scan_idx_range_to_stack[0], "to", scan_idx_range_to_stack[1])


#
if(is_live_vis):
    vis = o3d.visualization.Visualizer() 
    vis.create_window('Map', visible = True) 

nodes_count = 0
pcd_combined_for_vis = o3d.geometry.PointCloud()
pcd_combined_for_save = None

# The scans from 000000.pcd should be prepared if it is not used (because below code indexing is designed in a naive way)
for node_idx in range(len(scan_files)):
    if(node_idx < scan_idx_range_to_stack[0] or node_idx >= scan_idx_range_to_stack[1]):
        continue

    nodes_count = nodes_count + 1
    if( nodes_count % node_skip is not 0): 
        if(node_idx is not scan_idx_range_to_stack[0]): # to ensure the vis init 
            continue

    scan_pose = poses[node_idx]

    scan_path = os.path.join(scan_dir, scan_files[node_idx])
    scan_pcd = o3d.io.read_point_cloud(scan_path)
    scan_pcd_global = scan_pcd.transform(scan_pose) # global coord
    scan_pcd_global_xyz = np.asarray(scan_pcd_global.points)

    if(is_o3d_vis):
        pcd_combined_for_vis += scan_pcd_global

    if is_live_vis:
        if(node_idx is scan_idx_range_to_stack[0]): # to ensure the vis init 
            vis.add_geometry(pcd_combined_for_vis) 

        vis.update_geometry(pcd_combined_for_vis)
        vis.poll_events()
        vis.update_renderer()

    # because open3d does not support intensity read and write, we use the pypcd 
    scan_pcd_with_intensity = pypcd.PointCloud.from_path(scan_path)
    print("scan", node_idx, "(using", node_skip, "skips)", "has", scan_pcd_with_intensity.pc_data.shape[0], "points")

    scan_pcd_with_intensity_global = scan_pcd_with_intensity
    scan_pcd_with_intensity_global.pc_data['x'] = scan_pcd_global_xyz[:, 0]
    scan_pcd_with_intensity_global.pc_data['y'] = scan_pcd_global_xyz[:, 1]
    scan_pcd_with_intensity_global.pc_data['z'] = scan_pcd_global_xyz[:, 2]
    if( node_idx == scan_idx_range_to_stack[0]):
        pcd_combined_for_save = scan_pcd_with_intensity_global
    else:
        pcd_combined_for_save = pypcd.cat_point_clouds(pcd_combined_for_save, scan_pcd_with_intensity_global)
    

if(is_o3d_vis):
    print("draw the merged map ")
    o3d.visualization.draw_geometries([pcd_combined_for_vis])

map_name = data_dir + "map_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + ".pcd"
pcd_combined_for_save.save_pcd(map_name, compression='binary_compressed')
print("map is save (path:", map_name, ")")


