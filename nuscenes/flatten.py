# https://claude.ai/share/f030e448-a18e-415c-8512-5fb080fa2478
import numpy as np
import open3d as o3d
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
import matplotlib.pyplot as plt
import os

# Path to your nuScenes dataset
NUSCENES_DIR = r".\\v1.0-mini"

# Load nuScenes with the mini split
nusc = NuScenes(version='v1.0-mini', dataroot=NUSCENES_DIR, verbose=True)

sample = nusc.sample[0]
lidar_token = sample['data']['LIDAR_TOP']
lidar_data = nusc.get('sample_data', lidar_token)

lidar_path = os.path.join(NUSCENES_DIR, lidar_data['filename'])
pc = LidarPointCloud.from_file(lidar_path)

# Prepare Open3D point cloud
points = pc.points[:3, :].T
# Apply simple Z filter
z_threshold = -1
filtered_points = points[points[:, 2] > z_threshold]
# Keep only X and Y coordinates for the BEV
bev_points = filtered_points[:, :2]  # Take only X and Y columns

print(f"Total points: {points.shape[0]}")
print(f"Filtered points: {filtered_points.shape[0]}")

# print("BEV points:")
# print(bev_points)

# Visualize the BEV
plt_fig = plt.figure(figsize=(10, 10))
plt.scatter(bev_points[:, 0], bev_points[:, 1], s=5)
plt.title('Bird\'s Eye View (BEV)')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.axis('equal')
plt.grid(True)
plt.show()

# 3D visualization
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(filtered_points)
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
o3d.visualization.draw_geometries([pcd, axis], window_name="Z filter")