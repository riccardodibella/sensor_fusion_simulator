import numpy as np
import open3d as o3d
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
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
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Add coordinate frame for reference
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)

o3d.visualization.draw_geometries(
    [pcd, axis],
    window_name="nuScenes LIDAR Point Cloud",
    width=1024,
    height=768
)