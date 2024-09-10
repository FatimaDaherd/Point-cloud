import open3d as o3d
import time

start_time = time.time()

# Load a sample point cloud from Stanford 3D Scanning Repository
pcd = o3d.io.read_point_cloud("xyzrgb_dragon.ply")

end_time = time.time()
# Visualize the point cloud with normals
o3d.visualization.draw_geometries([pcd], window_name="Point Cloud")


elapsed_time = end_time - start_time
print(f"Elapsed time: {elapsed_time} seconds")