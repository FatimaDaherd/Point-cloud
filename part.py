#This code performs several operations on a 3D point cloud loaded from a PLY file named "Gemstone.ply." It first removes statistical outliers from the point cloud to clean the data. The point cloud is then converted to a NumPy array for easier manipulation, and a region of interest is defined to filter points within specified coordinate ranges. The filtered points are used to create a new point cloud, for which normals are estimated and oriented consistently. The code then generates a Poisson surface reconstruction mesh from this cropped point cloud, simplifies the mesh by reducing the number of triangles, applies Laplacian smoothing to refine the mesh, and removes non-manifold edges and degenerate triangles. Finally, the resulting mesh is visualized, and the total elapsed processing time is printed. The mesh can optionally be saved to a file.
import open3d as o3d
import numpy as np
import time

start_time = time.time()

# Load the point cloud from a PLY file
dataname = "Gemstone.ply"
pcd = o3d.io.read_point_cloud(dataname)

# Remove statistical outliers
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Convert the point cloud to a numpy array for easier manipulation
points = np.asarray(pcd.points)

# Define the region of interest (e.g., x, y, z coordinates ranges)
x_min, x_max = -10, 10
y_min, y_max = -10, 10
z_min, z_max = -10, 10

# Create a mask for points within the specified range
mask = (points[:, 0] >= x_min) & (points[:, 0] <= x_max) & \
       (points[:, 1] >= y_min) & (points[:, 1] <= y_max) & \
       (points[:, 2] >= z_min) & (points[:, 2] <= z_max)

# Apply the mask to filter the points
filtered_points = points[mask]

# Create a new point cloud with the filtered points
cropped_pcd = o3d.geometry.PointCloud()
cropped_pcd.points = o3d.utility.Vector3dVector(filtered_points)

# Estimate normals for the cropped point cloud
cropped_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
cropped_pcd.orient_normals_consistent_tangent_plane(50)

# Create Poisson surface reconstruction mesh from the cropped point cloud
poisson_mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cropped_pcd, depth=10, scale=1.0)

# Simplify the mesh to reduce the number of triangles
poisson_mesh = poisson_mesh.simplify_quadric_decimation(target_number_of_triangles=60000)

# Apply Laplacian smoothing
poisson_mesh = poisson_mesh.filter_smooth_laplacian(number_of_iterations=50)

# Remove non-manifold edges and degenerate triangles
poisson_mesh.remove_non_manifold_edges()
poisson_mesh.remove_degenerate_triangles()

# Save the resulting mesh to a file (uncomment to save)
output_filename = "processed_mesh.ply"
o3d.io.write_triangle_mesh(output_filename, poisson_mesh)

end_time = time.time()

# Visualize the resulting mesh
o3d.visualization.draw_geometries([poisson_mesh], window_name="Mesh Poisson")

elapsed_time = end_time - start_time
print(f"Elapsed time: {elapsed_time} seconds")
