#This code performs several operations on a 3D point cloud loaded from a PLY file named "Gemstone.ply." It first removes statistical outliers from the point cloud to clean the data. The point cloud is then converted to a NumPy array for easier manipulation, and a region of interest is defined to filter points within specified coordinate ranges. The filtered points are used to create a new point cloud, for which normals are estimated and oriented consistently. The code then generates a Poisson surface reconstruction mesh from this cropped point cloud, simplifies the mesh by reducing the number of triangles, applies Laplacian smoothing to refine the mesh, and removes non-manifold edges and degenerate triangles. Finally, the resulting mesh is visualized, and the total elapsed processing time is printed. The mesh can optionally be saved to a file.
import open3d as o3d
import time

start_time = time.time()

# Load the point cloud from a PLY file
dataname = "Gemstone.ply"
pcd = o3d.io.read_point_cloud(dataname)

# Remove statistical outliers
pcd = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)[0]

# Crop the point cloud to its axis-aligned bounding box
bbox = pcd.get_axis_aligned_bounding_box()
pcd = pcd.crop(bbox)

# Estimate normals for the point cloud with adjusted parameters
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
pcd.orient_normals_consistent_tangent_plane(50)

# Create Poisson surface reconstruction mesh from the point cloud with adjusted parameters
poisson_mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10, scale=1.0)

# Simplify the mesh to reduce the number of triangles
poisson_mesh = poisson_mesh.simplify_quadric_decimation(target_number_of_triangles=60000)

# Apply Laplacian smoothing
poisson_mesh = poisson_mesh.filter_smooth_laplacian(number_of_iterations=50)


# Remove non-manifold edges
poisson_mesh.remove_non_manifold_edges()
poisson_mesh.remove_degenerate_triangles()


# Save the resulting mesh to a file
#output_filename = "processed_mesh.ply"
#o3d.io.write_triangle_mesh(output_filename, poisson_mesh)

end_time = time.time()

# Visualize the resulting mesh
o3d.visualization.draw_geometries([poisson_mesh], window_name="Mesh Poisson")

elapsed_time = end_time - start_time
print(f"Elapsed time: {elapsed_time} seconds")