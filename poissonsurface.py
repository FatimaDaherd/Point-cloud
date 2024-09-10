import open3d as o3d

# Load the point cloud from a PLY file
dataname = "Gemstone.ply"
pcd = o3d.io.read_point_cloud(dataname)

pcd = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)[0]

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

# Calculate the volume of the mesh
if poisson_mesh.is_watertight():
    volume = poisson_mesh.get_volume()
    print(f"Volume of the mesh: {volume}")
else:
    print("The mesh is not watertight, unable to calculate volume.")

# Save the resulting mesh to a file
output_filename = "processed_mesh_dragon.ply"
o3d.io.write_triangle_mesh(output_filename, poisson_mesh)

# Visualize the resulting mesh
o3d.visualization.draw_geometries([poisson_mesh], window_name="Mesh Poisson")
