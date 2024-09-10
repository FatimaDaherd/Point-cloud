import open3d as o3d

# Load the Gemstone.ply file into a PointCloud object
pcd = o3d.io.read_point_cloud("Gemstone.ply")

# Set the alpha parameter for alpha shape computation
alpha = 5

# Create a triangle mesh from the point cloud using alpha shape
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

# Compute vertex normals for the mesh
mesh.compute_vertex_normals()

# Save the resulting mesh to a file (e.g., "result_mesh.obj")
#o3d.io.write_triangle_mesh("result_mesh.ply", mesh)

# Visualize the resulting mesh with back faces shown
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


