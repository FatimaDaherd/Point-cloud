import open3d as o3d

# Path to the saved mesh file
mesh_filename = "processed_mesh.ply"

# Load the mesh from the file
mesh = o3d.io.read_triangle_mesh(mesh_filename)

# Check if the mesh is successfully loaded
if not mesh.is_empty():
    print("Mesh loaded successfully.")
else:
    print("Failed to load mesh.")

# Visualize the loaded mesh
o3d.visualization.draw_geometries([mesh], window_name="Loaded Mesh")
