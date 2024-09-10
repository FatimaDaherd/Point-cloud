def process_point_cloud(filename, normal_radius=0.2, max_nn=50, poisson_depth=10, poisson_scale=1.0, target_triangles=60000, smooth_iterations=100):
    import open3d as o3d

    # Load the point cloud from a PLY file
    pcd = o3d.io.read_point_cloud(filename)

    # Optional: Remove duplicate points
    pcd = pcd.remove_duplicated_points()

    # Remove statistical outliers
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Estimate normals for the point cloud with adjusted parameters
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=max_nn))
    pcd.orient_normals_consistent_tangent_plane(50)

    # Create Poisson surface reconstruction mesh from the point cloud with adjusted parameters
    poisson_mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=poisson_depth, scale=poisson_scale)

    # Simplify the mesh to reduce the number of triangles
    poisson_mesh = poisson_mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles)

    # Apply Laplacian smoothing
    poisson_mesh = poisson_mesh.filter_smooth_laplacian(number_of_iterations=smooth_iterations)

    # Visualize the resulting mesh
    o3d.visualization.draw_geometries([poisson_mesh], window_name="Mesh Poisson")

    # Save the resulting mesh to a file
    #output_filename = "processed_mesh.ply"
    #o3d.io.write_triangle_mesh(output_filename, poisson_mesh)

process_point_cloud("Gemstone.ply", normal_radius=0.2, max_nn=50, poisson_depth=12, poisson_scale=1.1, target_triangles=60000, smooth_iterations=100)

