import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# Step 1: Define input arrays
# Replace these arrays with your actual data
theta_array = np.linspace(0, 2 * np.pi, num=100)
radius_array = np.random.uniform(5, 10, size=theta_array.shape)

# Step 2: Create 2D profile and extrude along Z-axis
z_min = 0
z_max = 20
num_layers = 140
z_array = np.linspace(z_min, z_max, num_layers)

points = []
for z in z_array:
    # Example variation (customize as needed)
    radius_variation = radius_array + np.sin(z / z_max * 2 * np.pi)
    x_layer = radius_variation * np.cos(theta_array)
    y_layer = radius_variation * np.sin(theta_array)
    z_layer = np.full_like(x_layer, z)
    layer_points = np.vstack((x_layer, y_layer, z_layer)).T
    points.append(layer_points)

# Combine all layers
points = np.vstack(points)

# Step 3: Create point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Optional: Assign colors based on Z-coordinate
z_norm = (points[:, 2] - z_min) / (z_max - z_min)
colors = plt.get_cmap('viridis')(z_norm)[:, :3]  # Extract RGB values
pcd.colors = o3d.utility.Vector3dVector(colors)

# Step 4: Visualize point cloud
o3d.visualization.draw_geometries([pcd])

# Step 5: Estimate normals
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)

# Step 6: Poisson surface reconstruction
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Info):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9
    )

# Step 7: Remove low-density vertices
densities = np.asarray(densities)
density_threshold = np.quantile(densities, 0.01)
vertices_to_remove = densities < density_threshold
mesh.remove_vertices_by_mask(vertices_to_remove)

# Simplify and smooth the mesh
mesh_simplified = mesh.simplify_quadric_decimation(target_number_of_triangles=10000)
mesh_smooth = mesh_simplified.filter_smooth_simple(number_of_iterations=5)

# Step 8: Visualize the mesh
o3d.visualization.draw_geometries([mesh_smooth], mesh_show_back_face=True)
