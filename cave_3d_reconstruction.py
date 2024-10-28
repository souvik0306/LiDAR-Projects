import numpy as np
import open3d as o3d
from noise import pnoise2

# Create a cylinder mesh to represent the cave tunnel
radius = 5.0
height = 20.0
cave_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height, resolution=100, split=4)
cave_mesh.translate((0, 0, -height / 2))  # Center the cylinder

# Deform the cylinder walls using Perlin noise
vertices = np.asarray(cave_mesh.vertices)
scale = 4.0
freq = 1.0
for i in range(len(vertices)):
    x, y, z = vertices[i]
    theta = np.arctan2(y, x)
    n = pnoise2(theta * freq, z * freq, octaves=4)
    displacement = n * scale
    r = np.sqrt(x**2 + y**2) + displacement
    vertices[i][0] = r * np.cos(theta)
    vertices[i][1] = r * np.sin(theta)

# Update the mesh with deformed vertices
cave_mesh.vertices = o3d.utility.Vector3dVector(vertices)

# *** Recompute normals after deformation ***
cave_mesh.compute_vertex_normals()
cave_mesh.compute_triangle_normals()

# *** Manually invert normals and reverse triangle winding ***
# Invert vertex normals
cave_mesh.vertex_normals = o3d.utility.Vector3dVector(-np.asarray(cave_mesh.vertex_normals))

# Reverse triangle winding
triangles = np.asarray(cave_mesh.triangles)
triangles = triangles[:, [0, 2, 1]]  # Swap the second and third vertices
cave_mesh.triangles = o3d.utility.Vector3iVector(triangles)

# *** Assign colors to the mesh vertices ***
# For example, assign colors based on the z-coordinate
vertices = np.asarray(cave_mesh.vertices)
colors = np.zeros_like(vertices)
z_min = vertices[:, 2].min()
z_max = vertices[:, 2].max()
z_norm = (vertices[:, 2] - z_min) / (z_max - z_min)

# Assign colors (customize as needed)
colors[:, 0] = z_norm                # Red channel
colors[:, 1] = 1.0 - z_norm          # Green channel
colors[:, 2] = np.abs(np.sin(z_norm * np.pi))  # Blue channel

# Attach the colors to the mesh
cave_mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

# Sample points from the mesh (colors will be transferred)
pcd_cave = cave_mesh.sample_points_poisson_disk(number_of_points=30000)

# Verify that the point cloud has colors
print("Point cloud has colors:", pcd_cave.has_colors())

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd_cave])

# Proceed with the rest of your code...

# Remove statistical outliers
pcd_clean, ind = pcd_cave.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Downsample the point cloud
voxel_size = 0.1  # Adjust based on your data
pcd_down = pcd_clean.voxel_down_sample(voxel_size=voxel_size)

# Ensure that colors are preserved after downsampling
print("Downsampled point cloud has colors:", pcd_down.has_colors())

# Estimate normals
pcd_down.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30)
)
pcd_down.orient_normals_consistent_tangent_plane(k=10)

# Poisson Surface Reconstruction
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Info):
    depth = 9  # Adjust for level of detail
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd_down, depth=depth
    )

# Remove low-density vertices to clean up the mesh
densities = np.asarray(densities)
density_threshold = np.quantile(densities, 0.01)
vertices_to_remove = densities < density_threshold
mesh.remove_vertices_by_mask(vertices_to_remove)

# Simplify and smooth the mesh
target_triangles = 10000
mesh_simplified = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles)
mesh_smooth = mesh_simplified.filter_smooth_simple(number_of_iterations=5)

# Transfer colors from the point cloud to the mesh vertices
pcd_tree = o3d.geometry.KDTreeFlann(pcd_down)
mesh_vertex_colors = []

mesh_vertices = np.asarray(mesh_smooth.vertices)
for vertex in mesh_vertices:
    # Perform nearest neighbor search
    [_, idx, _] = pcd_tree.search_knn_vector_3d(vertex, 1)
    # Get the color of the nearest point
    color = np.asarray(pcd_down.colors)[idx[0]]
    # Append the color to the list
    mesh_vertex_colors.append(color)

# Assign the colors to the mesh vertices
mesh_smooth.vertex_colors = o3d.utility.Vector3dVector(mesh_vertex_colors)

# Visualize the colored mesh
o3d.visualization.draw_geometries([mesh_smooth])
