import numpy as np
import open3d as o3d
from noise import pnoise2

# Create a cylinder mesh to represent the cave tunnel
radius = 5.0
height = 20.0
cave_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height, resolution=100, split=4)
cave_mesh.compute_vertex_normals()
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

# Update the mesh
cave_mesh.vertices = o3d.utility.Vector3dVector(vertices)
cave_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(cave_mesh.triangles)[:, [0, 2, 1]])
cave_mesh.vertex_normals = o3d.utility.Vector3dVector(-np.asarray(cave_mesh.vertex_normals))

# Sample points
pcd_cave = cave_mesh.sample_points_poisson_disk(number_of_points=30000)

# Visualize
o3d.visualization.draw_geometries([pcd_cave])
