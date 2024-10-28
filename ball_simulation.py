
import numpy as np
import open3d as o3d

# Create a plane
plane = o3d.geometry.TriangleMesh.create_box(width=20, height=20, depth=0.1)
plane.translate((-10, -10, -0.05))  # Position the plane

# Create a sphere
sphere = o3d.geometry.TriangleMesh.create_sphere(radius=2.0)
sphere.translate((-5, 0, 2))  # Initial position of the sphere

# Sample points from the meshes
pcd_plane = plane.sample_points_poisson_disk(number_of_points=1000)
pcd_sphere = sphere.sample_points_poisson_disk(number_of_points=500)

# Combine the point clouds
pcd_combined = pcd_plane + pcd_sphere

# Initialize the visualizer
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Moving Object Simulation')

# Add the combined point cloud to the visualizer
vis.add_geometry(pcd_combined)

# Set rendering options (optional)
render_option = vis.get_render_option()
render_option.background_color = np.asarray([0.0, 0.0, 0.0])  # Black background

# Run the animation
for i in range(100):
    # Move the sphere along the x-axis
    sphere.translate((0.1, 0, 0), relative=True)
    
    # Re-sample the points from the moved sphere
    pcd_sphere.points = sphere.sample_points_poisson_disk(number_of_points=500).points
    
    # Update the combined point cloud
    pcd_combined.points = o3d.utility.Vector3dVector(
        np.vstack((np.asarray(pcd_plane.points), np.asarray(pcd_sphere.points)))
    )
    
    # Update the geometry in the visualizer
    vis.update_geometry(pcd_combined)
    vis.poll_events()
    vis.update_renderer()

# Close the visualizer window
vis.destroy_window()
