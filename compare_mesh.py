import open3d as o3d
import numpy as np

# Load meshes
ref = o3d.io.read_triangle_mesh("/ros2_ws/src/backup/construction_cone.ply")
rec = o3d.io.read_triangle_mesh("/ros2_ws/src/backup/mesh_cloud.ply")

# Sample points on both surfaces
ref_pts = ref.sample_points_uniformly(number_of_points=500000)
rec_pts = rec.sample_points_uniformly(number_of_points=500000)

# Compute distances (Chamfer)
dist_ref_to_rec = np.asarray(ref_pts.compute_point_cloud_distance(rec_pts))
dist_rec_to_ref = np.asarray(rec_pts.compute_point_cloud_distance(ref_pts))

chamfer = dist_ref_to_rec.mean() + dist_rec_to_ref.mean()
hausdorff = max(dist_ref_to_rec.max(), dist_rec_to_ref.max())

print("Chamfer Distance:", chamfer)
print("Hausdorff Distance:", hausdorff)

#--------------------------------Recentering
# Known translation vector of the mesh
T = np.array([0., 20., 0.])   # <-- fill this in


# Apply inverse translation (recenter)
rec.vertices = o3d.utility.Vector3dVector(np.asarray(rec.vertices) - T)
#-----------------Flip axis
ref.rotate(ref.get_rotation_matrix_from_xyz((np.pi / 2,0, 0)),
              center=(0, 0, 0))
ref.rotate(ref.get_rotation_matrix_from_xyz((0,np.pi, 0)),
              center=(0, 0, 0))
#----------------Scaling
scale = 0.23
#ref.scale(scale, center=ref.get_center())
verts = np.asarray(ref.vertices)
# 1. Trova il minimo in Z (base)
z_min = verts[:, 2].min()

# 2. Sposta il modello in modo che la base sia a Z=0
verts[:, 2] -= z_min

# 3. Scala attorno alla base
verts *= scale

# 4. (non serve rialzare: la base è ancora a Z=0)
ref.vertices = o3d.utility.Vector3dVector(np.asarray(verts))

ref_pts = ref.sample_points_uniformly(number_of_points=500000)
rec_pts = rec.sample_points_uniformly(number_of_points=500000)

# Compute distances (Chamfer)
dist_ref_to_rec = np.asarray(ref_pts.compute_point_cloud_distance(rec_pts))
dist_rec_to_ref = np.asarray(rec_pts.compute_point_cloud_distance(ref_pts))

chamfer = dist_ref_to_rec.mean() + dist_rec_to_ref.mean()
hausdorff = max(dist_ref_to_rec.max(), dist_rec_to_ref.max())

print("Chamfer Distance:", chamfer)
print("Hausdorff Distance:", hausdorff)

app = o3d.visualization.gui.Application.instance
app.initialize()

w = o3d.visualization.O3DVisualizer("Compare Meshes", 1024, 768)
w.show_settings = True

w.add_geometry("Reference", ref)
w.add_geometry("Reconstruction", rec)

app.add_window(w)
app.run()


# Save result
o3d.io.write_triangle_mesh("/ros2_ws/src/backup/estimated_recentered.obj", rec)
o3d.io.write_triangle_mesh("/ros2_ws/src/backup/construction_cone_scaled.obj", ref)


