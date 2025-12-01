import copy
import open3d as o3d
import numpy as np
import os
from glob import glob

def generate_mesh(pcd):
        
        if not pcd.has_points():
            print("Error: Point cloud is empty")
            return
        
        print(f"Point cloud loaded with {len(pcd.points)} points")
        
        if not pcd.has_normals():
            print("Estimating normals...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
            pcd.orient_normals_consistent_tangent_plane(k=15)
        
        print("Generating mesh using Poisson reconstruction...")
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9
        )
        
        densities = np.asarray(densities)
        density_threshold = np.quantile(densities, 0.01)
        vertices_to_remove = densities < density_threshold
        mesh.remove_vertices_by_mask(vertices_to_remove)
        
        print(f"Mesh generated with {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")
        return mesh

def draw_comparisons(source, target, title="BHO"):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name=title)
    
#==========Registration Functions #start
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source,target,voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def  get_corrective_transformation(source,target, voxel_size = 0.05,threshold = 0.02):       # ---- Registration ----
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source,target,voxel_size)
        result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
        print(result_ransac)
        trans_init =result_ransac.transformation
        """ evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold,trans_init) """
        reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        return reg_p2p.transformation
#==========Registration Functions #end 
def transform_pcl_to_origin(pcl):
    known_starting_transformation = np.array([0., 20., 0.])  # known translation vector
    pts = np.asarray(pcl.points)
    pts -= known_starting_transformation
    pcl.points = o3d.utility.Vector3dVector(pts)
    
def get_scale_factor_to_height(mesh, target_dim):
    """
    Scales the mesh such that its largest bounding box dimension matches the target_dim.
    
    Args:
    - mesh (o3d.geometry.TriangleMesh): The mesh to be scaled.
    - target_dim (float): The target dimension (in meters, for example) to scale the mesh to.
    
    Returns:
    - o3d.geometry.TriangleMesh: The scaled mesh.
    """
    
    # Compute the axis-aligned bounding box of the mesh
    bbox = mesh.get_axis_aligned_bounding_box()
    
    # Get the current size (dimension) of the bounding box in each direction (x, y, z)
    height = bbox.get_extent()[2]  # This gives the [width, height, depth] of the bounding box
    
    
    # Calculate the scaling factor needed to match the target dimension
    scale_factor = target_dim / height

    return scale_factor

def scale_mantaing_position(mesh, scale_factor):
    points = np.asarray(mesh.vertices)

    # Shift the Z values (make minimum Z value zero)
    z_min = points[:, 2].min()
    points[:, 2] -= z_min

    points *= scale_factor

    # Create a new PointCloud object with the scaled points
    scaled_point_cloud = o3d.geometry.PointCloud()
    scaled_point_cloud.points = o3d.utility.Vector3dVector(points)


# -----------------------------
# Paths
# -----------------------------
if __name__ == "__main__":
    ref_path = "/ros2_ws/src/backup/construction_cone.ply"
    input_dir = "/ros2_ws/src/backup/working_directory/working_directory/mesh_cone/history_all_mesh"       # directory containing mesh_cloud*.ply
    output_dir = "/ros2_ws/src/backup/results/"  # output directory

    os.makedirs(output_dir, exist_ok=True)
    SAMPLES_FOR_THE_DISTANCE = 500000
    reference_geometry_height = 1.08


    # -----------------------------
    # Load reference mesh (only once)
    # -----------------------------
    ref = o3d.io.read_triangle_mesh(ref_path)
    #rotate the geometry since is different modelling programs use different definitions of height axis

    ply_files = glob(os.path.join(input_dir, "*.ply"))
    ref.rotate(ref.get_rotation_matrix_from_xyz(( -np.pi/2, 0,0)), center=(0, 0, 0)) 

    # Get the most complete poincloud as reference
    ply_files = [f for f in ply_files if os.path.abspath(f) != os.path.abspath(ref_path)]
    pointclouds = [(f, o3d.io.read_point_cloud(f)) for f in ply_files]
    largest_file, largest_pcl = max(pointclouds, key=lambda x: len(x[1].points))
    draw_comparisons(largest_pcl,ref,"As Loaded")
    print(f"Largest file: {largest_file}")
    print(f"Point count: {len(largest_pcl.points)}")
    
    # Bring the poincloud as close as possible to the origin knowing the true position of the object
    transform_pcl_to_origin(largest_pcl)
    print(ref.get_axis_aligned_bounding_box())
    print(largest_pcl.get_axis_aligned_bounding_box())

    draw_comparisons(largest_pcl,ref,"Before Scaling")

    
    # Since the gazebo mesh model has a different height than in the simulation, it must be rescaled
    scale_factor = get_scale_factor_to_height(ref, reference_geometry_height)
    print("computed Scale factor", scale_factor)
    scale_mantaing_position(ref, scale_factor)# 1.18/4.29(dimension reported by Gazebo)/(Dimension repoted by Blender)
    draw_comparisons(largest_pcl,ref,"after Scaling")

    print("reference, scaled to:",ref.get_axis_aligned_bounding_box())
    ref_pts = ref.sample_points_uniformly(number_of_points=SAMPLES_FOR_THE_DISTANCE)
    
    """     corrective_transformation = get_corrective_transformation(largest_pcl,ref_pts)
    largest_pcl.transform(corrective_transformation)
    o3d.visualization.draw_geometries([largest_pcl, ref], window_name="alligment with complete pcl") """


    if not ply_files:
        print("No PLY files found to process.")
        exit()


    for pcl_file in ply_files:
        print(f"\n------------------------------------------------------")
        print(f"Processing file: {pcl_file}")
        print(f"------------------------------------------------------")

        # Load reconstruction mesh
        rec = o3d.io.read_point_cloud(pcl_file)
        transform_pcl_to_origin(rec)
        T = get_corrective_transformation(rec,ref_pts)
        rec.transform(T)
        draw_comparisons(rec,ref,"after Scaling")


        rec = generate_mesh(rec)
        
        # ---- Sampling ----
        rec_pts = rec.sample_points_uniformly(number_of_points=SAMPLES_FOR_THE_DISTANCE)
        
        draw_comparisons(rec_pts, ref_pts,"Models on which compute the distances")
        

        # ---- Distances ----
        dist_ref_to_rec = np.asarray(ref_pts.compute_point_cloud_distance(rec_pts))
        dist_rec_to_ref = np.asarray(rec_pts.compute_point_cloud_distance(ref_pts))

        chamfer = dist_ref_to_rec.mean() + dist_rec_to_ref.mean()
        hausdorff = max(dist_ref_to_rec.max(), dist_rec_to_ref.max())
        print("Chamfer Distance:", chamfer)
        print("Hausdorff Distance:", hausdorff)

        # ---- Save results ----
        base_name = os.path.splitext(os.path.basename(pcl_file))[0]

        rec_out = os.path.join(output_dir, f"{base_name}_recentered.obj")
        ref_out = os.path.join(output_dir, "construction_cone_scaled.obj")

        o3d.io.write_triangle_mesh(rec_out, rec)
        o3d.io.write_triangle_mesh(ref_out, ref)  # saved once; overwritten but identical

        print(f"Saved: {rec_out}")


""" import open3d as o3d
import numpy as np

# Load meshes
ref = o3d.io.read_triangle_mesh("/ros2_ws/src/backup/construction_cone.ply")
rec = o3d.io.read_triangle_mesh("/ros2_ws/src/backup/mesh_cloud.ply")


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
 """