import os
import datetime
import open3d as o3d
import numpy as np
import copy



class PointCloudManager:
    
    def __init__(self, detected_objects_list=None, point_clouds=None, output_dir=None, 
                 voxel_size=0.002, robot_namespace="robot"):
        
        
        self.detected_objects_list = detected_objects_list
        self.true_object_pcl = []
        self.point_clouds = point_clouds
        
        # Set default output directory with absolute path accessible by rosuser
        if output_dir is None:
            # Use ROS workspace directory which rosuser can access
            output_dir = "/ros2_ws/src/vision_system/ply_detected"
        self.output_dir = output_dir
        
        # Directory for filtered point clouds
        self.filtered_dir = "/ros2_ws/src/vision_system/ply_filtered"
        
        self.robot_namespace = robot_namespace
        
        self.voxel_size = voxel_size
        
        
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.filtered_dir, exist_ok=True)
        self.get_3d_points_from_bbox()
    
    def get_3d_points_from_bbox(self):
        if not self.detected_objects_list:
            print("[WARN] NO OBJECT.")
            return

        #print(f"[INFO] elboration of {len(self.detected_objects_list)} object detected...")
        for obj in self.detected_objects_list:
            label = obj.get('label', 'unknown')
            obj_id = obj.get('id', -1)
            points_3d = obj.get('pcl_object', [])

            if not points_3d:
                print(f"[WARN] no 3D point valid for object '{label}' (ID {obj_id}).")
                continue

            self.true_object_pcl.append({
                'label': label,
                'id': obj_id,
                'points': points_3d
            })
            #print(f"[INFO] object '{label}' (ID {obj_id}) -- relative points {len(points_3d)}")

    def save_object_pointcloud_to_file(self, nick_name="object"):
        
        if not self.true_object_pcl:
            print(f"[WARN] no object to save for true_object_pcl.")
            return
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        for obj_data in self.true_object_pcl:
            label = obj_data.get('label', 'unknown')
            object_id = obj_data.get('id', -1)
            points_3d = obj_data.get('points', [])

            if not points_3d:
                print(f"[WARN] no point for object: '{label}' (ID {object_id}).")
                continue

            # Format: label_robotnamespace_timestamp.ply
            filename = os.path.join(
                self.output_dir, 
                f"{label}_{self.robot_namespace}_{timestamp}.ply"
            )

            try:
                with open(filename, 'w') as f:
                    # Header PLY
                    f.write("ply\n")
                    f.write("format ascii 1.0\n")
                    f.write(f"comment object '{label}' (ID {object_id}) from robot '{self.robot_namespace}'\n")
                    f.write(f"element vertex {len(points_3d)}\n")
                    f.write("property float x\n")
                    f.write("property float y\n")
                    f.write("property float z\n")
                    f.write("end_header\n")

                    # Dati dei punti
                    for p in points_3d:
                        x = p.get('x', 0.0)
                        y = p.get('y', 0.0)
                        z = p.get('z', 0.0)
                        f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

                print(f"[OK!!] SAVED {len(points_3d)} Points in: '{filename}'.")

            except Exception as e:
                print(f"[ERROR] Error during save file PLY: {e}")

    def load_point_clouds_from_files(self, file_paths):
        
        for file_path in file_paths:
            if not os.path.exists(file_path):
                print(f"[WARN] File not find: {file_path}")
                continue
            
            pcd = o3d.io.read_point_cloud(file_path)
            print(f"[INFO] load point cloud '{file_path}'")
            self.point_clouds.append(pcd)
        
        return self.point_clouds
    
    # == filter function
    def remove_plane_background(self, o3d_pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000, plane_type='floor'):

        # 1. RANSAC to segment the plane
        plane_model, inliers = o3d_pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        # set di indici totali
        all_indices = set(range(len(o3d_pcd.points)))
        # Crea un set di punti nel piano
        inlier_indices = set(inliers)
        # Gli outlier sono la differenza: tutti i punti - punti del piano
        outlier_indices = list(all_indices - inlier_indices)
        # Estrai la nuvola di punti 
        pcd_foreground = o3d_pcd.select_by_index(outlier_indices)
        # TODO: Rimozione dei componenti connessi più piccoli (rumore galleggiante)*
            # Se la nuvola di punti risultante è ancora troppo grande e include oggetti indesiderati,
            # si può applicare qui una rimozione dei cluster per tenere solo l'oggetto più grande.
        
        if len(pcd_foreground.points) > 0:
            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
                # Identifica i cluster
                labels = np.array(pcd_foreground.cluster_dbscan(eps=0.05, min_points=10))
                
            if len(labels) > 0:
                # Trova l'etichetta del cluster più grande
                unique_labels, counts = np.unique(labels, return_counts=True)
                if unique_labels.size > 0 and unique_labels[0] != -1: 
                    largest_cluster_label = unique_labels[np.argmax(counts)]
                    
                    # Seleziona solo i punti che appartengono al cluster più grande
                    pcd_foreground = pcd_foreground.select_by_index(
                        np.where(labels == largest_cluster_label)[0]
                    )

        return pcd_foreground
        
    def clean_and_smooth_point_cloud(self, pcd):
        # Convert list of dictionaries to Open3D PointCloud if needed
        if isinstance(pcd, list):
            if not pcd:
                print("[WARN] Empty point cloud list.")
                return []
            
            # Extract xyz coordinates from dictionaries
            points = np.array([[p['x'], p['y'], p['z']] for p in pcd])
            
            # Create Open3D PointCloud
            o3d_pcd = o3d.geometry.PointCloud()
            o3d_pcd.points = o3d.utility.Vector3dVector(points)
        elif isinstance(pcd, o3d.geometry.PointCloud):
            o3d_pcd = pcd
        else:
            raise TypeError(f"Unsupported point cloud type: {type(pcd)}")
        #remove back ground
        o3d_pcd = self.remove_plane_background(o3d_pcd, distance_threshold=0.03) 
        # Remove outliers
        pcd_clean, _ = o3d_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        pcd_uniform = pcd_clean.voxel_down_sample(voxel_size=self.voxel_size)
        
        # Stima delle normali
        pcd_uniform.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 5, max_nn=30
            )
        )
        # Orienta le normali in modo consistente
        pcd_uniform.orient_normals_consistent_tangent_plane(30)
        
        # Convert back to list format if input was a list
        if isinstance(pcd, list):
            filtered_points = np.asarray(pcd_uniform.points)
            return [{'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])} 
                    for p in filtered_points]
        
        return pcd_uniform
    
    # == Visualizer function
    def visualize_point_clouds(self, point_clouds, colors=None, window_name="Point Clouds", 
                              show_normals=False):
        if not point_clouds:
            print("[WARN] Nessuna point cloud da visualizzare.")
            return
        
        vis_pcds = []
        default_colors = [[1, 0, 0], [0, 0, 1], [0, 1, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
        
        for i, pcd in enumerate(point_clouds):
            vis_pcd = copy.deepcopy(pcd)
            color = colors[i] if colors and i < len(colors) else default_colors[i % len(default_colors)]
            vis_pcd.paint_uniform_color(color)
            vis_pcds.append(vis_pcd)
        
        o3d.visualization.draw_geometries(
            vis_pcds,
            window_name=window_name,
            width=1024,
            height=768,
            point_show_normal=show_normals
        )
    
    def visualize_filtered_objects(self):
        if not self.detected_objects_list:
            print("[WARN] No detected objects to visualize.")
            return
        
        print(f"[INFO] Visualizing {len(self.detected_objects_list)} filtered objects...")
        
        # Convert filtered point clouds to Open3D format
        o3d_point_clouds = []
        
        for obj in self.detected_objects_list:
            if 'pcl_object' not in obj or not obj['pcl_object']:
                continue
            
            points_3d = obj['pcl_object']
            
            # Convert to numpy array
            if isinstance(points_3d, list):
                points = np.array([[p['x'], p['y'], p['z']] for p in points_3d])
            else:
                continue
            
            # Create Open3D PointCloud
            o3d_pcd = o3d.geometry.PointCloud()
            o3d_pcd.points = o3d.utility.Vector3dVector(points)
            o3d_point_clouds.append(o3d_pcd)
            
            print(f"[INFO] Object '{obj['label']}' (ID {obj['id']}): {len(points)} points")
        
        if o3d_point_clouds:
            # Visualize all filtered point clouds together
            self.visualize_point_clouds(
                o3d_point_clouds,
                window_name="Filtered Point Clouds",
                show_normals=False
            )
        else:
            print("[WARN] No valid point clouds to visualize.")
    
    