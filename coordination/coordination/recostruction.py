from venv import logger
import open3d as o3d
import numpy as np
import copy
from sklearn.cluster import DBSCAN
import os
from pathlib import Path
import json
import yaml

class PointCloudProcessor:
       
    def __init__(self, ply_directory: str, ply_save_directory: str, yaml_file=None, robot_poses=None, logger=None):
        
        self.ply_directory = Path(ply_directory)
        self.ply_save_directory = Path(ply_save_directory)
        self.yaml_file = yaml_file
        self.logger = logger if logger is not None else __import__('logging').getLogger(__name__)
        
        if not os.path.exists(ply_directory):
            raise FileNotFoundError(f"Directory does not exist: {ply_directory}")
        if not os.path.exists(ply_save_directory):
            raise FileNotFoundError(f"Directory does not exist: {ply_save_directory}")

        
        
        self.raw_clouds = []
        self.processed_clouds = []
        self.merged_cloud = None
        self.point_t = []
        self.boxxes = []
        self.big_box = None
        self.clusters_boxxes = []
        
        
        if robot_poses is None:
            # solo per le prove
            self.robot_pose = [
                {'position': np.array([0., 16., 0.05]), 'yaw': 1.5708, 'name': 'shelfino1'},
                {'position': np.array([0., 24.0, 0.05]), 'yaw': -1.5708, 'name': 'pollo'},
                # {'position': np.array([4., 20.0, 0.05]), 'yaw': 3.14, 'name': 'mario'},
            ]  
        else:
            self.robot_pose = robot_poses
        
        print(f"PointCloudProcessor initialized.")
    
        
    def load_ply_files(self):
        cloud_list = []
        ply_files = sorted(Path(self.ply_directory).glob("*.ply"))
        for ply in ply_files:
            pcd = o3d.io.read_point_cloud(str(ply))
            if not pcd.is_empty():
                cloud_list.append(pcd)
        return cloud_list
    
    def clean_and_smooth_point_cloud(self, pcd, voxel_size=0.002):
        pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd_uniform = pcd_clean.voxel_down_sample(voxel_size=voxel_size)
        pcd_uniform.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30))
        pcd_uniform.orient_normals_consistent_tangent_plane(30)
        return pcd_uniform
        
    def remove_floor_points_ransac(self, pcd, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
        points = np.asarray(pcd.points)
        
        if len(points) == 0:
            return pcd
        
        # Use RANSAC to detect the dominant plane
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        [a, b, c, d] = plane_model
        print(f"Detected plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
        
        # Check if it's a horizontal plane (normal should be mostly vertical)
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)
        
        # Angle between plane normal and vertical axis (z-axis)
        vertical = np.array([0, 0, 1])
        angle = np.arccos(np.abs(np.dot(normal, vertical)))
        angle_deg = np.degrees(angle)
        
        print(f"Plane normal: {normal}")
        print(f"Angle from vertical: {angle_deg:.1f}°")
        
        # Only remove if it's approximately horizontal (angle < 15 degrees)
        if angle_deg < 15:
            # Remove the inliers (floor points)
            pcd_no_floor = pcd.select_by_index(inliers, invert=True)
            
            removed_count = len(inliers)
            kept_count = len(pcd_no_floor.points)
            
            print(f"Removed {removed_count} floor points (kept {kept_count} points)")
            return pcd_no_floor
        else:
            print(f"Detected plane is not horizontal (angle {angle_deg:.1f}°), keeping all points")
            return pcd

    def remove_floor_points_histogram(self, pcd, bin_size=0.01, floor_percentile=10, z_margin=0.02):
        points = np.asarray(pcd.points)
        
        if len(points) == 0:
            return pcd
        
        z_values = points[:, 2]
        
        # Create histogram of Z values
        hist, bin_edges = np.histogram(z_values, bins=int((z_values.max() - z_values.min()) / bin_size))
        
        # Find the Z level with most points (likely the floor)
        max_bin_idx = np.argmax(hist)
        floor_level = (bin_edges[max_bin_idx] + bin_edges[max_bin_idx + 1]) / 2
        
        print(f"Detected floor level at Z = {floor_level:.4f} (bin with {hist[max_bin_idx]} points)")
        
        # Keep points above floor + margin
        threshold = floor_level + z_margin
        mask = points[:, 2] > threshold
        filtered_points = points[mask]
        
        # Create new point cloud
        pcd_no_floor = o3d.geometry.PointCloud()
        pcd_no_floor.points = o3d.utility.Vector3dVector(filtered_points)
        
        # Copy colors and normals if available
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            pcd_no_floor.colors = o3d.utility.Vector3dVector(colors[mask])
        
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            pcd_no_floor.normals = o3d.utility.Vector3dVector(normals[mask])
        
        removed_count = len(points) - len(filtered_points)
        print(f"Removed {removed_count} floor points (kept {len(filtered_points)} points)")
        
        return pcd_no_floor

    def remove_floor_points_adaptive(self, pcd, num_bins=50, prominence_factor=2.0, z_margin=0.02):
        from scipy.signal import find_peaks
        
        points = np.asarray(pcd.points)
        
        if len(points) == 0:
            return pcd
        
        z_values = points[:, 2]
        
        # Create histogram
        hist, bin_edges = np.histogram(z_values, bins=num_bins)
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
        
        # Find peaks in histogram
        peaks, properties = find_peaks(hist, prominence=hist.max() / prominence_factor)
        
        if len(peaks) == 0:
            print("No prominent peaks found, using minimum Z value")
            floor_level = z_values.min()
        else:
            # The floor is likely the lowest prominent peak
            floor_peak_idx = peaks[0]
            floor_level = bin_centers[floor_peak_idx]
            print(f"Found {len(peaks)} prominent levels, floor at Z = {floor_level:.4f}")
        
        # Keep points above floor + margin
        threshold = floor_level + z_margin
        mask = points[:, 2] > threshold
        filtered_points = points[mask]
        
        # Create new point cloud
        pcd_no_floor = o3d.geometry.PointCloud()
        pcd_no_floor.points = o3d.utility.Vector3dVector(filtered_points)
        
        # Copy colors and normals if available
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            pcd_no_floor.colors = o3d.utility.Vector3dVector(colors[mask])
        
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            pcd_no_floor.normals = o3d.utility.Vector3dVector(normals[mask])
        
        removed_count = len(points) - len(filtered_points)
        print(f"Removed {removed_count} floor points (kept {len(filtered_points)} points)")
        
        return pcd_no_floor
    
    def remove_floor_points(self, pcd, method='histogram', **kwargs):

        if method == 'ransac':
            return self.remove_floor_points_ransac(pcd, **kwargs)
        elif method == 'histogram':
            return self.remove_floor_points_histogram(pcd, **kwargs)
        elif method == 'adaptive':
            return self.remove_floor_points_adaptive(pcd, **kwargs)
        else:
            raise ValueError(f"Unknown method: {method}")
    
    def process_all_pointclouds(self):
        processed = []
        for pcd in self.raw_clouds:
            pcd_proc = self.clean_and_smooth_point_cloud(pcd)
            pcd_proc = self.remove_floor_points(pcd_proc)
            print(f"Processed points: {len(pcd_proc.points)}")
            processed.append(pcd_proc)
        return processed
    
    def guassian_distribution_point_clouds(self, processed, variance=1.5):
        
        for pcd in processed:
            points = np.asarray(pcd.points)
            centered = points - np.mean(points, axis=0)
            centroid = np.mean(points, axis=0)

            centroid_pcd = o3d.geometry.PointCloud()  
            cov = np.cov(centered.T)
            eigvals, eigvecs = np.linalg.eigh(cov)

            order = np.argsort(eigvals)[::-1]
            eigvals = eigvals[order]
            eigvecs = eigvecs[:, order]

            points_pca = centered @ eigvecs
            dist_elliptic = np.sqrt(
                (points_pca[:,0]/np.sqrt(eigvals[0]))**2 +
                (points_pca[:,1]/np.sqrt(eigvals[1]))**2 +
                (points_pca[:,2]/np.sqrt(eigvals[2]))**2
            )

            observations = np.exp(-(dist_elliptic**2) / (2 * variance**2))

            obs_norm = (observations - observations.min()) / (observations.max() - observations.min())
           
            colors = np.zeros((len(obs_norm), 3))
            for i, val in enumerate(obs_norm):
                if val > 0.5:
                    colors[i] = [2*(1.0-val), 1.0, 0.0]
                else:
                    colors[i] = [1.0, 2*val, 0.0]
            pcd.colors = o3d.utility.Vector3dVector(colors)

            for point, obs, col in zip(points, observations, colors):
                self.point_t.append([point, obs, col.tolist(), centroid])
        
        print(f"Created {len(self.point_t)} point observations with Gaussian distribution")
        return self.point_t
    
    def create_boxxes(self, grid_divisions=(10, 10, 10)):
        if len(self.point_t) == 0:
            print("No point_t data available. Run guassian_distribution_point_clouds() first.")
            return
        
        # Extract all points
        points = np.array([p[0] for p in self.point_t], dtype=float)
        
        # Create main bounding box
        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)
        self.big_box = {
            'min_bound': min_bound,
            'max_bound': max_bound,
            'center': (min_bound + max_bound) / 2,
            'extent': max_bound - min_bound
        }
        
        print(f"Main bounding box created: min={min_bound}, max={max_bound}")
        
        # Calculate cell dimensions
        nx, ny, nz = grid_divisions
        cell_size = self.big_box['extent'] / np.array([nx, ny, nz])
        
        # Create subcells
        self.boxxes = []
        cell_id = 0
        
        for i in range(nx):
            for j in range(ny):
                for k in range(nz):
                    # Calculate cell bounds
                    cell_min = min_bound + cell_size * np.array([i, j, k])
                    cell_max = cell_min + cell_size
                    cell_center = (cell_min + cell_max) / 2
                    
                    # Find points in this cell
                    mask = np.all((points >= cell_min) & (points < cell_max), axis=1)
                    points_in_cell = [self.point_t[idx] for idx in np.where(mask)[0]]
                    
                    # Create cell data
                    cell_data = {
                        'id': cell_id,
                        'pose_box': {
                            'min_bound': cell_min,
                            'max_bound': cell_max
                        },
                        'centroid': cell_center,
                        'point_t_cell': points_in_cell if len(points_in_cell) > 0 else None,
                        'avg_observation': np.mean([p[1] for p in points_in_cell]) if len(points_in_cell) > 0 else None
                    }
                    
                    self.boxxes.append(cell_data)
                    cell_id += 1
        
        non_empty_cells = sum(1 for box in self.boxxes if box['point_t_cell'] is not None)
        print(f"Created {len(self.boxxes)} cells ({non_empty_cells} non-empty) with grid divisions {grid_divisions}")
        return self.boxxes
    
    def create_clusters_boxxes(self, observation_threshold=0.2, eps=0.1, min_samples=3, min_cluster_size=15):
        """
        Create clusters of boxes with low observation values.
        Groups nearby boxes with avg_observation below threshold into regions.
        """
        if len(self.boxxes) == 0:
            print("No boxes available. Run create_boxxes() first.")
            return
        
        # Filter boxes with low observation
        low_obs_boxes = []
        low_obs_indices = []
        
        for idx, box in enumerate(self.boxxes):
            if box['avg_observation'] is not None and box['avg_observation'] < observation_threshold:
                low_obs_boxes.append(box)
                low_obs_indices.append(idx)
        
        if len(low_obs_boxes) == 0:
            print(f"No boxes found with observation < {observation_threshold}")
            return
        
        print(f"Found {len(low_obs_boxes)} boxes with low observation (< {observation_threshold})")
        
        # Extract centroids for clustering
        centroids = np.array([box['centroid'] for box in low_obs_boxes])
        
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(centroids)
        labels = clustering.labels_
        
        # Group boxes by cluster
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)  # Remove noise label
        
        self.clusters_boxxes = []
        
        for label in unique_labels:
            cluster_mask = labels == label
            cluster_boxes = [low_obs_boxes[i] for i in np.where(cluster_mask)[0]]
            
            # Filter by minimum cluster size
            if len(cluster_boxes) < min_cluster_size:
                continue
            
            # Calculate cluster centroid
            cluster_centroids = np.array([box['centroid'] for box in cluster_boxes])
            cluster_centroid = np.mean(cluster_centroids, axis=0)
            
            # Calculate cluster bounds
            all_mins = np.array([box['pose_box']['min_bound'] for box in cluster_boxes])
            all_maxs = np.array([box['pose_box']['max_bound'] for box in cluster_boxes])
            cluster_min = np.min(all_mins, axis=0)
            cluster_max = np.max(all_maxs, axis=0)
            
            # Calculate average observation for the cluster
            avg_obs = np.mean([box['avg_observation'] for box in cluster_boxes])
            
            cluster_data = {
                'cluster_id': label,
                'boxes': cluster_boxes,
                'centroid': cluster_centroid,
                'min_bound': cluster_min,
                'max_bound': cluster_max,
                'num_boxes': len(cluster_boxes),
                'avg_observation': avg_obs
            }
            
            self.clusters_boxxes.append(cluster_data)
        
        print(f"Created {len(self.clusters_boxxes)} clusters from low-observation boxes")
        return self.clusters_boxxes
    
    def avg_obs_all_cells(self):
        observations = [box['avg_observation'] for box in self.boxxes if box['avg_observation'] is not None]
        if len(observations) == 0:
            print("No observations available to calculate average.")
            return None
        mean_observation = np.mean(observations)
        print(f"Average observation across all cells: {mean_observation}")
        return mean_observation
    
    def save_boxxes_json(self, filename="boxxes_object.json"):
        if len(self.boxxes) == 0:
            print("ATTENTION NO BOXES TO SAVE")
            return
        
        data_boxxes = copy.deepcopy(self.boxxes)
        
        for cell_data in data_boxxes:
            # (np.ndarray -> list)
            cell_data['pose_box']['min_bound'] = cell_data['pose_box']['min_bound'].tolist()
            cell_data['pose_box']['max_bound'] = cell_data['pose_box']['max_bound'].tolist()
            cell_data['centroid'] = cell_data['centroid'].tolist()
            
            # (np.float64 -> float)
            if isinstance(cell_data['avg_observation'], np.floating):
                cell_data['avg_observation'] = float(cell_data['avg_observation'])

            # (npy list of lists)
            if cell_data['point_t_cell'] is not None:
                # p is a list [point, obs, col]
                converted_points = []
                for p in cell_data['point_t_cell']:
                    converted_point = [
                        p[0].tolist() if isinstance(p[0], np.ndarray) else p[0],  # point
                        float(p[1]) if isinstance(p[1], np.floating) else p[1],    # obs
                        p[2].tolist() if isinstance(p[2], np.ndarray) else p[2]    # col
                    ]
                    converted_points.append(converted_point)
                cell_data['point_t_cell'] = converted_points
        filepath = self.ply_save_directory / filename
        try:
            with open(filepath, 'w') as f:  
                json.dump(data_boxxes, f, indent=4)
            print(f"data saved to: {filepath}")
        except Exception as e:
            print(f"Error saving JSON file: {e}")

    def save_point_cloud(self):
        if len(self.point_t) == 0:
            print("no point to save")
            return

        points = np.array([p[0] for p in self.point_t], dtype=float)
        colors = np.array([p[2] for p in self.point_t], dtype=float)
    
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        o3d.io.write_point_cloud(self.ply_save_directory / "complete_cloud.ply", pcd)
        print(f"Point cloud saved to {self.ply_save_directory / 'complete_cloud.ply'}")

    def join_old_and_actual_values_boxxes(self):
        json_filepath = self.ply_save_directory / "boxxes_object.json"
        
        # Check if previous data exists
        if not json_filepath.exists():
            print(f"No JSON file found at {json_filepath}. Skipping merge.")
            return
        
        try:
            with open(json_filepath, 'r') as f:
                old_data = json.load(f)
            
            print(f"Loaded {len(old_data)} previous boxes from JSON")
            
            # Extract all old point_t data
            old_point_t = []
            for cell_data in old_data:
                if cell_data['point_t_cell'] is not None:
                    # Convert back from JSON format to internal format
                    for p in cell_data['point_t_cell']:
                        point = np.array(p[0], dtype=float)
                        obs = float(p[1])                  
                        col = p[2]
                        old_point_t.append([point, obs, col])
            
            print(f"Extracted {len(old_point_t)} old points")
            print(f"Current points: {len(self.point_t)}")
            
            # Merge old and new point_t
            # Create a set of current point
            current_points_set = set()
            for p in self.point_t:
                point_tuple = tuple(p[0].tolist() if isinstance(p[0], np.ndarray) else p[0])
                current_points_set.add(point_tuple)
            
            # Add old points that don't overlap with current ones
            added_count = 0
            for old_p in old_point_t:
                point_tuple = tuple(old_p[0].tolist())
                if point_tuple not in current_points_set:
                    self.point_t.append(old_p)
                    current_points_set.add(point_tuple)
                    added_count += 1
            
            print(f"Added {added_count} unique old points to current point_t")
            print(f"Total points after merge: {len(self.point_t)}")
            
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
        except Exception as e:
            print(f"Error loading or merging data: {e}")
    
    def eliminate_ply_files(self):
        ply_files = sorted(Path(self.ply_directory).glob("*.ply"))
        for ply in ply_files:
            try:
                os.remove(ply)
                print(f"Deleted file: {ply}")
            except Exception as e:
                print(f"Error deleting file {ply}: {e}")
    
    def load_type_obj_yaml(self):
        if self.yaml_file is None:
            print("YAML file path is not set.")
            return {}
        yaml_path = Path(self.yaml_file)
        if not yaml_path.exists():
            print(f"YAML file not found at {yaml_path}")
            return {}
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            print(f"Loaded object radii from {yaml_path}: {data}")
            return data
        except Exception as e:
            print(f"Error loading YAML file: {e}")
            return {}
    
    
    def detect_object_type_from_ply(self):
        ply_files = sorted(self.ply_directory.glob("*.ply"))
        
        if not ply_files:
            print("No PLY files found to detect object type")
            return None
        
        first_file = ply_files[0].stem  # Get filename without extension
        
        if "_filtered" in first_file:
            object_type = first_file.split("_filtered")[0]
        else:
            # Fallback: take everything before last underscore
            parts = first_file.split('_')
            object_type = '_'.join(parts[:-1]) if len(parts) > 1 else first_file
        
        print(f"Detected object type: '{object_type}' from file: {ply_files[0].name}")
        return object_type.lower()
    
    def centorids_on_circle(self, objs_names_yaml, object_type_ply):
        center_circle = self.big_box['center']      
        radius_circle = objs_names_yaml[object_type_ply]['radius']
        new_centroids = []
        print(f"Using radius {radius_circle} for object type '{object_type_ply}'")
        # Set z coordinate to 0 (ignore height)
        center_2d = center_circle.copy()
        center_2d[2] = 0.0
        if len(self.clusters_boxxes) == 0:
            print("No clusters available to distribute on circle")
            return
        
        # Calculate angle step for evenly distributed clusters
        num_clusters = len(self.clusters_boxxes)
        angle_step = 2 * np.pi / num_clusters
        
        print(f"Distributing {num_clusters} cluster centroids on circle:")
        print(f"  Center: {center_2d}")
        print(f"  Radius: {radius_circle}")
        
        # Project each cluster centroid onto the circle
        for cluster in self.clusters_boxxes:
            old_centroid = cluster['centroid'].copy()
            
            # Get 2D position (ignore z)
            old_2d = old_centroid.copy()
            old_2d[2] = 0.0
            
            # Calculate vector from circle center to old centroid
            direction = old_2d - center_2d
            
            # Calculate angle of this direction
            angle = np.arctan2(direction[1], direction[0])
            
            # Project onto circle at this angle
            x = center_2d[0] + radius_circle * np.cos(angle)
            y = center_2d[1] + radius_circle * np.sin(angle)
            z = 0.0
            
            new_centroid = np.array([x, y, z])
            new_centroids.append(new_centroid)
            
            # Calculate distance moved
            distance_moved = np.linalg.norm(new_centroid - old_2d)
            
            print(f"  Cluster {cluster['cluster_id']}: {old_centroid} -> {new_centroid}")
            print(f"    Angle: {np.degrees(angle):.1f}°, Distance moved: {distance_moved:.3f}")
        
        print("Cluster centroids projected onto circle at their radial positions")
        return new_centroids, center_2d, radius_circle
    
    def full_pipeline(self, robot_poses=None):
        # Load and process
        self.raw_clouds = self.load_ply_files()
        print(f"Loaded {len(self.raw_clouds)} point clouds.")
        self.processed_clouds = self.process_all_pointclouds()
        self.guassian_distribution_point_clouds(self.processed_clouds)
        # self.visualize_point_t()
        # create clusters
        self.join_old_and_actual_values_boxxes()
        self.create_boxxes(grid_divisions=(20,20,20))
        self.visualize_box_mesh()
        self.visualize_boxxes()
        
        self.create_clusters_boxxes()
        self.visualize_cluster_boxxes()
        
        objs_names_yaml = self.load_type_obj_yaml()
        object_type_ply = self.detect_object_type_from_ply()
                
        self.logger.info(f"Loaded YAML object names: {objs_names_yaml}")
        self.logger.info(f"Detected object type from PLY: {object_type_ply}")
        
        new_centroids, center_2d, radius_circle = self.centorids_on_circle(objs_names_yaml, object_type_ply)
        self.visualize_cluster_boxxes_with_new_centroids(new_centroids, center_2d, radius_circle)
        # compute average observation
        mean_observation = self.avg_obs_all_cells()
        # save point and boxes
        self.save_point_cloud()
        self.save_boxxes_json()
        self.eliminate_ply_files()
        return new_centroids, self.big_box['center'], mean_observation      
        
      
    def visualize_point_t(self):
        if len(self.point_t) == 0:
            print("No point_t data available. Run guassian_distribution_point_clouds() first.")
            return
        points = np.array([p[0] for p in self.point_t], dtype=float)
        colors = np.array([p[2] for p in self.point_t], dtype=float)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        geometries = [pcd]
        
        # Add centroids as red spheres
        centroids = np.array([p[3] for p in self.point_t], dtype=float)
        unique_centroids = np.unique(centroids, axis=0)
        
        for centroid in unique_centroids:
            centroid_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
            centroid_sphere.translate(centroid)
            centroid_sphere.paint_uniform_color([1, 0, 0])  # Red color
            centroid_sphere.compute_vertex_normals()
            geometries.append(centroid_sphere)

        print(f"Visualizing {len(points)} points from point_t with {len(unique_centroids)} centroids...")
        o3d.visualization.draw_geometries(geometries)
        
    def visualize_boxxes(self):
        if len(self.boxxes) == 0:
            print("No boxes available. Run create_boxxes() first.")
            return
        
        geometries = []
        
        # Create line set for each box
        for box in self.boxxes:
            min_b = box['pose_box']['min_bound']
            max_b = box['pose_box']['max_bound']
            
            # Create axis-aligned bounding box
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_b, max_bound=max_b)
            
            # Green -> Yellow -> Red gradient based on observation value
            val = box['avg_observation']
            if val is None:
                continue
            
            elif val > 0.5:
                # Green -> Yellow
                color = [2*(1.0-val), 1.0, 0.0]
            else:
                # Yellow -> Red
                color = [1.0, 2*val, 0.0]
        
            bbox.color = color
            geometries.append(bbox)
        
        # Add point cloud if available
        if len(self.point_t) > 0:
            points = np.array([p[0] for p in self.point_t], dtype=float)
            colors = np.array([p[2] for p in self.point_t], dtype=float)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            geometries.append(pcd)
        
        print(f"Visualizing {len(self.boxxes)} boxes...")
        o3d.visualization.draw_geometries(geometries)
        
    def visualize_box_mesh(self, boxxes=None):
        if boxxes is None:
            boxxes = self.boxxes
        if len(boxxes) == 0:
            print("No boxes available. Run create_boxxes() first.")
            return
        
        geometries = []

        for box in self.boxxes:
            min_b = np.array(box['pose_box']['min_bound'])
            max_b = np.array(box['pose_box']['max_bound'])
            size = max_b - min_b

            # Mesh piena
            mesh = o3d.geometry.TriangleMesh.create_box(
                width=size[0],
                height=size[1],
                depth=size[2]
            )
            mesh.translate(min_b)
            mesh.compute_vertex_normals()

            # Colore con gradiente
            val = box['avg_observation']
            if val is None:
                # White for empty boxes
                continue
            elif val > 0.5:
                color = [2*(1.0-val), 1.0, 0.0]
            else:
                color = [1.0, 2*val, 0.0]

            mesh.paint_uniform_color(color)

            geometries.append(mesh)

        # Point cloud
        if len(self.point_t) > 0:
            points = np.array([p[0] for p in self.point_t], dtype=float)
            colors = np.array([p[2] for p in self.point_t], dtype=float)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            geometries.append(pcd)

        print(f"Visualizing {len(geometries)} geometries...")
        o3d.visualization.draw_geometries(geometries)
    
    def visualize_cluster_boxxes(self):
        """
        Visualize clusters with their centroids and the main bounding box.
        """
        if len(self.clusters_boxxes) == 0:
            print("No clusters available. Run create_clusters_boxxes() first.")
            return
        
        geometries = []
        
        # Add main big box (wireframe in blue)
        if self.big_box is not None:
            big_bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=self.big_box['min_bound'],
                max_bound=self.big_box['max_bound']
            )
            big_bbox.color = [0, 0, 1]  # Blue
            geometries.append(big_bbox)
            
            # Add global centroid sphere (white/gray)
            global_centroid_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            global_centroid_sphere.translate(self.big_box['center'])
            
            global_centroid_sphere.paint_uniform_color([1, 1, 1])  # White
            global_centroid_sphere.compute_vertex_normals()
            geometries.append(global_centroid_sphere)
            
            # Add coordinate frame at global centroid
            global_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
            global_frame.translate(self.big_box['center'])
            geometries.append(global_frame)
        
        # Color palette for clusters
        colors = [
            [1, 0, 0],      # Red
            [0, 1, 0],      # Green
            [1, 1, 0],      # Yellow
            [1, 0, 1],      # Magenta
            [0, 1, 1],      # Cyan
            [1, 0.5, 0],    # Orange
            [0.5, 0, 1],    # Purple
            [0, 0.5, 0.5],  # Teal
        ]
        
        for idx, cluster in enumerate(self.clusters_boxxes):
            color = colors[idx % len(colors)]
            
            # Create bounding box for cluster
            cluster_bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=cluster['min_bound'],
                max_bound=cluster['max_bound']
            )
            cluster_bbox.color = color
            geometries.append(cluster_bbox)
            
            # Create sphere at centroid
            centroid_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            centroid_sphere.translate(cluster['centroid'])
            centroid_sphere.paint_uniform_color(color)
            centroid_sphere.compute_vertex_normals()
            geometries.append(centroid_sphere)                  
            
            # Add coordinate frame at centroid
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
            frame.translate(cluster['centroid'])
            geometries.append(frame)
        
        # Add original point cloud if available
        if len(self.point_t) > 0:
            points = np.array([p[0] for p in self.point_t], dtype=float)
            colors_pc = np.array([p[2] for p in self.point_t], dtype=float)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors_pc)
            geometries.append(pcd)
        
        print(f"Visualizing {len(self.clusters_boxxes)} clusters with centroids and main bounding box")
        print(f"Global centroid at: {self.big_box['center']}")
        o3d.visualization.draw_geometries(geometries)   

    def create_robot_frame(self, position, yaw, name, size=0.1):
        robot_body = o3d.geometry.TriangleMesh.create_cylinder(radius=size/2, height=size/3)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size*1.5)
        arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=size/10, cone_radius=size/5,
            cylinder_height=size*0.6, cone_height=size*0.4)
        
        color = [0, 0.5, 1] if "shelfino" in name.lower() else [1, 0.5, 0]
        robot_body.paint_uniform_color(color)
        arrow.paint_uniform_color([1, 0, 0])
        
        R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0], [0,0,1]])
        for geom in [robot_body, frame, arrow]:
            geom.rotate(R, center=(0,0,0))
            geom.translate(position)
        return [robot_body, frame, arrow]
    
    def visualize_cluster_boxxes_with_new_centroids(self, new_centroids, center_2d, radius_circle):
        if len(self.clusters_boxxes) == 0:
            print("No clusters available. Run create_clusters_boxxes() first.")
            return
        
        if new_centroids is None or len(new_centroids) == 0:
            print("No new centroids provided.")
            return
        
        geometries = []
        
        # Add main big box (wireframe in blue)
        if self.big_box is not None:
            big_bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=self.big_box['min_bound'],
                max_bound=self.big_box['max_bound']
            )
            big_bbox.color = [0, 0, 1]  # Blue
            geometries.append(big_bbox)
            
            # Add circle center sphere (white)
            center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
            center_sphere.translate(center_2d)
            center_sphere.paint_uniform_color([1, 1, 1])
            center_sphere.compute_vertex_normals()
            geometries.append(center_sphere)
            
            # Add coordinate frame at circle center
            center_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
            center_frame.translate(center_2d)
            geometries.append(center_frame)
        
        # Create circle visualization
        num_points = 100
        angles = np.linspace(0, 2 * np.pi, num_points)
        circle_points = []
        for angle in angles:
            x = center_2d[0] + radius_circle * np.cos(angle)
            y = center_2d[1] + radius_circle * np.sin(angle)
            z = center_2d[2]
            circle_points.append([x, y, z])
        
        circle_points = np.array(circle_points)
        lines = [[i, (i + 1) % num_points] for i in range(num_points)]
        circle_line_set = o3d.geometry.LineSet()
        circle_line_set.points = o3d.utility.Vector3dVector(circle_points)
        circle_line_set.lines = o3d.utility.Vector2iVector(lines)
        circle_line_set.colors = o3d.utility.Vector3dVector([[0.5, 0.5, 0.5]] * len(lines))
        geometries.append(circle_line_set)
        
        # Color palette for clusters
        colors = [
            [1, 0, 0],      # Red
            [0, 1, 0],      # Green
            [1, 1, 0],      # Yellow
            [1, 0, 1],      # Magenta
            [0, 1, 1],      # Cyan
            [1, 0.5, 0],    # Orange
            [0.5, 0, 1],    # Purple
            [0, 0.5, 0.5],  # Teal
        ]
        
        for idx, cluster in enumerate(self.clusters_boxxes):
            color = colors[idx % len(colors)]
            
            # Original cluster bounding box (semi-transparent)
            cluster_bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=cluster['min_bound'],
                max_bound=cluster['max_bound']
            )
            cluster_bbox.color = color
            geometries.append(cluster_bbox)
            
            # Original centroid (small sphere)
            old_centroid = cluster['centroid']
            old_centroid_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            old_centroid_sphere.translate(old_centroid)
            old_centroid_sphere.paint_uniform_color(color)
            old_centroid_sphere.compute_vertex_normals()
            geometries.append(old_centroid_sphere)
            
            # New centroid on circle (larger sphere)
            if idx < len(new_centroids):
                new_centroid = new_centroids[idx]
                new_centroid_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
                new_centroid_sphere.translate(new_centroid)
                new_centroid_sphere.paint_uniform_color(color)
                new_centroid_sphere.compute_vertex_normals()
                geometries.append(new_centroid_sphere)
                
                # Add coordinate frame at new centroid
                new_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
                new_frame.translate(new_centroid)
                geometries.append(new_frame)
                
                # Line connecting old and new centroids
                line_points = np.array([old_centroid, new_centroid])
                line = o3d.geometry.LineSet()
                line.points = o3d.utility.Vector3dVector(line_points)
                line.lines = o3d.utility.Vector2iVector([[0, 1]])
                line.colors = o3d.utility.Vector3dVector([color])
                geometries.append(line)
        
        # Add original point cloud if available
        if len(self.point_t) > 0:
            points = np.array([p[0] for p in self.point_t], dtype=float)
            colors_pc = np.array([p[2] for p in self.point_t], dtype=float)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors_pc)
            geometries.append(pcd)
        
        print(f"Visualizing {len(self.clusters_boxxes)} clusters with repositioned centroids")
        print(f"Circle center: {center_2d}, radius: {radius_circle}")
        o3d.visualization.draw_geometries(geometries)
    
def main():
    ply_directory = "/ros2_ws/src/working_directory/point_cloud/filtered_ply"
    # where to save processed .ply files --> in mesh folder save also the .ply and the mesh files
    ply_save_directory = "/ros2_ws/src/working_directory/mesh"
    processor = PointCloudProcessor(ply_directory=ply_directory, ply_save_directory=ply_save_directory)

    processor.full_pipeline()
    # processor.visualize_point_cloud()

    print("FINISH")
    
if __name__ == "__main__":
    main()