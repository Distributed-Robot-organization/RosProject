import open3d as o3d
import numpy as np
import copy
from sklearn.cluster import DBSCAN
import os
from pathlib import Path

class PointCloudProcessor:
       
    # Bayesian constants
    P_PRIOR = 0.5
    L_PRIOR = 0.0
    L_OCC = 2.2
    L_FREE = -2.2
    L_MIN, L_MAX = -6.0, 6.0

    # Consensus parameters
    CONSENSUS_ITERS = 3
    CONSENSUS_ALPHA = 0.5

    def __init__(self, ply_directory: str, ply_save_directory: str):
        
        self.ply_directory = Path(ply_directory)
        self.ply_save_directory = Path(ply_save_directory)

        if not os.path.exists(ply_directory):
            raise FileNotFoundError(f"Directory does not exist: {ply_directory}")

        print(f"PointCloudProcessor initialized. Loading PLY from: {ply_directory}")
        
        self.raw_clouds = []
        self.processed_clouds = []
        self.merged_cloud = None
        self.voxel_grid = None
        self.bbox = None
        self.box_info = []
        self.clusters = []
        self.global_centroid = None
        
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
    
    def process_all_pointclouds(self):
        processed = []
        for pcd in self.raw_clouds:
            pcd_proc = self.clean_and_smooth_point_cloud(pcd)
            print(f"Processed points: {len(pcd_proc.points)}")
            processed.append(pcd_proc)
        return processed
    
    def merge_pointclouds(self, pcl_list, voxel_size=0.002):
        merged = o3d.geometry.PointCloud()
        for p in pcl_list:
            merged += p
        merged = merged.voxel_down_sample(voxel_size=voxel_size)
        merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        merged.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        merged.orient_normals_consistent_tangent_plane(30)
        return merged
    
    def create_voxel_grid(self, voxel_size=0.005):
        bbox = self.merged_cloud.get_axis_aligned_bounding_box()
        min_bound = np.array(bbox.min_bound) - voxel_size * 2
        max_bound = np.array(bbox.max_bound) + voxel_size * 2
        self.voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(
            self.merged_cloud, voxel_size=voxel_size, min_bound=min_bound, max_bound=max_bound)
        self.bbox = bbox
        grid_size = np.ceil((max_bound - min_bound) / voxel_size).astype(int)
        print(f"Voxel grid: {grid_size[0]}x{grid_size[1]}x{grid_size[2]}, occupied: {len(self.voxel_grid.get_voxels())}")
        # show to ply with voxels
        #o3d.visualization.draw_geometries([self.voxel_grid], window_name="Voxel Grid", width=1280, height=720)
    
    def create_uniform_grid_boxes(self, box_size=0.1):
        min_bound, max_bound = np.array(self.bbox.min_bound), np.array(self.bbox.max_bound)
        grid_dims = np.ceil((max_bound - min_bound) / box_size).astype(int)
        print(f"Box grid: {grid_dims[0]}x{grid_dims[1]}x{grid_dims[2]}, total: {np.prod(grid_dims)}")
        
        self.box_info = []
        for i in range(grid_dims[0]):
            for j in range(grid_dims[1]):
                for k in range(grid_dims[2]):
                    box_min = min_bound + np.array([i, j, k]) * box_size
                    box_max = box_min + box_size
                    bbox = o3d.geometry.AxisAlignedBoundingBox(box_min, box_max)
                    bbox.color = (0.7, 0.7, 0.7)
                    self.box_info.append({
                        'index': (i, j, k), 'min': box_min, 'max': box_max,
                        'center': (box_min + box_max) / 2, 'bbox': bbox,
                        'has_points': False, 'points_count': 0, 'hit_by_ray': False,
                        'log_odds': self.L_PRIOR, 'observations': 0
                    })
        return grid_dims, box_size
    
    def check_boxes_with_points(self, boxes):
        points = np.asarray(self.merged_cloud.points)
        for box in boxes:
            mask = np.all((points >= box['min']) & (points <= box['max']), axis=1)
            box['points_count'] = int(np.sum(mask))
            box['has_points'] = box['points_count'] > 0
    
    def ray_box_intersection(self, ray_origin, ray_direction, box_min, box_max, max_distance):
        tmin, tmax = -np.inf, np.inf
        for i in range(3):
            if abs(ray_direction[i]) < 1e-8:
                if ray_origin[i] < box_min[i] or ray_origin[i] > box_max[i]:
                    return False
            else:
                t1, t2 = (box_min[i] - ray_origin[i]) / ray_direction[i], (box_max[i] - ray_origin[i]) / ray_direction[i]
                if t1 > t2: t1, t2 = t2, t1
                tmin, tmax = max(tmin, t1), min(tmax, t2)
                if tmin > tmax: return False
        return tmin >= 0 and tmin <= max_distance
    
    def raycast_from_robot(self, robot_pos, robot_yaw, boxes, max_distance=30.0, num_h=60, num_v=20, fov_h=np.pi/2, fov_v=np.pi/6):
        Rz = np.array([[np.cos(robot_yaw), -np.sin(robot_yaw), 0],
                       [np.sin(robot_yaw), np.cos(robot_yaw), 0], [0, 0, 1]])
        rays_hit = 0
        for i in range(num_h):
            for j in range(num_v):
                angle_h = -fov_h/2 + (i/(num_h-1))*fov_h
                angle_v = -fov_v/2 + (j/(num_v-1))*fov_v
                dloc = np.array([np.cos(angle_v)*np.cos(angle_h), np.cos(angle_v)*np.sin(angle_h), np.sin(angle_v)])
                direction = Rz @ dloc
                for box in boxes:
                    if self.ray_box_intersection(robot_pos, direction, box['min'], box['max'], max_distance):
                        box['hit_by_ray'] = True
                        rays_hit += 1
        return rays_hit
    
    def bayes_update_boxes(self, boxes):
        for box in boxes:
            if box['hit_by_ray']:
                L_incr = self.L_OCC if box['has_points'] else self.L_FREE
                box['log_odds'] = np.clip(box['log_odds'] + L_incr, self.L_MIN, self.L_MAX)
                box['observations'] += 1
    
    def consensus_step(self, robots_boxes):
        L_lists = {b['index']: [] for b in robots_boxes[0]}
        for r in robots_boxes:
            for b in r:
                L_lists[b['index']].append(b['log_odds'])
        for r in robots_boxes:
            for b in r:
                L_avg = np.mean(L_lists[b['index']])
                b['log_odds'] = np.clip((1-self.CONSENSUS_ALPHA)*b['log_odds'] + self.CONSENSUS_ALPHA*L_avg, self.L_MIN, self.L_MAX)
    
    def color_boxes_by_probability(self, boxes, occ_thresh=0.6, free_thresh=0.4):
        empty_boxes, occupied_boxes, unknown_boxes = [], [], []
        for box in boxes:
            p = 1.0 / (1.0 + np.exp(-box['log_odds']))
            if p >= occ_thresh:
                box['bbox'].color = (0, 1, 0)
                occupied_boxes.append(box)
            elif p <= free_thresh:
                box['bbox'].color = (1, 0, 0)
                empty_boxes.append(box)
            else:
                box['bbox'].color = (0.3, 0.3, 0.3)
                unknown_boxes.append(box)
        return empty_boxes, occupied_boxes, unknown_boxes
    
    def color_boxes_by_density(self, occupied_boxes, threshold_percentile=30):
        if not occupied_boxes: return [], []
        points_counts = [box['points_count'] for box in occupied_boxes]
        threshold = np.percentile(points_counts, threshold_percentile)
        min_points, max_points = min(points_counts), max(points_counts)
        print(f"\nDensity: min={min_points}, max={max_points}, threshold={threshold:.1f}")
        
        low_density, high_density = [], []
        for box in occupied_boxes:
            points = box['points_count']
            density_ratio = (points - min_points) / (max_points - min_points) if max_points > min_points else 0
            if points <= threshold:
                box['bbox'].color = (1.0, 0.5 + 0.5 * density_ratio, 0.0)
                box['density_level'] = 'low'
                low_density.append(box)
            else:
                box['bbox'].color = (0.0, 0.5 + 0.5 * density_ratio, 0.0)
                box['density_level'] = 'high'
                high_density.append(box)
        print(f"Low density: {len(low_density)}, High density: {len(high_density)}")
        return low_density, high_density
    
    def cluster_low_density_boxes(self, low_density_boxes, box_size):
        if not low_density_boxes: return []
        print("\n=== Clustering ===")
        centers = np.array([box['center'] for box in low_density_boxes])
        labels = DBSCAN(eps=box_size * 1.8, min_samples=2).fit(centers).labels_
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        print(f"Clusters: {n_clusters}, Noise: {list(labels).count(-1)}")
        
        clusters = []
        for cid in range(n_clusters):
            cboxes = [low_density_boxes[i] for i in range(len(low_density_boxes)) if labels[i] == cid]
            ccenters = np.array([b['center'] for b in cboxes])
            all_mins, all_maxs = np.array([b['min'] for b in cboxes]), np.array([b['max'] for b in cboxes])
            clusters.append({
                'id': cid, 'boxes': cboxes, 'centroid': np.mean(ccenters, axis=0),
                'min_bound': np.min(all_mins, axis=0), 'max_bound': np.max(all_maxs, axis=0),
                'size': len(cboxes), 'volume': np.prod(np.max(all_maxs, axis=0) - np.min(all_mins, axis=0))
            })
            print(f"Cluster {cid}: size={len(cboxes)}, volume={clusters[-1]['volume']:.6f}")
        return clusters
    
    def create_cluster_visualization(self, box_size):
        geometries = []
        colors = [[1,0,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1],[1,0.5,0],[0.5,0,1],[0,0.5,0]]
        
        # Ottieni dimensioni cluster per colore graduato
        if self.clusters:
            sizes = [c['size'] for c in self.clusters]
            min_size, max_size = min(sizes), max(sizes)
            
            for i, c in enumerate(self.clusters):
                # Colore base dal palette
                base_color = colors[i % len(colors)]
                
                # Intensità basata sulla dimensione del cluster
                size_ratio = (c['size'] - min_size) / (max_size - min_size) if max_size > min_size else 0.5
                # Più grande il cluster, più intenso il colore
                color = [ch * (0.4 + 0.6 * size_ratio) for ch in base_color]
                
                # Colora le box del cluster
                for box in c['boxes']: 
                    box['bbox'].color = color
                
                # Sfera al centroide
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=box_size*0.5)
                sphere.paint_uniform_color(color)
                sphere.translate(c['centroid'])
                geometries.append(sphere)
                
                # Bounding box del cluster
                cbbox = o3d.geometry.AxisAlignedBoundingBox(c['min_bound'], c['max_bound'])
                cbbox.color = color
                geometries.append(cbbox)
                
                # Frame di riferimento al centroide
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=box_size*2)
                frame.translate(c['centroid'])
                geometries.append(frame)
        
        return geometries
    
    def create_infill_boxes(self, box_size):
        infill = []
        print("\n=== Creating infill ===")
        for c in self.clusters:
            dims = np.ceil((c['max_bound'] - c['min_bound']) / box_size).astype(int)
            print(f"Cluster {c['id']}: {dims[0]}x{dims[1]}x{dims[2]} infill boxes")
            for i in range(dims[0]):
                for j in range(dims[1]):
                    for k in range(dims[2]):
                        bmin = c['min_bound'] + np.array([i,j,k])*box_size
                        bbox = o3d.geometry.AxisAlignedBoundingBox(bmin, bmin + box_size)
                        bbox.color = (1,1,0)
                        infill.append({'cluster_id': c['id'], 'min': bmin, 'max': bmin + box_size,
                                     'center': (bmin + bmin + box_size)/2, 'bbox': bbox})
        return infill
    
    def calculate_global_centroid(self, box_size):
        points = np.asarray(self.merged_cloud.points)
        self.global_centroid = np.mean(points, axis=0)
        print(f"\nGlobal centroid: [{self.global_centroid[0]:.3f}, {self.global_centroid[1]:.3f}, {self.global_centroid[2]:.3f}]")
        
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=box_size * 0.8)
        sphere.paint_uniform_color([1, 0, 1])
        sphere.translate(self.global_centroid)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=box_size * 3)
        frame.translate(self.global_centroid)
        
        print("\n=== Cluster distances ===")
        for cluster in self.clusters:
            distance = np.linalg.norm(self.global_centroid - cluster['centroid'])
            print(f"Cluster {cluster['id']}: distance={distance:.3f}m")
        
        return [sphere, frame]
    
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
    
    def multi_robot_bayesian_mapping(self, robot_configs, T=3, box_size=0.1):
        print("\n=== Multi-robot Bayesian mapping ===")
        grid_dims, _ = self.create_uniform_grid_boxes(box_size)
        
        # Initialize robot boxes
        robots_boxes = []
        for _ in robot_configs:
            robot_boxes = [copy.deepcopy({**b, 'log_odds': self.L_PRIOR, 'observations': 0, 
                                         'hit_by_ray': False, 'has_points': False, 'points_count': 0}) 
                          for b in self.box_info]
            robots_boxes.append(robot_boxes)
        
        # Simulate T timesteps --> to be adapted to real robot poses and raycasting
        for t in range(T):
            print(f"\n=== Timestep {t+1}/{T} ===")
            for robot_boxes in robots_boxes:
                for b in robot_boxes:
                    b['hit_by_ray'] = False
                    b['has_points'] = False
                    b['points_count'] = 0
            
            for i, (robot_boxes, config) in enumerate(zip(robots_boxes, robot_configs)):
                self.check_boxes_with_points(robot_boxes)
                hits = self.raycast_from_robot(config['position'], config['yaw'], robot_boxes)
                print(f"Robot {i+1} hits: {hits}")
                self.bayes_update_boxes(robot_boxes)
            
            for _ in range(self.CONSENSUS_ITERS):
                self.consensus_step(robots_boxes)
        
        # Merge results
        merged_boxes = []
        for bb1, bb2 in zip(robots_boxes[0], robots_boxes[1]):
            m = copy.deepcopy(bb1)
            m['log_odds'] = (bb1['log_odds'] + bb2['log_odds']) / 2.0
            merged_boxes.append(m)
        
        return merged_boxes
    
    def visualize_point_cloud(self, cloud=None):
        if cloud is None:
            cloud = self.merged_cloud
        o3d.visualization.draw_geometries([cloud], window_name="Final Cloud", width=1280, height=840)

    def save_point_cloud(self, cloud):
        o3d.io.write_point_cloud(self.ply_save_directory / "Nome_cloud.ply", cloud)
        print(f"Point cloud saved to {self.ply_save_directory / 'Nome_cloud.ply'}")

    def full_pipeline(self):
        # Load and process
        self.raw_clouds = self.load_ply_files()
        print(f"Loaded {len(self.raw_clouds)} point clouds.")
        self.processed_clouds = self.process_all_pointclouds()
        self.merged_cloud = self.merge_pointclouds(self.processed_clouds)
        self.save_point_cloud(self.merged_cloud)
        
        # Voxel grid
        self.create_voxel_grid()
        
        # Robot configs --> ora da sistemare le pose dei robot
        robot_configs = [
            {'position': np.array([0., 16., 0.05]), 'yaw': 1.5708, 'name': 'shelfino1'},
            {'position': np.array([0., 24.0, 0.05]), 'yaw': -1.5708, 'name': 'pollo'},
            {'position': np.array([4., 20.0, 0.05]), 'yaw': 3.14, 'name': 'mario'},
        ]
        
        # Multi-robot mapping
        merged_boxes = self.multi_robot_bayesian_mapping(robot_configs, T=3, box_size=0.1)
        
        # Analysis
        empty_boxes, occupied_boxes, unknown_boxes = self.color_boxes_by_probability(merged_boxes)
        low_density, high_density = self.color_boxes_by_density(occupied_boxes)
        self.clusters = self.cluster_low_density_boxes(low_density, 0.1)
        
        # Visualization
        cluster_geom = self.create_cluster_visualization(0.1)
        infill = self.create_infill_boxes(0.1)
        centroid_geom = self.calculate_global_centroid(0.1)
        
        robot_meshes = []
        for cfg in robot_configs:
            robot_meshes.extend(self.create_robot_frame(cfg['position'], cfg['yaw'], cfg['name'], 0.3))
        
        pcd_vis = copy.deepcopy(self.merged_cloud)
        pcd_vis.paint_uniform_color([0,1,0])
        
        o3d.visualization.draw_geometries(
            [self.voxel_grid, pcd_vis] + [b['bbox'] for b in infill] + 
            cluster_geom + centroid_geom + robot_meshes + [self.bbox],
            window_name="Complete visualization", width=1024, height=768)
        
        print("Point clouds processed successfully!")


def main():
    ply_directory = "/ros2_ws/ply_filtered"
    ply_save_directory = "/ros2_ws/complete_pcl"
    processor = PointCloudProcessor(ply_directory=ply_directory, ply_save_directory=ply_save_directory)

    processor.full_pipeline()
    processor.visualize_point_cloud()

    print("FINISH")
    
if __name__ == "__main__":
    main()