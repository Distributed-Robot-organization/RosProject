import json
import time
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from pathlib import Path

class ScanningAnalyzer:
    def __init__(self, json_directory, ply_directory, real_centroid):
        self.json_directory = Path(json_directory)
        self.ply_directory = Path(ply_directory)
        self.real_centroid = real_centroid
        
        self.iterations_data = []
        self.point_clouds = []
        self.json_centroids = []       # Centroids calculated from JSON box data
        self.avg_observations = []     # Avg observation values from JSON
        self.point_cloud_centroids = [] # Centroids calculated from actual PLY geometry

    def load_iterations(self, pattern="boxxes_object_*.json"):
        """Loads scanning iteration data from JSON files."""
        json_files = sorted(self.json_directory.glob(pattern))
        
        self.iterations_data = []
        for json_file in json_files:
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                    self.iterations_data.append({
                        'filename': json_file.name,
                        'data': data
                    })
            except Exception as e:
                print(f"Error loading JSON {json_file.name}: {e}")
        
        print(f"Loaded {len(self.iterations_data)} iteration files")
        self._extract_json_data()

    def load_point_clouds(self, pattern="*.ply"):
        """Loads point cloud files from the directory."""
        ply_files = sorted(self.ply_directory.glob(pattern))
        
        self.point_clouds = []
        for ply_file in ply_files:
            try:
                pcd = o3d.io.read_point_cloud(str(ply_file))
                self.point_clouds.append({
                    'filename': ply_file.name,
                    'cloud': pcd,
                    'num_points': len(pcd.points)
                })
                print(f"Loaded: {ply_file.name} ({len(pcd.points)} points)")
            except Exception as e:
                print(f"Error loading PLY {ply_file.name}: {e}")
        
        print(f"\nTotal point clouds loaded: {len(self.point_clouds)}")
        return self.point_clouds

    def _create_centroid_sphere(self, centroid, radius=0.05, color=[1, 0, 1]):
        """Helper to create a visual sphere at a specific coordinate."""
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.translate(centroid)
        sphere.paint_uniform_color(color)
        return sphere

    def visualize_point_clouds(self, mode='animation'):
        """
        Visualizes point clouds.
        Modes: 'all' (static overlay), 'individual' (one by one), 'animation' (looping).
        """
        if not self.point_clouds:
            print("No point clouds loaded. Call load_point_clouds() first.")
            return
        
        # Ensure centroids are calculated before visualization
        if not self.point_cloud_centroids:
            self.calculate_point_cloud_centroids()
        
        print(f"Starting visualization in mode: {mode}")

        if mode == 'all':
            geometries = []
            for i, pc_data in enumerate(self.point_clouds):
                pcd = pc_data['cloud']
                # Color mapping for differentiation
                color = plt.cm.viridis(i / len(self.point_clouds))[:3]
                pcd.paint_uniform_color(color)
                geometries.append(pcd)
                
                # Add centroid if available
                if i < len(self.point_cloud_centroids) and self.point_cloud_centroids[i]['centroid'] is not None:
                    centroid = self.point_cloud_centroids[i]['centroid']
                    geometries.append(self._create_centroid_sphere(centroid, radius=0.05, color=[1, 0, 0]))
            
            o3d.visualization.draw_geometries(geometries, window_name="All Point Clouds", width=1920, height=1080)
        
        elif mode == 'individual':
            for i, pc_data in enumerate(self.point_clouds):
                geometries = [pc_data['cloud']]
                title = f"{pc_data['filename']} ({pc_data['num_points']} points)"
                
                if i < len(self.point_cloud_centroids) and self.point_cloud_centroids[i]['centroid'] is not None:
                    centroid = self.point_cloud_centroids[i]['centroid']
                    geometries.append(self._create_centroid_sphere(centroid, radius=0.05, color=[1, 0, 1]))
                    print(f"Showing: {title} - Centroid: {centroid}")
                else:
                    print(f"Showing: {title}")
                
                o3d.visualization.draw_geometries(geometries, window_name=pc_data['filename'], width=1920, height=1080)
        
        elif mode == 'animation':            
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name="Point Cloud Animation (Loop)", width=1920, height=1080)
            
            # Create a placeholder geometry to initialize the visualizer
            vis.add_geometry(self.point_clouds[0]['cloud'])
            
            frame_delay = 0.5
            print("Animation running. Close window to stop.")
            
            try:
                while True:  # Infinite loop
                    for i, pc_data in enumerate(self.point_clouds):
                        vis.clear_geometries()
                        
                        # Add current cloud
                        vis.add_geometry(pc_data['cloud'], reset_bounding_box=(i == 0))
                        
                        # Add current centroid
                        if i < len(self.point_cloud_centroids) and self.point_cloud_centroids[i]['centroid'] is not None:
                            centroid = self.point_cloud_centroids[i]['centroid']
                            sphere = self._create_centroid_sphere(centroid, radius=0.02, color=[1, 0, 0])
                            vis.add_geometry(sphere, reset_bounding_box=False)
                            print(f"Frame {i+1}/{len(self.point_clouds)}: {pc_data['filename']}")
                        
                        if not vis.poll_events():
                            raise KeyboardInterrupt
                        
                        vis.update_renderer()
                        time.sleep(frame_delay)
            except KeyboardInterrupt:
                print("\nAnimation stopped.")
            finally:
                vis.destroy_window()

    def centroid_errors(self):
        if self.real_centroid is None:
            print("Error: self.real_centroid is not set.")
            return

        # Handle 2D to 3D conversion if necessary (Assume Z=0 for 2D inputs)
        if len(self.real_centroid) == 2:
            real_c = np.array([self.real_centroid[0], self.real_centroid[1], 0.0])
        elif len(self.real_centroid) == 3:
            real_c = self.real_centroid
        else:
            print(f"Error: Unexpected dimension for real_centroid: {len(self.real_centroid)}")
            return

        pc_errors = []
        
        # Calculate errors
        for pc_centroid_data in self.point_cloud_centroids:
            centroid = pc_centroid_data['centroid']
            if centroid is not None:
                error = np.linalg.norm(centroid - real_c)
                pc_errors.append(error)
            else:
                pc_errors.append(np.nan) 
        
        # Align lengths
        iterations = list(range(len(self.iterations_data)))
        pc_errors = pc_errors[:len(iterations)]
        pc_errors_array = np.array(pc_errors)

        # Plotting
        plt.figure(figsize=(14, 7))
        plt.plot(iterations, pc_errors_array, 'r-^', linewidth=2, markersize=6, alpha=0.7, 
                 label='Point Cloud Centroid Error (PLY Data)')
        
        # Calculate stats for visualization
        all_valid_errors = pc_errors_array[~np.isnan(pc_errors_array)]
        min_error = 0.0

        if len(all_valid_errors) > 0:
            min_error = np.nanmin(all_valid_errors)
            max_error = np.nanmax(all_valid_errors)
            
            # Dynamic margin for Y-axis
            error_range = max_error - min_error
            margin = error_range * 0.1 if error_range > 1e-6 else 0.005 
            plt.ylim(min_error - margin, max_error + margin)
            
            plt.axhline(min_error, color='k', linestyle=':', linewidth=1, 
                        label=f'Min Error: {min_error:.4f}')

        plt.xlabel('Iteration', fontsize=12, fontweight='bold')
        plt.ylabel(f'Error (Distance from {real_c})', fontsize=12, fontweight='bold')
        plt.title('Centroid Error Progression', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.4, linestyle='--')
        plt.legend(loc='upper right', fontsize=10)
        
        # Stats Box
        stats_text = (f'Real Centroid: [{real_c[0]:.2f}, {real_c[1]:.2f}, {real_c[2]:.2f}]\n'
                      f'Avg Error: {np.nanmean(pc_errors_array):.4f}\n'
                      f'Min Error: {min_error:.4f}')
        props = dict(boxstyle='round', facecolor='lightcoral', alpha=0.7)
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, fontsize=10,
                 verticalalignment='top', bbox=props)
        
        plt.show()
        return pc_errors

    def _extract_json_data(self):
        """Parses loaded JSON data to extract weighted centroids and observations."""
        self.json_centroids = []
        self.avg_observations = []

        for iteration in self.iterations_data:
            data = iteration['data']
            
            if not data or not isinstance(data, list):
                self.json_centroids.append(None)
                self.avg_observations.append(0.0)
                continue

            # Filter valid boxes
            valid_boxes = [(idx, box) for idx, box in enumerate(data) 
                           if box.get('avg_observation') is not None]
            
            if valid_boxes:
                weighted_sum = np.zeros(3)
                total_weight = 0
                
                for idx, box in valid_boxes:
                    weight = box['avg_observation']
                    
                    # Convert linear index to 3D coordinates (assuming 10x10xN grid structure)
                    z = idx // 100
                    y = (idx % 100) // 10
                    x = idx % 10
                    
                    position = np.array([x, y, z]) * 0.1
                    weighted_sum += position * weight
                    total_weight += weight
                
                if total_weight > 0:
                    self.json_centroids.append(weighted_sum / total_weight)
                else:
                    self.json_centroids.append(None)
                
                # Store average observation
                avg_obs = np.mean([box['avg_observation'] for _, box in valid_boxes])
                self.avg_observations.append(avg_obs)
            else:
                self.json_centroids.append(None)
                self.avg_observations.append(0.0)
    
    def plot_scanning_progress(self, save_path=None):
        """Plots the progress of observation quality over iterations."""
        if not self.avg_observations:
            print("Warning: No observation data available.")
            return
        
        iterations = list(range(len(self.avg_observations)))
        
        plt.figure(figsize=(14, 7))
        plt.plot(iterations, self.avg_observations, 'g-o', linewidth=2, markersize=4, alpha=0.7)
        plt.fill_between(iterations, self.avg_observations, alpha=0.2, color='green')
        
        plt.xlabel('Iteration', fontsize=12, fontweight='bold')
        plt.ylabel('Average Observation Value', fontsize=12, fontweight='bold')
        plt.title('Scanning Coverage Quality', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, linestyle='--')
        
        if self.avg_observations:
            min_obs = min(self.avg_observations)
            max_obs = max(self.avg_observations)
            
            # Calculate dynamic margin (10% of the range) or fixed if flat
            margin = (max_obs - min_obs) * 0.1 if max_obs > min_obs else 0.05
            plt.ylim(min_obs - margin, max_obs + margin)

        # Trend line calculation
        if len(iterations) > 1:
            z = np.polyfit(iterations, self.avg_observations, 1)
            p = np.poly1d(z)
            plt.plot(iterations, p(iterations), "r--", alpha=0.6, linewidth=2, label='Trend')
            
            improvement = self.avg_observations[-1] - self.avg_observations[0]
            
            # Safe percentage calculation
            start_val = self.avg_observations[0]
            improvement_pct = (improvement / start_val * 100) if start_val != 0 else 0.0

            textstr = (f'Start: {self.avg_observations[0]:.4f}\n'
                       f'End: {self.avg_observations[-1]:.4f}\n'
                       f'Improvement: {improvement:.4f} ({improvement_pct:.2f}%)')
            
            props = dict(boxstyle='round', facecolor='lightgreen', alpha=0.7)
            plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, verticalalignment='top', bbox=props)
        
        plt.legend(loc='best')
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_all(self, visualize_clouds=True, cloud_mode='all'):
        """Wrapper to execute all plotting and visualization functions."""
        self.plot_scanning_progress()
        
        if visualize_clouds and self.point_clouds:
            self.visualize_point_clouds(mode=cloud_mode)
    
    def get_statistics(self):
        """Return dictionary of scanning statistics."""
        stats = {
            'total_iterations': len(self.iterations_data),
            'json_centroids_found': len([c for c in self.json_centroids if c is not None]),
            'ply_centroids_found': len([c for c in self.point_cloud_centroids if c['centroid'] is not None])
        }
        
        if self.avg_observations:
            stats['observation_start'] = self.avg_observations[0]
            stats['observation_end'] = self.avg_observations[-1]
            stats['net_improvement'] = stats['observation_end'] - stats['observation_start']
            
        return stats
    
    def calculate_point_cloud_centroids(self):
        """Calculates the arithmetic mean (centroid) of every loaded point cloud."""
        if not self.point_clouds:
            print("No point clouds loaded. Call load_point_clouds() first.")
            return []
        
        self.point_cloud_centroids = []
        print(f"Calculating centroids for {len(self.point_clouds)} clouds...")
        
        for i, pc_data in enumerate(self.point_clouds):
            pcd = pc_data['cloud']
            points = np.asarray(pcd.points)
            
            centroid_entry = {
                'filename': pc_data['filename'],
                'num_points': pc_data['num_points'],
                'centroid': None
            }

            if len(points) > 0:
                centroid = np.mean(points, axis=0)
                centroid_entry['centroid'] = centroid
            else:
                print(f"Warning: Cloud {pc_data['filename']} is empty.")
            
            self.point_cloud_centroids.append(centroid_entry)
        
        return self.point_cloud_centroids


if __name__ == "__main__":
    # Configuration
    JSON_DIR = "/ros2_ws/src/working_directory/mesh/history_all_json"
    PLY_DIR = "/ros2_ws/src/working_directory/mesh/history_all_mesh"
    REAL_CENTROID = np.array([0.0, 20.0])  
    
    # Initialize
    analyzer = ScanningAnalyzer(JSON_DIR, PLY_DIR, REAL_CENTROID)
    
    # Load Data
    analyzer.load_iterations()
    analyzer.load_point_clouds()
    
    # Calculations
    analyzer.calculate_point_cloud_centroids()
    analyzer.centroid_errors()
    
    # Visualization
    analyzer.plot_all(visualize_clouds=True, cloud_mode='animation')
    
    # Statistics
    print("\nScanning Statistics:")
    for key, value in analyzer.get_statistics().items():
        print(f"  {key}: {value}")