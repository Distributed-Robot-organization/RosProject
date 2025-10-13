import os
import datetime
import open3d as o3d
import numpy as np
import copy



class PointCloudManager:
    
    def __init__(self, detected_objects_list, output_dir="output", voxel_size=0.002, point_clouds=None):
        
        
        self.detected_objects_list = detected_objects_list
        self.true_object_pcl = []
        self.point_clouds = point_clouds
        self.output_dir = output_dir
        
        self.voxel_size = voxel_size
        
        
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_3d_points_from_bbox()
    
    def get_3d_points_from_bbox(self):
        if not self.detected_objects_list:
            print("[WARN] NO OBJECT.")
            return

        print(f"[INFO] elboration of {len(self.detected_objects_list)} object detected...")

        for obj in self.detected_objects_list:
            label = obj.get('label', 'unknown')
            obj_id = obj.get('id', -1)
            points_3d = obj.get('pcl_object', [])

            if not points_3d:
                print(f"[WARN] Nessun punto 3D valido per l'oggetto '{label}' (ID {obj_id}).")
                continue

            self.true_object_pcl.append({
                'label': label,
                'id': obj_id,
                'points': points_3d
            })
            print(f"[INFO] Oggetto '{label}' (ID {obj_id}) → trovati {len(points_3d)} punti 3D.")

    def save_object_pointcloud_to_file(self):
        
        if not self.true_object_pcl:
            print(f"[WARN] Nessun oggetto da salvare in true_object_pcl.")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        for obj_data in self.true_object_pcl:
            label = obj_data.get('label', 'unknown')
            object_id = obj_data.get('id', -1)
            points_3d = obj_data.get('points', [])

            if not points_3d:
                print(f"[WARN] Nessun punto da salvare per l'oggetto '{label}' (ID {object_id}).")
                continue

            filename = os.path.join(self.output_dir, f"object_{label}_{object_id}_{timestamp}.ply")

            try:
                with open(filename, 'w') as f:
                    # Header PLY
                    f.write("ply\n")
                    f.write("format ascii 1.0\n")
                    f.write(f"comment Oggetto '{label}' (ID {object_id})\n")
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

                print(f"[✓] Salvato {len(points_3d)} punti in '{filename}'.")

            except Exception as e:
                print(f"[ERROR] Errore nel salvataggio del file PLY: {e}")

    def load_point_clouds_from_files(self, file_paths):
        
        
        for file_path in file_paths:
            if not os.path.exists(file_path):
                print(f"[WARN] File non trovato: {file_path}")
                continue
            
            pcd = o3d.io.read_point_cloud(file_path)
            print(f"[INFO] Caricata point cloud da '{file_path}': {len(pcd.points)} punti")
            self.point_clouds.append(pcd)
        
        return self.point_clouds
    
    # DA SISTEMARE !!!!!!!
    def visualize_point_clouds(self, point_clouds, colors=None, window_name="Point Clouds", 
                              show_normals=False):
        """Visualizza multiple point cloud con colori diversi"""
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
    
    def merge_point_clouds(self, point_clouds, clean=True):
        """Combina multiple point cloud in una sola"""
        if not point_clouds:
            print("[WARN] Nessuna point cloud da combinare.")
            return None
        
        print(f"\n[INFO] Combinazione di {len(point_clouds)} point clouds...")
        
        # Pulisci ogni point cloud se richiesto
        if clean:
            cleaned_pcds = []
            for i, pcd in enumerate(point_clouds):
                print(f"[INFO] Pulizia point cloud {i+1}/{len(point_clouds)}...")
                cleaned_pcd = self.clean_and_smooth_point_cloud(pcd)
                print(f"  Punti: {len(pcd.points)} -> {len(cleaned_pcd.points)}")
                cleaned_pcds.append(cleaned_pcd)
            point_clouds = cleaned_pcds
        
        # Combina tutte le point cloud
        combined_pcd = point_clouds[0]
        for pcd in point_clouds[1:]:
            combined_pcd += pcd
        
        print(f"[INFO] Punti totali combinati: {len(combined_pcd.points)}")
        
        # Rimuovi duplicati
        combined_pcd = combined_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        print(f"[INFO] Punti dopo rimozione duplicati: {len(combined_pcd.points)}")
        
        # Rimuovi outliers globali
        combined_pcd, _ = combined_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"[INFO] Punti dopo rimozione outliers: {len(combined_pcd.points)}")
        
        # Ricalcola le normali
        combined_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
        )
        combined_pcd.orient_normals_consistent_tangent_plane(30)
        
        return combined_pcd
    
    
    def create_mesh_from_pointcloud(self, pcd, depth=9, density_quantile=0.1):
        """Crea una mesh triangolare dalla point cloud usando Poisson Reconstruction"""
        print("\n[INFO] Creazione mesh con Poisson Surface Reconstruction...")
        
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=depth
        )
        print(f"[INFO] Mesh creata: {len(mesh.vertices)} vertici, {len(mesh.triangles)} triangoli")
        
        # Rimuovi vertici a bassa densità
        vertices_to_remove = densities < np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(vertices_to_remove)
        print(f"[INFO] Mesh dopo pulizia: {len(mesh.vertices)} vertici, {len(mesh.triangles)} triangoli")
        
        return mesh
    
    def reconstruct_object_from_multiple_views(self, file_paths, save_output=True, 
                                               visualize=True, create_mesh=True):
        """
        Pipeline completa: carica, pulisci, combina e ricostruisci oggetto da multiple viste
        """
        print("\n" + "="*60)
        print("RICOSTRUZIONE 3D DA MULTIPLE VISTE")
        print("="*60)
        
        # 1. Carica point clouds
        print("\n[STEP 1] Caricamento point clouds...")
        point_clouds = self.load_point_clouds_from_files(file_paths)
        
        if not point_clouds:
            print("[ERROR] Nessuna point cloud caricata.")
            return None, None
        
        # 2. Visualizza point clouds originali
        if visualize:
            print("\n[STEP 2] Visualizzazione point clouds originali...")
            self.visualize_point_clouds(
                point_clouds,
                window_name="Point Clouds Originali"
            )
        
        # 3. Combina e pulisci
        print("\n[STEP 3] Combinazione e pulizia point clouds...")
        combined_pcd = self.merge_point_clouds(point_clouds, clean=True)
        
        if combined_pcd is None:
            print("[ERROR] Errore nella combinazione delle point clouds.")
            return None, None
        
        # 4. Visualizza risultato combinato
        if visualize:
            print("\n[STEP 4] Visualizzazione point cloud combinata...")
            combined_pcd_vis = copy.deepcopy(combined_pcd)
            combined_pcd_vis.paint_uniform_color([0.5, 0.5, 0.5])
            o3d.visualization.draw_geometries(
                [combined_pcd_vis],
                window_name="Point Cloud Combinata",
                width=1024,
                height=768
            )
        
        # 5. Salva point cloud combinata
        if save_output:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            pcd_output = os.path.join(self.output_dir, f"reconstructed_pcd_{timestamp}.ply")
            o3d.io.write_point_cloud(pcd_output, combined_pcd)
            print(f"[✓] Point cloud salvata: {pcd_output}")
        
        # 6. Crea mesh (opzionale)
        mesh = None
        if create_mesh:
            print("\n[STEP 5] Creazione mesh...")
            mesh = self.create_mesh_from_pointcloud(combined_pcd)
            
            if visualize and mesh:
                print("\n[STEP 6] Visualizzazione mesh...")
                mesh_vis = copy.deepcopy(mesh)
                mesh_vis.paint_uniform_color([1, 0.706, 0])
                o3d.visualization.draw_geometries(
                    [mesh_vis],
                    window_name="Mesh Ricostruita",
                    width=1024,
                    height=768
                )
            
            if save_output and mesh:
                mesh_output = os.path.join(self.output_dir, f"reconstructed_mesh_{timestamp}.ply")
                o3d.io.write_triangle_mesh(mesh_output, mesh)
                print(f"[✓] Mesh salvata: {mesh_output}")
        
        print("\n" + "="*60)
        print("RICOSTRUZIONE COMPLETATA")
        print("="*60)
        
        return combined_pcd, mesh
    
    # == filter function
    def clean_and_smooth_point_cloud(self, pcd):
        # Remove outliers
        pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        pcd_uniform = pcd_clean.voxel_down_sample(voxel_size=self.voxel_size)
        
        # Stima delle normali
        pcd_uniform.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 5, max_nn=30
            )
        )
        # Orienta le normali in modo consistente
        pcd_uniform.orient_normals_consistent_tangent_plane(30)
        
        return pcd_uniform