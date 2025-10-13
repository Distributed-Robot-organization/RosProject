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
    