import open3d as o3d
import numpy as np


# def remove_background(pcd: o3d.geometry.PointCloud, method: str = "plane", **kwargs):
#     if method == "plane":
#         distance_threshold = kwargs.get("distance_threshold", 0.003)
#         ransac_n = kwargs.get("ransac_n", 3)
#         num_iterations = kwargs.get("num_iterations", 2000)

#         # Downsample opzionale
#         voxel = kwargs.get("voxel_size", None)
#         working = pcd.voxel_down_sample(voxel) if voxel else pcd

#         plane_model, inliers = working.segment_plane(
#             distance_threshold=distance_threshold,
#             ransac_n=ransac_n,
#             num_iterations=num_iterations
#         )
        
#         # Separa piano e oggetto
#         if voxel:
#             bg = working.select_by_index(inliers)
#             fg = working.select_by_index(inliers, invert=True)
#         else:
#             bg = pcd.select_by_index(inliers)
#             fg = pcd.select_by_index(inliers, invert=True)

#         return fg, bg, {"plane_model": plane_model}

#     elif method == "statistical":
#         nb_neighbors = kwargs.get("nb_neighbors", 20)
#         std_ratio = kwargs.get("std_ratio", 2.0)
#         fg, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
#         bg = pcd.select_by_index(ind, invert=True)
#         return fg, bg, None

#     elif method == "radius":
#         nb_points = kwargs.get("nb_points", 16)
#         radius = kwargs.get("radius", 0.02)
#         fg, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
#         bg = pcd.select_by_index(ind, invert=True)
#         return fg, bg, None
    
#     elif method == "crop_z":
#         # Rimuove punti sopra una certa altezza Z
#         points = np.asarray(pcd.points)
#         z_threshold = kwargs.get("z_threshold", None)
        
#         if z_threshold is None:
#             # Calcola automaticamente: media + 2 deviazioni standard
#             z_mean = np.mean(points[:, 2])
#             z_std = np.std(points[:, 2])
#             z_threshold = z_mean + 2 * z_std
        
#         # Maschera: mantieni solo punti sotto la soglia
#         mask = points[:, 2] < z_threshold
#         indices = np.where(mask)[0]
        
#         fg = pcd.select_by_index(indices)
#         bg = pcd.select_by_index(indices, invert=True)
        
#         return fg, bg, {"z_threshold": z_threshold}

#     else:
#         raise ValueError(f"Metodo sconosciuto: {method}")


# if __name__ == "__main__":
#     pcd = o3d.io.read_point_cloud("object_small_cone_0_20251012_080741.ply")
#     print(f"Point cloud originale: {np.asarray(pcd.points).shape[0]} punti")

#     # Step 1: Rimuovi piano principale (tavolo)
#     foreground, background, info = remove_background(
#         pcd,
#         method="plane",
#         distance_threshold=0.003,
#         ransac_n=3,
#         num_iterations=3000,
#     )
#     print(f"Dopo rimozione piano: {np.asarray(foreground.points).shape[0]} punti")

#     # Step 2: Rimuovi punti sopra (soffitto/outlier alti)
#     foreground_cropped, ceiling, crop_info = remove_background(
#         foreground,
#         method="crop_z",
#         # z_threshold=0.15  # Specifica manualmente se necessario
#     )
#     print(f"Dopo crop Z: {np.asarray(foreground_cropped.points).shape[0]} punti")
#     print(f"Soglia Z applicata: {crop_info['z_threshold']:.4f}")

#     # Step 3: Pulizia finale outlier
#     foreground_clean, _, _ = remove_background(
#         foreground_cropped,
#         method="statistical",
#         nb_neighbors=30,
#         std_ratio=1.5
#     )
#     print(f"Dopo pulizia finale: {np.asarray(foreground_clean.points).shape[0]} punti")

#     # Visualizza risultato
#     foreground_clean.paint_uniform_color([0.2, 0.8, 0.2])
#     o3d.visualization.draw_geometries([foreground_clean])

#     # Salva
#     o3d.io.write_point_cloud("object_small_cone_clean.ply", foreground_clean)

pcd = o3d.io.read_point_cloud("cone_reconstructed.ply.ply")
print(pcd)
o3d.visualization.draw_geometries([pcd])
