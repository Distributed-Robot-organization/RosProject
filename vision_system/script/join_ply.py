import open3d as o3d
import numpy as np
import copy

def clean_and_smooth_point_cloud(pcd, voxel_size=0.002):
    """Pulisce e rende più uniforme la point cloud"""
    # Rimuovi outliers statistici
    pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    # Voxel downsampling per uniformare la densità
    pcd_uniform = pcd_clean.voxel_down_sample(voxel_size=voxel_size)
    
    # Stima le normali
    pcd_uniform.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30))
    
    # Orienta le normali in modo consistente
    pcd_uniform.orient_normals_consistent_tangent_plane(30)
    
    return pcd_uniform

# Leggi solo i primi due file .ply
pcd1 = o3d.io.read_point_cloud("cone_view2_clean.ply")
pcd2 = o3d.io.read_point_cloud("cone_view1_clean.ply")

print("Point cloud 1:")
print(pcd1)
print("\nPoint cloud 2:")
print(pcd2)

# Visualizza le point cloud originali
print("\n=== Visualizzazione point clouds originali ===")
pcd1_orig = copy.deepcopy(pcd1)
pcd2_orig = copy.deepcopy(pcd2)
pcd1_orig.paint_uniform_color([1, 0, 0])  # Rosso
pcd2_orig.paint_uniform_color([0, 0, 1])  # Blu
o3d.visualization.draw_geometries([pcd1_orig, pcd2_orig],
                                   window_name="STEP 1: Point clouds originali",
                                   width=1024, height=768)

# Pulisci e rendi uniformi le point cloud
print("\n=== Pulizia e uniformazione point clouds ===")
voxel_size = 0.002  # 2mm - regola per controllare la densità finale
pcd1_clean = clean_and_smooth_point_cloud(pcd1, voxel_size)
pcd2_clean = clean_and_smooth_point_cloud(pcd2, voxel_size)

print(f"Punti pcd1: {len(pcd1.points)} -> {len(pcd1_clean.points)}")
print(f"Punti pcd2: {len(pcd2.points)} -> {len(pcd2_clean.points)}")

# Visualizza le point cloud pulite
print("\n=== Visualizzazione point clouds pulite ===")
pcd1_vis = copy.deepcopy(pcd1_clean)
pcd2_vis = copy.deepcopy(pcd2_clean)
pcd1_vis.paint_uniform_color([1, 0, 0])
pcd2_vis.paint_uniform_color([0, 0, 1])
o3d.visualization.draw_geometries([pcd1_vis, pcd2_vis],
                                   window_name="STEP 2: Point clouds pulite e uniformi",
                                   width=1024, height=768)

# Combina le point cloud (mantenendo le posizioni originali)
print("\n=== Combinazione point clouds ===")
combined_pcd = pcd1_clean + pcd2_clean
print(f"Punti totali combinati: {len(combined_pcd.points)}")

# Rimuovi eventuali duplicati nelle zone di sovrapposizione
combined_pcd_final = combined_pcd.voxel_down_sample(voxel_size=voxel_size)
print(f"Punti dopo rimozione duplicati: {len(combined_pcd_final.points)}")

# Rimuovi outliers globali
combined_pcd_final, ind = combined_pcd_final.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f"Punti dopo rimozione outliers: {len(combined_pcd_final.points)}")

# Ricalcola le normali per la point cloud finale
combined_pcd_final.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
combined_pcd_final.orient_normals_consistent_tangent_plane(30)

# Visualizza il risultato finale
print("\n=== Visualizzazione risultato finale ===")
combined_pcd_final.paint_uniform_color([0.5, 0.5, 0.5])
o3d.visualization.draw_geometries([combined_pcd_final],
                                   window_name="STEP 3: Point cloud finale combinata",
                                   width=1024, height=768,
                                   point_show_normal=False)

# Salva il risultato
output_file = "cone_reconstructed.ply"
o3d.io.write_point_cloud(output_file, combined_pcd_final)
print(f"\nPoint cloud combinata salvata in: {output_file}")

# Salva anche le singole point cloud pulite
o3d.io.write_point_cloud("cone_view1_clean.ply", pcd1_clean)
o3d.io.write_point_cloud("cone_view2_clean.ply", pcd2_clean)
print("Point clouds singole salvate come cone_view1_clean.ply, cone_view2_clean.ply")

# Opzionale: crea una mesh dalla point cloud combinata
print("\n=== Creazione mesh (opzionale) ===")
print("Creazione mesh con Poisson Surface Reconstruction...")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    combined_pcd_final, depth=9)
print(f"Mesh creata con {len(mesh.vertices)} vertici e {len(mesh.triangles)} triangoli")

# Rimuovi vertici a bassa densità
vertices_to_remove = densities < np.quantile(densities, 0.1)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(f"Mesh dopo pulizia: {len(mesh.vertices)} vertici e {len(mesh.triangles)} triangoli")

# Visualizza la mesh
mesh.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([mesh],
                                   window_name="STEP 4: Mesh ricostruita",
                                   width=1024, height=768)

# Salva la mesh
mesh_file = "cone_mesh.ply"
o3d.io.write_triangle_mesh(mesh_file, mesh)
print(f"Mesh salvata in: {mesh_file}")
# Visualizza la mesh
mesh.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([mesh],
                                   window_name="STEP 4: Mesh ricostruita",
                                   width=1024, height=768)

# Salva la mesh
mesh_file = "cone_mesh.ply"
o3d.io.write_triangle_mesh(mesh_file, mesh)
print(f"Mesh salvata in: {mesh_file}")
pcd1_vis.paint_uniform_color([1, 0, 0])
pcd2_vis.paint_uniform_color([0, 0, 1])
pcd3_vis.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([pcd1_vis, pcd2_vis, pcd3_vis],
                                   window_name="STEP 2: Point clouds allineate",
                                   width=1024, height=768)

# Combina le point cloud
print("\n=== Combinazione point clouds ===")
combined_pcd = pcd1 + pcd2_aligned + pcd3_aligned
print(f"Punti totali combinati: {len(combined_pcd.points)}")

# Rimuovi duplicati e punti troppo vicini con voxel downsampling
combined_pcd_clean = combined_pcd.voxel_down_sample(voxel_size=0.002)
print(f"Punti dopo voxel downsampling: {len(combined_pcd_clean.points)}")

# Rimuovi outliers dalla point cloud combinata
combined_pcd_clean, ind = combined_pcd_clean.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f"Punti dopo rimozione outliers: {len(combined_pcd_clean.points)}")

# Stima le normali per la point cloud finale
combined_pcd_clean.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
combined_pcd_clean.orient_normals_consistent_tangent_plane(30)

# Visualizza il risultato finale
print("\n=== Visualizzazione risultato finale ===")
combined_pcd_clean.paint_uniform_color([0.5, 0.5, 0.5])
o3d.visualization.draw_geometries([combined_pcd_clean],
                                   window_name="STEP 3: Point cloud finale combinata",
                                   width=1024, height=768,
                                   point_show_normal=False)

# Salva il risultato
output_file = "cone_reconstructed.ply"
o3d.io.write_point_cloud(output_file, combined_pcd_clean)
print(f"\nPoint cloud combinata salvata in: {output_file}")

# Opzionale: crea una mesh dalla point cloud
print("\n=== Creazione mesh (opzionale) ===")
print("Creazione mesh con Poisson Surface Reconstruction...")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    combined_pcd_clean, depth=9)
print(f"Mesh creata con {len(mesh.vertices)} vertici e {len(mesh.triangles)} triangoli")

# Rimuovi vertici a bassa densità
vertices_to_remove = densities < np.quantile(densities, 0.1)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(f"Mesh dopo pulizia: {len(mesh.vertices)} vertici e {len(mesh.triangles)} triangoli")

# Visualizza la mesh
mesh.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([mesh],
                                   window_name="STEP 4: Mesh ricostruita",
                                   width=1024, height=768)

# Salva la mesh
mesh_file = "cone_mesh.ply"
o3d.io.write_triangle_mesh(mesh_file, mesh)
print(f"Mesh salvata in: {mesh_file}")
