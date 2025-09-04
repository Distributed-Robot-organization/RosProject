import open3d as o3d

# Specifica il percorso del file .ply
file_path = "object_small_cone_0.ply"

# Carica il file .ply
point_cloud = o3d.io.read_point_cloud(file_path)

# Visualizza la point cloud
o3d.visualization.draw_geometries([point_cloud])