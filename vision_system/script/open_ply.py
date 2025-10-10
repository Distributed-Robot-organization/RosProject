import open3d as o3d
import numpy as np
# Carica il file
pcd = o3d.io.read_point_cloud("object_small_cone_0.ply")

# Stampa alcune info
print(pcd)
# Visualizza la nuvola
o3d.visualization.draw_geometries([pcd])
