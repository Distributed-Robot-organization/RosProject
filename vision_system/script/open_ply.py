import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("object_small_cone_0_20251011_102949.ply")
print(pcd)
o3d.visualization.draw_geometries([pcd])
