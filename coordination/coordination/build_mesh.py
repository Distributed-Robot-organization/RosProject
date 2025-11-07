import open3d as o3d
import numpy as np
import os

class BuildMesh:
    def __init__(self, ply_save_directory, cloud_file="complete_cloud.ply"):
        self.ply_save_directory = ply_save_directory
        self.cloud_file = cloud_file
        self.join_path = os.path.join(self.ply_save_directory, self.cloud_file)
        self.mesh = None
    
    def generate_mesh(self):
        print(f"Loading point cloud from {self.join_path}")
        pcd = o3d.io.read_point_cloud(self.join_path)
        
        if not pcd.has_points():
            print("Error: Point cloud is empty")
            return
        
        print(f"Point cloud loaded with {len(pcd.points)} points")
        
        if not pcd.has_normals():
            print("Estimating normals...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
            pcd.orient_normals_consistent_tangent_plane(k=15)
        
        print("Generating mesh using Poisson reconstruction...")
        self.mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9
        )
        
        densities = np.asarray(densities)
        density_threshold = np.quantile(densities, 0.01)
        vertices_to_remove = densities < density_threshold
        self.mesh.remove_vertices_by_mask(vertices_to_remove)
        
        print(f"Mesh generated with {len(self.mesh.vertices)} vertices and {len(self.mesh.triangles)} triangles")
        
    def save_mesh(self):
        if self.mesh is None:
            print("Error: No mesh to save. Generate mesh first.")
            return
        
        mesh_path = os.path.join(self.ply_save_directory, "mesh_cloud.ply")
        print(f"Saving mesh to {mesh_path}")
        o3d.io.write_triangle_mesh(mesh_path, self.mesh)
        print("Mesh saved successfully")
       
    
    def visualize_mesh(self):
        if self.mesh is None:
            print("Error: No mesh to visualize. Generate mesh first.")
            return
        
        o3d.visualization.draw_geometries([self.mesh])
       
       
       
def main():
    ply_save_directory = "/ros2_ws/src/working_directory/mesh"
    cloud_file = "complete_cloud.ply"
    processor = BuildMesh(ply_save_directory, cloud_file)
    processor.generate_mesh()
    processor.save_mesh()
    processor.visualize_mesh()

    print("FINISH")
    
if __name__ == "__main__":
    main()