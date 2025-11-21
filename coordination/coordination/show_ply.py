import open3d as o3d
from pathlib import Path


def show_ply_sequence(ply_directory: str):
    ply_directory = Path(ply_directory)

    if not ply_directory.exists():
        raise FileNotFoundError(f"Folder does not exist: {ply_directory}")

    ply_files = sorted(ply_directory.glob("*.ply"))

    if not ply_files:
        print(f"No .ply files found in {ply_directory}")
        return

    print(f"Found {len(ply_files)} .ply files")

    for i, ply_path in enumerate(ply_files, start=1):
        print(f"[{i}/{len(ply_files)}] Displaying: {ply_path.name}")

        pcd = o3d.io.read_point_cloud(str(ply_path))
        if pcd.is_empty():
            print(f"  -> WARNING: {ply_path.name} is empty, skipping.")
            continue

        o3d.visualization.draw_geometries(
            [pcd],
            window_name=f"{i}/{len(ply_files)} - {ply_path.name}",
            width=800,
            height=600
        )


def merge_all_pointclouds(ply_directory: str, voxel_size=0.0):
    ply_directory = Path(ply_directory)

    ply_files = sorted(ply_directory.glob("*.ply"))
    if not ply_files:
        print(f"No .ply files found in {ply_directory}")
        return None

    merged = o3d.geometry.PointCloud()

    print(f"Merging {len(ply_files)} point clouds...")

    for ply_path in ply_files:
        pcd = o3d.io.read_point_cloud(str(ply_path))
        if not pcd.is_empty():
            merged += pcd

    print(f"Total merged points: {len(merged.points)}")

    if voxel_size > 0:
        print(f"Applying voxel downsample (voxel_size={voxel_size})...")
        merged = merged.voxel_down_sample(voxel_size)

    print(f"Final points after downsample: {len(merged.points)}")

    return merged


def visualize_merged_cloud(ply_directory: str, voxel_size=0.0):
    merged = merge_all_pointclouds(ply_directory, voxel_size)
    if merged is None:
        return
    o3d.visualization.draw_geometries([merged], window_name="Merged Point Cloud")



if __name__ == "__main__":
    directory = "/ros2_ws/src/working_directory/point_cloud/filtered_ply"

    # 1) visualizza ogni singola cloud
    show_ply_sequence(directory)

    # 2) visualizza la cloud unita
    visualize_merged_cloud(directory, voxel_size=0.01)
