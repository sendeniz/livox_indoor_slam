import argparse
import numpy as np
import open3d as o3d
from pathlib import Path

def load_point_cloud(bin_file):
    pc = np.fromfile(bin_file, dtype=np.float32)
    print(f"Raw loaded shape: {pc.shape} from {bin_file.name}")  # Flat array
    pc = pc.reshape(-1, 3)
    print(f"Reshaped to: {pc.shape}")
    print("First 3 raw points:")
    print(pc[:3])
    return pc

def load_pose(txt_file):
    with open(txt_file, "r") as f:
        line = f.readline().strip()
        pose = list(map(float, line.split()))  # [x, y, phi]
        print(f"Loaded pose from {txt_file.name}: {pose}")
        return pose

def transform_point_cloud(points, x, y, phi):
    print(f"Pose before transform: x={x}, y={y}, phi={phi}")
    c, s = np.cos(phi), np.sin(phi)
    R = np.array([[c, -s, 0],
                  [s,  c, 0],
                  [0,  0, 1]])
    T = np.array([x, y, 0.0])
    transformed = (R @ points.T).T + T
    print(f"Transformed shape: {transformed.shape}")
    print("First 3 transformed points:")
    print(transformed[:3])
    return transformed

def visualize_all_transformed_point_clouds(pc_folder, pose_folder):
    pc_files = sorted(Path(pc_folder).glob("*.bin"))
    pose_files = sorted(Path(pose_folder).glob("*.txt"))

    if len(pc_files) != len(pose_files):
        print("Mismatch in number of point clouds and pose files.")
        return

    geometries = []

    for pc_file, pose_file in zip(pc_files, pose_files):
        print(f"\nProcessing:\n  Point Cloud: {pc_file}\n  Pose: {pose_file}")
        pc = load_point_cloud(pc_file)
        x, y, phi = load_pose(pose_file)
        pc_transformed = transform_point_cloud(pc, x, y, phi)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_transformed)
        pcd.paint_uniform_color([0.8, 0.3, 0.3])
        geometries.append(pcd)

    print(f"\nLoaded and transformed {len(geometries)} point clouds.")
    o3d.visualization.draw_geometries(geometries, window_name="All Transformed Point Clouds")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize all transformed point clouds using x,y,phi poses.")
    parser.add_argument("pc_folder", type=str, help="Path to folder with .bin point clouds.")
    parser.add_argument("pose_folder", type=str, help="Path to folder with .txt pose files.")
    args = parser.parse_args()

    visualize_all_transformed_point_clouds(args.pc_folder, args.pose_folder)
