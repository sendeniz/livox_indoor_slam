import os
import argparse
import numpy as np
import open3d as o3d

def read_bin_file(bin_path):
    """Read a KITTI .bin file and return points (x, y, z)."""
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return points[:, :3]  # Return only x, y, z (ignore reflectance)

def visualize_two_point_clouds(pc1, pc2, name1="Cloud 1", name2="Cloud 2"):
    """Visualize two point clouds together with different colors."""
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pc1)
    pcd1.paint_uniform_color([1, 0, 0])  # Red
    
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pc2)
    pcd2.paint_uniform_color([0, 0, 1])  # Blue
    
    # Set names for the legend (Open3D doesn't natively support legends, but this helps in code)
    print(f"Visualizing:\n- {name1} (Red)\n- {name2} (Blue)")
    o3d.visualization.draw_geometries([pcd1, pcd2])

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Visualize two KITTI LiDAR point clouds.')
    parser.add_argument('--folder_path', type=str, required=True,
                       help='Path to the folder containing .bin files (e.g., "velodyne_points/data")')
    args = parser.parse_args()

    # Automatically pick the first and last point cloud in the folder
    bin_files = sorted([f for f in os.listdir(args.folder_path) if f.endswith('.bin')])
    if len(bin_files) < 2:
        raise ValueError("Need at least 2 .bin files in the folder")

    file1 = bin_files[0]    # First file
    file2 = bin_files[-1]   # Last file (change index if you want specific files)

    # Load the two point clouds
    pc1 = read_bin_file(os.path.join(args.folder_path, file1))
    pc2 = read_bin_file(os.path.join(args.folder_path, file2))

    # Visualize them together
    visualize_two_point_clouds(pc1, pc2, file1, file2)