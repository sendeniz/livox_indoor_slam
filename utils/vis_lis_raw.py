import os
import numpy as np
import open3d as o3d

def read_bin_file(bin_path):
    """Read a point cloud from .bin file (x,y,z format)"""
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 3)
    return points

def visualize_two_point_clouds(pc1, pc2, name1="First Cloud", name2="Last Cloud"):
    """Visualize two point clouds together with different colors"""
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pc1)
    pcd1.paint_uniform_color([1, 0, 0])  # Red
    
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pc2)
    pcd2.paint_uniform_color([0, 0, 1])  # Blue
    
    print(f"Visualizing raw point clouds:\n- {name1} (Red)\n- {name2} (Blue)")
    o3d.visualization.draw_geometries([pcd1, pcd2])

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Visualize first and last point clouds')
    parser.add_argument('folder_path', help='Path to folder containing .bin files')
    args = parser.parse_args()

    # Get sorted .bin files
    bin_files = sorted([f for f in os.listdir(args.folder_path) if f.endswith('.bin')])
    if len(bin_files) < 2:
        raise ValueError("Need at least 2 .bin files in the folder")

    # Load first and last clouds
    first_pc = read_bin_file(os.path.join(args.folder_path, bin_files[0]))
    last_pc = read_bin_file(os.path.join(args.folder_path, bin_files[-1]))

    # Visualize them together
    visualize_two_point_clouds(first_pc, last_pc, bin_files[0], bin_files[-1])