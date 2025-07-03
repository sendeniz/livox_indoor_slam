import argparse
import numpy as np
import open3d as o3d
from pathlib import Path

def load_point_cloud(bin_file):
    return np.fromfile(bin_file, dtype=np.float32).reshape(-1, 3)

def load_pose(txt_file):
    with open(txt_file, "r") as f:
        line = f.readline().strip()
        return list(map(float, line.split()))  # [x, y, phi]

def transform_point_cloud(points, x, y, phi):
    # 2D rotation matrix in XY plane
    c, s = np.cos(phi), np.sin(phi)
    R = np.array([[c, -s, 0],
                  [s,  c, 0],
                  [0,  0, 1]])
    T = np.array([x, y, 0.0])
    return (R @ points.T).T + T

def create_arrow_at_pose(x, y, phi, length=0.5):
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.02, cone_radius=0.04,
        cylinder_height=length * 0.8, cone_height=length * 0.2
    )
    arrow.paint_uniform_color([1, 0, 0])  # Red arrow

    # Rotation around Z axis
    R = arrow.get_rotation_matrix_from_axis_angle([0, 0, phi])
    arrow.rotate(R, center=(0, 0, 0))

    # Translation to pose location
    arrow.translate((x, y, 0))
    return arrow

def visualize_transformed_point_clouds(pc_folder, pose_folder):
    pc_files = sorted(Path(pc_folder).glob("*.bin"))
    pose_files = sorted(Path(pose_folder).glob("*.txt"))

    if len(pc_files) != len(pose_files):
        print("Mismatch in number of point clouds and pose files.")
        print(f"Point clouds: {len(pc_files)}, Poses: {len(pose_files)}")
        return

    path_points = []
    geometries = []

    for pc_file, pose_file in zip(pc_files, pose_files):
        pc = load_point_cloud(pc_file)
        x, y, phi = load_pose(pose_file)
        pc_transformed = transform_point_cloud(pc, x, y, phi)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_transformed)
        pcd.paint_uniform_color([0.6, 0.6, 0.9])  # Light blue

        # Store as geometry with transparency support
        pcd.estimate_normals()
        geometries.append(pcd)

        # Save pose for path + arrow
        path_points.append([x, y, 0.0])
        geometries.append(create_arrow_at_pose(x, y, phi))

    # Create red path line
    if len(path_points) >= 2:
        points = o3d.utility.Vector3dVector(path_points)
        lines = o3d.utility.Vector2iVector([[i, i+1] for i in range(len(path_points)-1)])
        colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(path_points)-1)])
        line_set = o3d.geometry.LineSet(points=points, lines=lines)
        line_set.colors = colors
        geometries.append(line_set)

    # Use Visualizer to enable transparency
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Clouds with Robot Path & Arrows")

    for geom in geometries:
        vis.add_geometry(geom)

    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.asarray([0.05, 0.05, 0.05])
    opt.mesh_show_back_face = True

    # Enable transparency
    for geom in geometries:
        if isinstance(geom, o3d.geometry.PointCloud):
            geom.paint_uniform_color([0.6, 0.6, 0.9])  # Light blue 


    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize aligned point clouds with path and direction arrows.")
    parser.add_argument("pc_folder", type=str, help="Path to folder with .bin point clouds.")
    parser.add_argument("pose_folder", type=str, help="Path to folder with .txt pose files.")
    args = parser.parse_args()

    visualize_transformed_point_clouds(args.pc_folder, args.pose_folder)
