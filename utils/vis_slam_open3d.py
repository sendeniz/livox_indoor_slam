import os
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import argparse

def load_point_clouds(ply_folder):
    pcds = []
    filenames = sorted([f for f in os.listdir(ply_folder) if f.endswith(".ply")])
    
    cmap = plt.get_cmap("tab20")
    num_colors = cmap.N
    
    for i, filename in enumerate(filenames):
        path = os.path.join(ply_folder, filename)
        pcd = o3d.io.read_point_cloud(path)
        
        color = cmap(i % num_colors)[:3]  
        pcd.paint_uniform_color(color)
        
        pcds.append(pcd)
    return pcds

def load_camera_trajectory(npy_path):
    poses = np.load(npy_path)  # shape (N, 4, 4)
    points = []
    for pose in poses:
        translation = pose[:3, 3]
        points.append(translation)
    return np.array(points)

def create_trajectory_lineset(points, color=[1, 0, 0]):
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    trajectory = o3d.geometry.LineSet()
    trajectory.points = o3d.utility.Vector3dVector(points)
    trajectory.lines = o3d.utility.Vector2iVector(lines)
    trajectory.colors = o3d.utility.Vector3dVector([color for _ in lines])
    return trajectory

def apply_transformation(points, rotation=np.eye(3), translation=np.zeros(3)):
    """
    Apply rotation and translation to Nx3 points.
    
    Args:
        points (np.array): Nx3 points.
        rotation (np.array): 3x3 rotation matrix.
        translation (np.array): 3-vector translation.
        
    Returns:
        Nx3 transformed points.
    """
    rotated = points @ rotation.T  # rotate points
    translated = rotated + translation  # translate points
    return translated

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Visualize point clouds and optionally camera trajectory.')
    parser.add_argument('--ply_folder', type=str, required=True,
                      help='Path to folder containing .ply files')
    parser.add_argument('--trajectory_file', type=str, required=False, default=None,
                      help='Path to .npy file containing camera trajectory (optional)')
    
    args = parser.parse_args()
    
    # Load point clouds
    pcds = load_point_clouds(args.ply_folder)
    
    # Define default rotation and translation
    thetax = np.pi * 1
    thetay = np.pi * 0.5
    thetaz = np.pi * .5

    rotation_x = np.array([[1, 0, 0],
                         [0, np.cos(thetax), -np.sin(thetax)],
                         [0, np.sin(thetax), np.cos(thetax)]])

    rotation_y = np.array([
        [np.cos(thetay), 0, np.sin(thetay)],
        [0, 1, 0],
        [-np.sin(thetay), 0, np.cos(thetay)]
    ])
    
    rotation_z = np.array([
        [np.cos(thetaz), -np.sin(thetaz), 0],
        [np.sin(thetaz),  np.cos(thetaz), 0],
        [0,          0,         1]
    ])
    
    translation = np.array([0, 0, 0])
    rotation = rotation_x @ rotation_y @ rotation_z
    
    # Initialize list of geometries to visualize
    geometries = pcds.copy()
    
    # Handle trajectory if provided
    if args.trajectory_file is not None:
        traj_points = load_camera_trajectory(args.trajectory_file)
        traj_points_transformed = apply_transformation(traj_points, rotation, translation)
        trajectory = create_trajectory_lineset(traj_points_transformed)
        geometries.append(trajectory)
    
    print("Visualizing point clouds" + (" and trajectory" if args.trajectory_file else ""))
    print("Press 'Q' to exit.")
    o3d.visualization.draw_geometries(geometries)