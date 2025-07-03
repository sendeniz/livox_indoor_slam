import argparse
import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData

def visualize_ply(ply_path):
    plydata = PlyData.read(ply_path)
    vertices = plydata['vertex']
    x = vertices['x']
    y = vertices['y']
    z = vertices['z']
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='b', marker='.', s=1)
    plt.show()

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Visualize KISS-SLAM PLY output')
    parser.add_argument('--plypath', type=str, required=True, help='Path to the PLY file')
    
    args = parser.parse_args()
    
    # Visualize the specified PLY file
    visualize_ply(args.plypath)