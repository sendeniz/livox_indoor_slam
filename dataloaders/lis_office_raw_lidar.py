import glob
import os
from pathlib import Path
import numpy as np

class CustomLiDARDataset:
    def __init__(self, data_dir: Path, sequence: str, *, topic=None, meta=None, **kwargs):
        """Initialize Custom LiDAR dataset.
        
        Args:
            data_dir: Path to the dataset root directory
            sequence: Sequence number as string (e.g., '0001')
            topic: Ignored (for compatibility with dataset factory)
            meta: Ignored (for compatibility with dataset factory)
            kwargs: Additional ignored arguments (for compatibility)
        """
        self.sequence_id = str(sequence).zfill(4)  # Assuming 4-digit sequence IDs
        self.root_dir = os.path.realpath(data_dir / f"2025_07_01/2025_07_01_drive_{self.sequence_id}_sync/")
        
        # Point cloud directory
        self.lidar_dir = os.path.join(self.root_dir, "livox_points/data/")
        scan_files = sorted(glob.glob(self.lidar_dir + "*.bin"))
        self.scan_files = scan_files  # Using all scans by default
        
        # If you need to correct the scan (like KITTI correction), uncomment below
        # from kiss_icp.pybind import kiss_icp_pybind
        # self.correct_scan = lambda frame: np.asarray(
        #     kiss_icp_pybind._Vector3dVector(frame)
        # )
        # Otherwise just pass through
        self.correct_scan = lambda frame: frame

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def read_point_cloud(self, scan_file: str):
        # For x,y,z only data (3 floats per point)
        points = np.fromfile(scan_file, dtype=np.float32).reshape((-1, 3)).astype(np.float64)
        points = self.correct_scan(points)
        return points, self.get_timestamps(points)
    

    @staticmethod
    def get_timestamps(points):
        # Same timestamp calculation as KITTI (based on yaw angle)
        x = points[:, 0]
        y = points[:, 1]
        yaw = -np.arctan2(y, x)
        timestamps = 0.5 * (yaw / np.pi + 1.0)
        return timestamps