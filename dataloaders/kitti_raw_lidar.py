import glob
import os
from pathlib import Path
import numpy as np

__raw_to_odometry_mapping__ = {
    "00": "2011_10_03/2011_10_03_drive_0027_sync/",
    "01": "2011_10_03/2011_10_03_drive_0042_sync/",
    "02": "2011_10_03/2011_10_03_drive_0034_sync/",
    "04": "2011_09_30/2011_09_30_drive_0016_sync/",
    "05": "2011_09_30/2011_09_30_drive_0018_sync/",
    "06": "2011_09_30/2011_09_30_drive_0020_sync/",
    "07": "2011_09_30/2011_09_30_drive_0027_sync/",
    "08": "2011_09_30/2011_09_30_drive_0028_sync/",
    "09": "2011_09_30/2011_09_30_drive_0033_sync/",
    "10": "2011_09_30/2011_09_30_drive_0034_sync/",
}


class KITTILiDARDataset:
    def __init__(self, data_dir: Path, sequence: str, *, topic=None, meta=None, **kwargs):
        """Initialize KITTI LiDAR dataset.
        
        Args:
            data_dir: Path to the dataset root directory
            sequence: Sequence number as string (e.g., '00')
            topic: Ignored (for compatibility with dataset factory)
            meta: Ignored (for compatibility with dataset factory)
            kwargs: Additional ignored arguments (for compatibility)
        """
        self.sequence_id = str(sequence).zfill(2)
        self.root_dir = os.path.realpath(data_dir / __raw_to_odometry_mapping__[self.sequence_id])
        self.valid_idx = self.get_benchmark_indices(self.sequence_id)

        self.velodyne_dir = os.path.join(self.root_dir, "velodyne_points/data/")
        scan_files = sorted(glob.glob(self.velodyne_dir + "*.bin"))
        self.scan_files = scan_files[self.valid_idx[0] : self.valid_idx[1] + 1]

        # Add correction for KITTI datasets, can be easily removed if unwanted
        from kiss_icp.pybind import kiss_icp_pybind
        self.correct_kitti_scan = lambda frame: np.asarray(
            kiss_icp_pybind._correct_kitti_scan(kiss_icp_pybind._Vector3dVector(frame))
        )

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def read_point_cloud(self, scan_file: str):
        points = np.fromfile(scan_file, dtype=np.float32).reshape((-1, 4))[:, :3].astype(np.float64)
        points = self.correct_kitti_scan(points)
        return points, self.get_timestamps(points)

    @staticmethod
    def get_timestamps(points):
        x = points[:, 0]
        y = points[:, 1]
        yaw = -np.arctan2(y, x)
        timestamps = 0.5 * (yaw / np.pi + 1.0)
        return timestamps

    @staticmethod
    def get_benchmark_indices(sequence_id: str):
        _raw_to_benchmark_indices = {
            "00": [0, 4540],
            "01": [0, 1100],
            "02": [0, 4660],
            "04": [0, 270],
            "05": [0, 2760],
            "06": [0, 1100],
            "07": [0, 1100],
            "08": [1100, 5170],
            "09": [0, 1590],
            "10": [0, 1200],
        }
        return _raw_to_benchmark_indices[sequence_id]