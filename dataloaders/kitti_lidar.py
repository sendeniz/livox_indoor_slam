import glob
import os
import numpy as np


class KITTILiDARDataset:
    def __init__(self, data_dir, sequence: str, *args, **kwargs):
        """Initialize the dataset with only LiDAR capabilities.
        
        Args:
            data_dir: Root directory of the KITTI dataset
            sequence: Sequence number as string (e.g., '00', '01', etc.)
            *args, **kwargs: Additional arguments are ignored to maintain compatibility
        """
        self.sequence_id = str(sequence).zfill(2)
        self.velodyne_dir = os.path.join(data_dir, "sequences", self.sequence_id, "velodyne")
        self.scan_files = sorted(glob.glob(os.path.join(self.velodyne_dir, "*.bin")))

    def __len__(self):
        """Return the number of scans in the sequence."""
        return len(self.scan_files)

    def __getitem__(self, idx):
        """Get scan at index idx."""
        return self.get_scan(idx), np.array([])  # Return empty array for compatibility

    def get_scan(self, idx):
        """Load and return a single LiDAR scan as Nx3 numpy array."""
        # Read binary file and reshape to Nx4 (x,y,z,intensity)
        points = np.fromfile(self.scan_files[idx], dtype=np.float32).reshape((-1, 4))
        
        # Return only x,y,z coordinates (remove intensity) and convert to float64
        return points[:, :3].astype(np.float64)

    def scans(self, idx):
        """Compatibility method with KISS-ICP pipeline."""
        return self.__getitem__(idx)

    def get_all_scans(self):
        """Generator to iterate through all scans in the sequence."""
        for scan_file in self.scan_files:
            yield self.get_scan(self.scan_files.index(scan_file))