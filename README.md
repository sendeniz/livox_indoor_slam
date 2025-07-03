## Indoor Lidar SLAM with Livox MID-360 at TU Berlin

This project utilizes a **Livox MID-360** LiDAR scanner to collect indoor point cloud data from the **4th floor of the Learning & Intelligent Systems (LIS) Lab** at **TU Berlin**. SLAM is performed using the [**KISS SLAM**](https://github.com/PRBonn/kiss-icp) package.

---

### Data Structure

We provide two formats of the recorded LiDAR data:

- **Raw Format**:  
  Each `.txt` file contains:
  - **First row**: `x`, `y`, `phi` (SE(2) pose of the mobile base around the z-axis).
  - **Subsequent rows**: `x`, `y`, `z` coordinates of the point cloud.
  - **File name**: The UNIX timestamp when the scan was recorded (end time).

- **KITTI-like Format**:
  - `.bin` files: Contain `x`, `y`, `z` point cloud data.
  - `oxts` `.txt` files: Contain the mobile base pose (`x`, `y`, `phi`), matching KITTI-style ground truth.
  - No IMU data used. For SLAM, SE(2) poses are converted to SE(3) by assuming a flat floor.

#### Utility: Convert Raw to KITTI-style

```bash
python3 utils/data_to_kitti_structure.py --input_dir '/home/shk/Downloads/log_data' --output_dir '/home/shk/Downloads'
```

---

### Visualize Point Clouds

#### KITTI-format Visualization

```bash
python3 utils/vis_pc.py '/home/shk/Downloads/2025_07_01/2025_07_01_drive_0001_sync/livox_points/data' '/home/shk/Downloads/2025_07_01/2025_07_01_drive_0001_sync/oxts/data'
```

#### Raw Format:  
Can be adapted from the above script.

---

### Visualize SLAM Output

```bash
python3 utils/vis_slam_open3d.py --ply_folder '/home/shk/Desktop/kiss-slam/slam_output/latest/local_maps/plys'
```

---

### Run KISS SLAM

#### On KITTI Raw LiDAR

```bash
python3 kiss_slam_pipeline '/home/shk/Downloads/raw_data_downloader' --dataloader kitti_raw_lidar --sequence 00
```

#### On LIS Lab (Indoor Livox) Data

```bash
python3 kiss_slam_pipeline '/home/shk/Downloads' --dataloader lis_office_raw_lidar --sequence 000
```

---

### Data Downloads

- **Raw LIS Indoor LiDAR Data**:  
  [Download](https://drive.google.com/file/d/1zLlG1QnEej6WVvuUTs3DnotH8_C0g1wK/view?usp=sharing)

- **KITTI-structured Version**:  
  [Download](https://drive.google.com/file/d/1f_CPt3jmUBkAX8Y7_8PyVxibmIxuWbcw/view?usp=sharing)

---

### Notes

- The `shk` in file paths refers to the user folder of a *Studentische Hilfskraft* (student assistant) at TU Berlin.
- All code assumes a 2D planar motion (no change in z), with `phi` being the heading angle.
