import os
import numpy as np
from pathlib import Path
from datetime import datetime, timedelta
import argparse

def convert_unix_to_kitti_timestamp(unix_ts):
    dt = datetime.utcfromtimestamp(unix_ts)
    return dt.strftime("%Y-%m-%d %H:%M:%S.") + f"{dt.microsecond:06d}"

def create_kitti_structure(input_dir, output_dir, drive_date="2025_07_01", drive_id="0001"):
    input_dir = Path(input_dir)
    output_base = Path(output_dir) / drive_date / f"{drive_date}_drive_{drive_id}_sync"
    livox_dir = output_base / "livox_points" / "data"
    livox_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(input_dir.glob("*pc.txt"))
    end_unix_list = [int(f.stem.replace("pc", "")) for f in files]

    if len(end_unix_list) > 1:
        deltas = [
            end_unix_list[i + 1] - end_unix_list[i]
            for i in range(len(end_unix_list) - 1)
        ]
        avg_delta_sec = sum(deltas) / len(deltas)
    else:
        avg_delta_sec = 0.1  # 100 ms fallback

    timestamps = []
    timestamps_start = []
    timestamps_end = []

    # Prepare for oxts
    oxts_dir = output_base / "oxts" / "data"
    oxts_dir.mkdir(parents=True, exist_ok=True)
    odom_timestamps = []

    prev_end_unix = end_unix_list[0] - avg_delta_sec

    for i, file in enumerate(files):
        end_unix = end_unix_list[i]
        end_time = datetime.utcfromtimestamp(end_unix)
        start_time = datetime.utcfromtimestamp(prev_end_unix)
        midpoint = start_time + (end_time - start_time) / 2

        data = np.loadtxt(file)
        if data.ndim == 1:
            data = data[np.newaxis, :]

        # Save LiDAR points excluding first row (which is odometry)
        lidar_points = data[1:]
        bin_filename = f"{i:010d}.bin"
        bin_path = livox_dir / bin_filename
        lidar_points.astype(np.float32).tofile(bin_path)

        # Save odometry (first row: x, y, phi)
        odom_data = data[0]
        odom_filename = f"{i:010d}.txt"
        odom_path = oxts_dir / odom_filename
        odom_line = f"{odom_data[0]} {odom_data[1]} {odom_data[2]}"
        with open(odom_path, "w") as f:
            f.write(odom_line + "\n")

        # Collect timestamps for all files
        timestamps_start.append(start_time.strftime("%Y-%m-%d %H:%M:%S.") + f"{start_time.microsecond:06d}")
        timestamps_end.append(end_time.strftime("%Y-%m-%d %H:%M:%S.") + f"{end_time.microsecond:06d}")
        timestamps.append(midpoint.strftime("%Y-%m-%d %H:%M:%S.") + f"{midpoint.microsecond:06d}")
        odom_timestamps.append(midpoint.strftime("%Y-%m-%d %H:%M:%S.") + f"{midpoint.microsecond:06d}")

        prev_end_unix = end_unix

    def write_list_to_file(lines, filepath):
        with open(filepath, "w") as f:
            for line in lines:
                f.write(line + "\n")

    # Write LiDAR timestamps
    vp_path = output_base / "livox_points"
    write_list_to_file(timestamps, vp_path / "timestamps.txt")
    write_list_to_file(timestamps_start, vp_path / "timestamps_start.txt")
    write_list_to_file(timestamps_end, vp_path / "timestamps_end.txt")

    # Write oxts timestamps
    oxts_path = output_base / "oxts"
    write_list_to_file(odom_timestamps, oxts_path / "timestamps.txt")

    # Write oxts dataformat.txt
    dataformat_text = (
        "x:        x position in meters (vehicle frame)\n"
        "y:        y position in meters (vehicle frame)\n"
        "phi:      rotation angle around z axis (radians)\n"
    )
    with open(oxts_path / "dataformat.txt", "w") as f:
        f.write(dataformat_text)

    print(f"KITTI-style conversion complete.\nData saved to: {output_base}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Livox data to KITTI-style format.")
    parser.add_argument("--input_dir", type=str, required=True, help="Path to input folder containing *pc.txt files.")
    parser.add_argument("--output_dir", type=str, required=True, help="Path to output base folder.")
    parser.add_argument("--drive_date", type=str, default="2025_07_01", help="Date for the drive folder (default: 2025_07_01).")
    parser.add_argument("--drive_id", type=str, default="0001", help="Drive ID (default: 0001).")

    args = parser.parse_args()

    create_kitti_structure(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        drive_date=args.drive_date,
        drive_id=args.drive_id
    )
