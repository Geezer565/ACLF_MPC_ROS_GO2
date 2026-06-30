#!/usr/bin/env python3
"""
Process Gazebo rosbag and generate tracking metrics + figures for the RS-RBF paper.

Outputs:
  - gazebo_tracking.csv        : time, x_ref, y_ref, x_actual, y_actual, tracking_error
  - gazebo_path_tracking.png   : XY path comparison plot
  - gazebo_metrics.txt         : mean/peak tracking error, loop rate
"""
import rosbag
import numpy as np
import sys
import os
import csv

# Try to import plotting; skip if not available
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def lemniscate_position(t, a=1.0, b=0.65, T_ref=16.0):
    """Figure-eight reference position."""
    omega = 2.0 * np.pi / T_ref
    x = a * np.sin(omega * t)
    y = b * np.sin(omega * t) * np.cos(omega * t)
    return x, y


def extract_data(bagfile, outdir):
    """Extract tracking data from rosbag."""
    bag = rosbag.Bag(bagfile)

    # --- Extract base positions from /gazebo/link_states ---
    base_times = []
    base_x, base_y, base_z = [], [], []

    for topic, msg, t in bag.read_messages(topics=['/gazebo/link_states']):
        ts = t.to_sec()
        try:
            idx = msg.name.index('go2::base')
        except ValueError:
            # Try 'base' without prefix
            try:
                idx = msg.name.index('base')
            except ValueError:
                continue

        pose = msg.pose[idx]
        base_times.append(ts)
        base_x.append(pose.position.x)
        base_y.append(pose.position.y)
        base_z.append(pose.position.z)

    if not base_times:
        print("ERROR: No base position data found in /gazebo/link_states!")
        print("Available link names in bag:")
        for topic, msg, t in bag.read_messages(topics=['/gazebo/link_states']):
            print(f"  {msg.name}")
            break
        bag.close()
        return None

    base_times = np.array(base_times)
    base_x = np.array(base_x)
    base_y = np.array(base_y)
    base_z = np.array(base_z)

    # Normalize time to start from 0
    t0 = base_times[0]
    base_times -= t0

    # --- Compute reference positions at base timestamps ---
    # Using SMALL parameters matching the Gazebo STANCE test:
    # a=0.12m, b=0.08m — the robot cant walk, only shift CoM in stance
    ref_x, ref_y = lemniscate_position(base_times, a=0.12, b=0.08, T_ref=16.0)

    # Offset: align first reference point to first actual position
    ref_x += base_x[0] - ref_x[0]
    ref_y += base_y[0] - ref_y[0]

    # --- Compute tracking error ---
    track_err = np.sqrt((base_x - ref_x)**2 + (base_y - ref_y)**2)

    # --- Metrics ---
    mean_err = np.mean(track_err)
    peak_err = np.max(track_err)
    steady_err = np.mean(track_err[-int(len(track_err)*0.5):])  # last 50%

    # Compute achieved control rate
    dt = np.diff(base_times)
    mean_dt = np.mean(dt)
    achieved_rate = 1.0 / mean_dt if mean_dt > 0 else 0

    metrics = {
        'mean_tracking_error_m': mean_err,
        'peak_tracking_error_m': peak_err,
        'steady_tracking_error_m': steady_err,
        'mean_loop_time_ms': mean_dt * 1000,
        'achieved_rate_hz': achieved_rate,
        'data_points': len(base_times),
        'duration_s': base_times[-1] - base_times[0],
    }

    # --- Save CSV ---
    csv_path = os.path.join(outdir, 'gazebo_tracking.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'x_ref', 'y_ref', 'x_actual', 'y_actual', 'tracking_error'])
        for i in range(len(base_times)):
            writer.writerow([base_times[i], ref_x[i], ref_y[i],
                             base_x[i], base_y[i], track_err[i]])
    print(f"  CSV saved: {csv_path} ({len(base_times)} points)")

    # --- Save metrics ---
    metrics_path = os.path.join(outdir, 'gazebo_metrics.txt')
    with open(metrics_path, 'w') as f:
        f.write("RS-RBF Gazebo Tracking Metrics\n")
        f.write("=" * 50 + "\n")
        f.write(f"Duration:              {metrics['duration_s']:.1f} s\n")
        f.write(f"Data points:           {metrics['data_points']}\n")
        f.write(f"Mean loop time:        {metrics['mean_loop_time_ms']:.1f} ms\n")
        f.write(f"Achieved rate:         {metrics['achieved_rate_hz']:.1f} Hz\n")
        f.write(f"Mean tracking error:   {metrics['mean_tracking_error_m']*1000:.1f} mm\n")
        f.write(f"Peak tracking error:   {metrics['peak_tracking_error_m']*1000:.1f} mm\n")
        f.write(f"Steady error (50%):    {metrics['steady_tracking_error_m']*1000:.1f} mm\n")
    print(f"  Metrics saved: {metrics_path}")

    # --- Generate figures ---
    if HAS_MPL:
        fig_path = os.path.join(outdir, 'gazebo_path_tracking.png')
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

        # XY path
        ax1.plot(ref_x, ref_y, 'k--', linewidth=1.5, label='Reference')
        ax1.plot(base_x, base_y, 'b-', linewidth=1.5, label='Actual (RS-RBF)')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('Gazebo Tracking: XY Path')
        ax1.legend()
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)

        # Tracking error over time
        ax2.plot(base_times, track_err * 1000, 'b-', linewidth=1.0)
        ax2.axhline(y=mean_err * 1000, color='r', linestyle='--',
                    label=f'Mean: {mean_err*1000:.1f} mm')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Tracking Error [mm]')
        ax2.set_title('Gazebo Tracking: Planar Error')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        fig.savefig(fig_path, dpi=150)
        plt.close()
        print(f"  Figure saved: {fig_path}")

    bag.close()
    return metrics


def main():
    bagfile = '/root/catkin_ws/gazebo_results/gazebo_tracking.bag'
    outdir = '/root/catkin_ws/gazebo_results'

    if not os.path.exists(bagfile):
        print(f"ERROR: Bag file not found: {bagfile}")
        sys.exit(1)

    print("=" * 50)
    print("  Processing Gazebo rosbag data")
    print("=" * 50)

    metrics = extract_data(bagfile, outdir)

    if metrics:
        print("\n" + "=" * 50)
        print("  SUMMARY")
        print("=" * 50)
        print(f"  Mean tracking error:  {metrics['mean_tracking_error_m']*1000:.1f} mm")
        print(f"  Peak tracking error:  {metrics['peak_tracking_error_m']*1000:.1f} mm")
        print(f"  Achieved loop rate:   {metrics['achieved_rate_hz']:.1f} Hz")
        print("=" * 50)


if __name__ == '__main__':
    main()
