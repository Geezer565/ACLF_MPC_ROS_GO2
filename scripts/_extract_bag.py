#!/usr/bin/env python3
"""_extract_bag.py — Extract tracking data from rosbag to CSV + metrics JSON.

This runs inside Docker (where rosbag is available).

Usage:
    python3 _extract_bag.py raw.bag output_dir/
"""

import sys, os, json, csv
import rosbag
import numpy as np


def extract(bagfile: str, outdir: str):
    bag = rosbag.Bag(bagfile)
    out = os.path.join(outdir, "tracking.csv")
    metrics_path = os.path.join(outdir, "metrics.json")

    # --- 1. Extract base pose from /gazebo/link_states ---
    rows = {}  # time → dict
    for topic, msg, t in bag.read_messages(topics=['/gazebo/link_states']):
        ts = t.to_sec()
        try:
            idx = msg.name.index('go2::base')
        except ValueError:
            try:
                idx = msg.name.index('base')
            except ValueError:
                continue
        pose = msg.pose[idx]
        rows.setdefault(ts, {}).update({
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w,
        })

    # --- 2. Extract joint states ---
    for topic, msg, t in bag.read_messages(topics=['/joint_states']):
        ts = t.to_sec()
        if ts in rows:
            for i, name in enumerate(msg.name):
                rows[ts][f'j_{name}'] = msg.position[i] if i < len(msg.position) else 0.0

    # --- 3. Extract cmd_vel ---
    for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
        ts = t.to_sec()
        if ts in rows:
            rows[ts].update({
                'cmd_vx': msg.linear.x,
                'cmd_vy': msg.linear.y,
                'cmd_wz': msg.angular.z,
            })

    bag.close()

    # --- Sort and normalize time ---
    times = sorted(rows.keys())
    if not times:
        print("ERROR: No data extracted!")
        return None

    t0 = times[0]
    data = [(t - t0, rows[t]) for t in times]

    # --- Write CSV ---
    # Gather all possible keys
    all_keys = set()
    for _, d in data:
        all_keys.update(d.keys())
    all_keys = sorted(all_keys)

    with open(out, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['time'] + all_keys
        writer.writerow(header)
        for t, d in data:
            row = [t] + [d.get(k, float('nan')) for k in all_keys]
            writer.writerow(row)

    print(f"  CSV: {out} ({len(data)} rows, {len(all_keys)} columns)")

    # --- Compute metrics ---
    has_pos = all(k in all_keys for k in ['x', 'y', 'z'])
    if has_pos:
        xs = np.array([d.get('x', np.nan) for _, d in data])
        ys = np.array([d.get('y', np.nan) for _, d in data])
        zs = np.array([d.get('z', np.nan) for _, d in data])

        # Position errors (deviation from mean position during tracking)
        x_std = np.nanstd(xs)
        y_std = np.nanstd(ys)
        z_std = np.nanstd(zs)

        # Velocity (finite differences)
        dts = np.diff([t for t, _ in data])
        vx = np.diff(xs) / dts
        vy = np.diff(ys) / dts

        metrics = {
            'n_points': len(data),
            'duration_s': float(data[-1][0]),
            'position_std_m': {'x': float(x_std), 'y': float(y_std), 'z': float(z_std)},
            'mean_speed_ms': float(np.nanmean(np.sqrt(vx**2 + vy**2))),
            'control_rate_hz': float(1.0 / np.mean(dts)) if len(dts) > 0 else 0,
        }
    else:
        metrics = {'n_points': len(data), 'duration_s': float(data[-1][0])}

    with open(metrics_path, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f"  Metrics: {metrics_path}")

    return metrics


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 _extract_bag.py <bagfile> <output_dir>")
        sys.exit(1)
    extract(sys.argv[1], sys.argv[2])
