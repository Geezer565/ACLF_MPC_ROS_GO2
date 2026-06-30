#!/usr/bin/env python3
"""gazebo_collect.py — Modular data collector for Go2 simulation experiments.

Each run creates a timestamped subfolder under gazebo_results/:
  exp_{mode}_{terrain}_{payload}/
    ├── raw.bag           # rosbag with all topics
    ├── tracking.csv      # extracted: time, pose, vel, tracking_err
    └── metrics.json      # RMSE, peak, steady-state stats

Usage (inside Docker):
    python3 scripts/gazebo_collect.py --mode legacy --terrain flat --payload none --duration 30
    python3 scripts/gazebo_collect.py --compare   # run full comparison matrix
"""

import os, sys, time, json, argparse, subprocess, csv
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_ROOT = PROJECT_ROOT / "gazebo_results"

MODES = ["off", "legacy", "rbf"]
TERRAINS = ["flat", "slope10", "slope20", "stairs", "rough", "varied"]
PAYLOADS = ["none", "brick_5kg", "brick_10kg", "box_16kg", "box_21kg"]


class DataCollector:
    def __init__(self, mode: str, terrain: str, payload: str):
        self.mode = mode
        self.terrain = terrain
        self.payload = payload
        self.exp_id = f"exp_{mode}_{terrain}_{payload}"
        self.exp_dir = DATA_ROOT / self.exp_id
        self.exp_dir.mkdir(parents=True, exist_ok=True)

    def update_controller_mode(self):
        """Update task.info to use the specified controller mode."""
        task_info = PROJECT_ROOT / "src/leggedcontrol_go2/legged_controllers/config/go2/task.info"
        content = task_info.read_text()
        import re
        content = re.sub(
            r'(legged_robot_interface\.adaptiveMode\s+)"[^"]*"',
            rf'\1"{self.mode}"',
            content
        )
        task_info.write_text(content)

    def build(self):
        subprocess.run([
            "docker", "exec", "unitree_ros1_go2", "bash", "-c",
            "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && "
            "catkin build legged_interface legged_controllers --no-deps"
        ], check=True)

    def start_gazebo(self, world_path: str = None):
        """Start Gazebo with specified world file."""
        if world_path is None:
            # Use flat default
            cmd = "/root/start_go2_sim.sh"
        else:
            cmd = f"/root/start_go2_sim.sh {world_path}"
        subprocess.Popen(["docker", "exec", "-d", "unitree_ros1_go2", "bash", "-c", cmd])

    def spawn_payload(self):
        if self.payload == "none":
            return
        subprocess.run([
            "docker", "exec", "unitree_ros1_go2", "bash", "-c",
            "source /opt/ros/noetic/setup.bash && "
            "source /root/catkin_ws/devel/setup.bash && "
            f"python3 /root/catkin_ws/scripts/payload/spawn_payload.py {self.payload}"
        ], timeout=15)

    def record_bag(self, duration: int) -> Path:
        """Record rosbag for specified duration."""
        bag_path = self.exp_dir / "raw.bag"
        topics = [
            "/legged_robot_mpc_observation",
            "/joint_states",
            "/cmd_vel",
            "/tf",
            "/gazebo/link_states",
        ]
        cmd = (
            "source /opt/ros/noetic/setup.bash && "
            f"rosbag record -O /root/catkin_ws/gazebo_results/{self.exp_id}/raw.bag "
            f"{' '.join(topics)} --duration={duration} --quiet"
        )
        subprocess.run([
            "docker", "exec", "unitree_ros1_go2", "bash", "-c", cmd
        ], timeout=duration + 30)
        return bag_path

    def extract_and_save(self, bag_path: Path):
        """Extract tracking data from bag and save as CSV + metrics JSON."""
        # This runs inside Docker where rosbag tools are available
        extract_script = PROJECT_ROOT / "scripts" / "_extract_bag.py"
        subprocess.run([
            "docker", "exec", "unitree_ros1_go2", "bash", "-c",
            "source /opt/ros/noetic/setup.bash && "
            f"python3 /root/catkin_ws/scripts/_extract_bag.py "
            f"/root/catkin_ws/gazebo_results/{self.exp_id}/raw.bag "
            f"/root/catkin_ws/gazebo_results/{self.exp_id}"
        ], timeout=60)

    def cleanup(self):
        """Kill Gazebo processes."""
        subprocess.run([
            "docker", "exec", "unitree_ros1_go2", "bash", "-c",
            "pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null; "
            "pkill -9 rosmaster 2>/dev/null; sleep 2"
        ], timeout=10)


# ═══════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Go2 Data Collector")
    parser.add_argument("--mode", default="legacy", choices=MODES)
    parser.add_argument("--terrain", default="flat", choices=TERRAINS)
    parser.add_argument("--payload", default="none")
    parser.add_argument("--duration", type=int, default=30, help="seconds")
    parser.add_argument("--compare", action="store_true",
                        help="Run all 3 modes on flat+payload")
    parser.add_argument("--list", action="store_true",
                        help="List past experiments")
    args = parser.parse_args()

    if args.list:
        print("\nPast experiments in gazebo_results/:\n")
        for d in sorted(DATA_ROOT.iterdir()):
            if d.is_dir() and d.name.startswith("exp_"):
                csv_file = d / "tracking.csv"
                metrics_file = d / "metrics.json"
                size = sum(f.stat().st_size for f in d.rglob("*")) / 1e6
                has_csv = "✓" if csv_file.exists() else "✗"
                has_metrics = "✓" if metrics_file.exists() else "✗"
                print(f"  {d.name:35s}  {size:5.1f}MB  csv:{has_csv}  metrics:{has_metrics}")
        print(f"\nTotal: {sum(1 for d in DATA_ROOT.iterdir() if d.is_dir() and d.name.startswith('exp_'))} experiments")
        return

    if args.compare:
        print("=" * 50)
        print("  Full Comparison Run")
        print("=" * 50)
        for mode in MODES:
            for payload in ["none", "brick_5kg"]:
                collector = DataCollector(mode, "flat", payload)
                print(f"\n▶ {collector.exp_id}")
                collector.update_controller_mode()
                collector.build()
                collector.start_gazebo()
                time.sleep(10)
                collector.spawn_payload()
                collector.record_bag(args.duration)
                collector.cleanup()
                bag = collector.exp_dir / "raw.bag"
                if bag.exists():
                    collector.extract_and_save(bag)
    else:
        collector = DataCollector(args.mode, args.terrain, args.payload)
        print(f"\n▶ {collector.exp_id}")
        print(f"  Data will be saved to: {collector.exp_dir}")
        print(f"\n  Manual steps:")
        print(f"  1. Start Gazebo with terrain: {args.terrain}")
        print(f"  2. Spawn payload: {args.payload}")
        print(f"  3. Record bag for {args.duration}s:")
        print(f"     rosbag record -O {collector.exp_dir}/raw.bag \\")
        print(f"       /legged_robot_mpc_observation /joint_states /cmd_vel \\")
        print(f"       --duration={args.duration}")
        print(f"  4. Extract data:")
        print(f"     python3 scripts/_extract_bag.py {collector.exp_dir}/raw.bag {collector.exp_dir}")
        print(f"  5. Plot: python3 scripts/gazebo_plot.py")


if __name__ == "__main__":
    main()
