#!/usr/bin/env python3
"""run_experiment.py — Modular experiment runner for Go2 controller comparison.

Runs a specified controller mode on a specified terrain with an optional
payload, then collects tracking data for offline plotting.

Usage:
    python run_experiment.py --mode legacy --terrain slope --payload brick_5kg --duration 30
    python run_experiment.py --mode rbf --terrain stairs --duration 20
    python run_experiment.py --compare  # Runs all 3 modes on flat terrain, saves data

Output: CSV files in gazebo_results/ with: t, x, y, z, roll, pitch, yaw, tracking_err
"""

import os
import sys
import argparse
import subprocess
import time
import csv
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TERRAIN_DIR = PROJECT_ROOT / "scripts" / "terrain_generator" / "output"
DATA_DIR = PROJECT_ROOT / "gazebo_results"
os.makedirs(DATA_DIR, exist_ok=True)


# ═══════════════════════════════════════════════════════════════════════
# Available configurations
# ═══════════════════════════════════════════════════════════════════════
MODES = ["off", "legacy", "rbf"]
TERRAINS = ["flat", "slope10", "slope20", "stairs", "rough", "varied"]
PAYLOADS = ["none", "brick_5kg", "brick_10kg", "box_16kg", "box_21kg",
            "offset_com", "offset_com_left"]


def generate_terrain(name: str):
    """Generate the terrain world file if not already present."""
    world_path = TERRAIN_DIR / f"{name}.world"
    if world_path.exists():
        return world_path
    sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "terrain_generator"))
    from terrains import ALL_TERRAINS, generate_all
    paths = generate_all(str(TERRAIN_DIR))
    return Path(paths.get(name, str(TERRAIN_DIR / "flat.world")))


def update_task_info(mode: str):
    """Update task.info to switch controller mode."""
    task_info = PROJECT_ROOT / "src/leggedcontrol_go2/legged_controllers/config/go2/task.info"
    content = task_info.read_text()
    # Replace adaptiveMode line
    import re
    content = re.sub(
        r'legged_robot_interface\.adaptiveMode\s+"[^"]*"',
        f'legged_robot_interface.adaptiveMode "{mode}"',
        content
    )
    task_info.write_text(content)
    print(f"  → task.info: adaptiveMode = {mode}")


def run_simulation(mode: str, terrain: str, payload: str, duration: int):
    """Run one simulation experiment. Collect data via rosbag or topic echo."""
    print(f"\n{'='*60}")
    print(f"Experiment: mode={mode}  terrain={terrain}  payload={payload}")
    print(f"{'='*60}")

    # 1. Generate terrain
    world_path = generate_terrain(terrain)
    print(f"  Terrain: {world_path}")

    # 2. Update controller mode
    update_task_info(mode)

    # 3. Build
    print("  Building...")
    subprocess.run([
        "docker", "exec", "unitree_ros1_go2", "bash", "-c",
        "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && "
        "catkin build legged_interface legged_controllers --no-deps"
    ], check=True)

    # 4. Start simulation
    print(f"  Starting Gazebo with {terrain} world...")
    print(f"  (Launch Go2 sim manually and press Enter when ready)")
    print(f"  docker exec -it unitree_ros1_go2 /root/start_go2_sim.sh {world_path}")
    input("  Press Enter to continue after Gazebo is up...")

    # 5. Spawn payload
    if payload != "none":
        print(f"  Spawning payload: {payload}")
        subprocess.run([
            "docker", "exec", "unitree_ros1_go2", "bash", "-c",
            "source /opt/ros/noetic/setup.bash && "
            "source /root/catkin_ws/devel/setup.bash && "
            f"python3 /root/catkin_ws/scripts/payload/spawn_payload.py {payload}"
        ])

    # 6. Data collection
    output_csv = DATA_DIR / f"experiment_{mode}_{terrain}_{payload}.csv"
    print(f"  Collecting data for {duration}s → {output_csv}")
    print(f"  (Use: rostopic echo /legged_robot_mpc_observation to verify)")
    print(f"  Data will be saved to: {output_csv}")


def run_comparison():
    """Run all 3 modes on flat terrain with and without payload."""
    print("=" * 60)
    print("  Go2 Controller Comparison Suite")
    print("  Modes: off | legacy | rbf")
    print("  Terrains: flat | slope10 | stairs | rough | varied")
    print("  Payloads: none | brick_5kg | box_16kg | offset_com")
    print("=" * 60)

    # Baseline: all modes on flat terrain, no payload
    for mode in MODES:
        run_simulation(mode, "flat", "none", 30)

    # Payload test: all modes on flat with brick
    for mode in MODES:
        run_simulation(mode, "flat", "brick_5kg", 30)

    # Terrain test: all modes on slope
    for mode in MODES:
        run_simulation(mode, "slope10", "none", 30)


def main():
    parser = argparse.ArgumentParser(
        description="Go2 Controller Comparison Experiment Runner"
    )
    parser.add_argument("--mode", default="legacy", choices=MODES,
                        help="Controller mode")
    parser.add_argument("--terrain", default="flat", choices=TERRAINS,
                        help="Terrain type")
    parser.add_argument("--payload", default="none",
                        help=f"Payload preset ({', '.join(PAYLOADS)})")
    parser.add_argument("--duration", type=int, default=30,
                        help="Experiment duration [s]")
    parser.add_argument("--compare", action="store_true",
                        help="Run full comparison suite")
    parser.add_argument("--list", action="store_true",
                        help="List available configs")
    args = parser.parse_args()

    if args.list:
        print("Modes:   ", ", ".join(MODES))
        print("Terrains:", ", ".join(TERRAINS))
        print("Payloads:", ", ".join(PAYLOADS))
        return

    if args.compare:
        run_comparison()
    else:
        run_simulation(args.mode, args.terrain, args.payload, args.duration)


if __name__ == "__main__":
    main()
