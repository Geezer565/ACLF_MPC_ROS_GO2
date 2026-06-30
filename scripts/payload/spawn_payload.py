#!/usr/bin/env python3
"""spawn_payload.py — Attach a payload to the Go2 robot in Gazebo.

Spawns a payload SDF model and attaches it to the robot's trunk link
via a fixed joint. The ACLF adaptive controller then estimates and
compensates for the unknown wrench automatically.

Usage:
    python spawn_payload.py brick_5kg      # spawn a 5.43kg brick
    python spawn_payload.py box_21kg       # spawn a 21.6kg box
    python spawn_payload.py none           # remove payload
    python spawn_payload.py --list         # list available presets
    python spawn_payload.py custom --mass 8.0 --com_x 0.1 --com_y 0.0 --com_z 0.15
"""

import os
import sys
import argparse
import tempfile
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

from payload_config import get_preset, list_presets, PayloadConfig

# Robot trunk link to attach payload to
TRUNK_LINK = "trunk"


def build_sdf(config: PayloadConfig) -> str:
    """Build an SDF model string for the payload."""
    m = config.mass_kg
    com = config.com_offset
    I = config.inertia
    size = config.visual_size
    color = config.visual_color

    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="payload">
    <static>false</static>
    <link name="payload_link">
      <inertial>
        <mass>{m}</mass>
        <inertia>
          <ixx>{I[0]}</ixx> <ixy>{I[3]}</ixy> <ixz>{I[4]}</ixz>
          <iyy>{I[1]}</iyy> <iyz>{I[5]}</iyz>
          <izz>{I[2]}</izz>
        </inertia>
        <pose>{com[0]} {com[1]} {com[2]} 0 0 0</pose>
      </inertial>
      <visual name="visual">
        <pose>{com[0]} {com[1]} {com[2]} 0 0 0</pose>
        <geometry>
          <{config.visual_shape}>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </{config.visual_shape}>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <pose>{com[0]} {com[1]} {com[2]} 0 0 0</pose>
        <geometry>
          <{config.visual_shape}>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </{config.visual_shape}>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>'''


def spawn_payload(config: PayloadConfig, robot_ns: str = ""):
    """Spawn the payload model and attach to robot trunk."""
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10.0)

    try:
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to connect to spawn service: {e}")
        return False

    sdf = build_sdf(config)

    # Spawn at origin — will be attached via joint
    pose = Pose()
    pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.3
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation.x = q[0]; pose.orientation.y = q[1]
    pose.orientation.z = q[2]; pose.orientation.w = q[3]

    try:
        spawn("payload", sdf, "", pose, "world")
        rospy.loginfo(f"Spawned payload: {config.name} ({config.mass_kg:.1f} kg)")
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn failed: {e}")
        return False


def remove_payload():
    """Remove the payload model from Gazebo."""
    rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
    try:
        delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete("payload")
        rospy.loginfo("Payload removed.")
    except rospy.ServiceException:
        rospy.logwarn("No payload to remove.")


def main():
    parser = argparse.ArgumentParser(description="Spawn/remove payload on Go2 robot")
    parser.add_argument("preset", nargs="?", default="none",
                        help="Payload preset name (use --list to see all)")
    parser.add_argument("--list", action="store_true", help="List available presets")
    parser.add_argument("--mass", type=float, default=5.0, help="Custom mass [kg]")
    parser.add_argument("--com_x", type=float, default=0.0, help="CoM x offset [m]")
    parser.add_argument("--com_y", type=float, default=0.0, help="CoM y offset [m]")
    parser.add_argument("--com_z", type=float, default=0.15, help="CoM z offset [m]")
    parser.add_argument("--robot_ns", default="", help="Robot namespace (if any)")
    args = parser.parse_args()

    if args.list:
        print("Available payload presets:")
        for name in list_presets():
            preset = get_preset(name)
            print(f"  {name:20s}  mass={preset.mass_kg:5.1f} kg  "
                  f"com=[{preset.com_offset[0]:+.2f}, {preset.com_offset[1]:+.2f}, {preset.com_offset[2]:+.2f}]")
        return

    rospy.init_node("payload_spawner", anonymous=True)

    if args.preset == "none":
        remove_payload()
        return

    if args.preset == "custom":
        config = PayloadConfig(
            name="custom",
            mass_kg=args.mass,
            com_offset=[args.com_x, args.com_y, args.com_z],
        )
    else:
        config = get_preset(args.preset)

    remove_payload()  # Remove old payload first
    rospy.sleep(0.5)
    spawn_payload(config, args.robot_ns)


if __name__ == "__main__":
    main()
