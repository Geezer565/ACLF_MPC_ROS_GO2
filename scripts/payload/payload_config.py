"""PayloadConfig — Modular payload definition for Go2 quadruped.

Each payload is defined by: mass, CoM offset, inertia, and visual geometry.
The adaptive controller (ACLF / RBF) estimates these parameters online
to compensate for the disturbance they create.

Ref: Minniti et al. 2021 — payloads of 5.43kg (brick) to 21.6kg (box)
"""

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class PayloadConfig:
    """Single payload definition."""

    name: str                           # e.g. "brick_5kg"
    mass_kg: float                      # additional mass
    com_offset: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.1])
    # CoM offset from trunk center [x, y, z] in base frame (m)
    inertia: List[float] = field(default_factory=lambda: [0.01, 0.01, 0.01, 0, 0, 0])
    # [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] in kg·m²

    # Visual (for Gazebo)
    visual_shape: str = "box"           # box | cylinder | sphere | mesh
    visual_size: List[float] = field(default_factory=lambda: [0.2, 0.15, 0.1])
    # [x, y, z] dimensions (m)
    visual_color: str = "1.0 0.5 0.0 1.0"  # RGBA

    # Dynamics
    vibrate: bool = False               # if True, applies oscillating force


# ═══════════════════════════════════════════════════════════════════════
# Presets — matching ACLF paper experiments
# ═══════════════════════════════════════════════════════════════════════
PRESETS = {
    # Paper A experiments
    "brick_5kg": PayloadConfig(
        name="brick_5kg",
        mass_kg=5.43,
        com_offset=[0.0, 0.0, 0.15],
        inertia=[0.015, 0.01, 0.01, 0, 0, 0],
        visual_shape="box",
        visual_size=[0.2, 0.15, 0.1],
        visual_color="0.8 0.3 0.1 1.0",
    ),

    "brick_10kg": PayloadConfig(
        name="brick_10kg",
        mass_kg=10.86,
        com_offset=[0.0, 0.0, 0.15],
        inertia=[0.03, 0.02, 0.02, 0, 0, 0],
        visual_shape="box",
        visual_size=[0.25, 0.18, 0.12],
        visual_color="0.8 0.3 0.1 1.0",
    ),

    "box_16kg": PayloadConfig(
        name="box_16kg",
        mass_kg=16.17,
        com_offset=[0.05, 0.0, 0.20],
        inertia=[0.08, 0.06, 0.06, 0, 0, 0],
        visual_shape="box",
        visual_size=[0.35, 0.25, 0.20],
        visual_color="0.6 0.4 0.2 1.0",
    ),

    "box_21kg": PayloadConfig(
        name="box_21kg",
        mass_kg=21.6,
        com_offset=[0.05, 0.0, 0.20],
        inertia=[0.10, 0.08, 0.08, 0, 0, 0],
        visual_shape="box",
        visual_size=[0.40, 0.28, 0.22],
        visual_color="0.6 0.4 0.2 1.0",
    ),

    # Controlled experiments (for comparison)
    "offset_com": PayloadConfig(
        name="offset_com",
        mass_kg=5.0,
        com_offset=[0.3, 0.0, 0.0],  # 30cm forward shift
        inertia=[0.005, 0.005, 0.005, 0, 0, 0],
        visual_shape="sphere",
        visual_size=[0.08, 0.08, 0.08],
        visual_color="0.2 0.6 1.0 1.0",
    ),

    "offset_com_left": PayloadConfig(
        name="offset_com_left",
        mass_kg=5.0,
        com_offset=[0.0, 0.15, 0.0],  # 15cm left shift
        inertia=[0.005, 0.005, 0.005, 0, 0, 0],
        visual_shape="sphere",
        visual_size=[0.08, 0.08, 0.08],
        visual_color="0.2 1.0 0.6 1.0",
    ),

    # No payload (baseline)
    "none": PayloadConfig(
        name="none",
        mass_kg=0.0,
        com_offset=[0.0, 0.0, 0.0],
    ),
}


def get_preset(name: str) -> PayloadConfig:
    """Get a payload preset by name. Use 'none' for baseline."""
    if name not in PRESETS:
        raise KeyError(f"Unknown payload preset: {name}. Available: {list(PRESETS.keys())}")
    return PRESETS[name]


def list_presets() -> List[str]:
    return list(PRESETS.keys())
