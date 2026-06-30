"""Concrete terrain types — each is a self-contained module.

Usage:
    from terrain_generator.terrains import Flat, Slope, Stairs, Rough, Varied
    world_path = Slope(angle_deg=15).generate_world()
"""

import math
import numpy as np
from .base_terrain import BaseTerrain, WorldConfig


# ═══════════════════════════════════════════════════════════════════════
# Flat (baseline)
# ═══════════════════════════════════════════════════════════════════════
class Flat(BaseTerrain):
    """Perfectly flat ground — baseline for all comparisons."""

    def __init__(self, config: WorldConfig = None):
        super().__init__(config)

    def height_at(self, x: float, y: float) -> float:
        return 0.0

    def name(self) -> str: return "flat"


# ═══════════════════════════════════════════════════════════════════════
# Slope (configurable angle)
# ═══════════════════════════════════════════════════════════════════════
@dataclass
class SlopeConfig(WorldConfig):
    angle_deg: float = 15.0           # incline angle
    slope_direction_deg: float = 0.0  # 0 = slope in +x direction


class Slope(BaseTerrain):
    """Inclined plane at configurable angle."""

    def __init__(self, angle_deg: float = 15.0, direction_deg: float = 0.0,
                 config: WorldConfig = None):
        super().__init__(config)
        self.angle = math.radians(angle_deg)
        self.direction = math.radians(direction_deg)

    def height_at(self, x: float, y: float) -> float:
        # Project (x,y) onto slope direction
        dist = x * math.cos(self.direction) + y * math.sin(self.direction)
        return dist * math.tan(self.angle)

    def name(self) -> str: return f"slope_{int(math.degrees(self.angle))}deg"


# ═══════════════════════════════════════════════════════════════════════
# Stairs (configurable step dimensions)
# ═══════════════════════════════════════════════════════════════════════
@dataclass
class StairsConfig(WorldConfig):
    step_height: float = 0.04         # riser [m]
    step_depth: float = 0.20          # tread [m]
    n_steps: int = 10
    start_x: float = 0.0              # where stairs begin


class Stairs(BaseTerrain):
    """Ascending stairs starting at start_x, extending in +x direction."""

    def __init__(self, step_height: float = 0.04, step_depth: float = 0.20,
                 n_steps: int = 10, start_x: float = 0.0,
                 config: WorldConfig = None):
        super().__init__(config)
        self.step_h = step_height
        self.step_d = step_depth
        self.n = n_steps
        self.x0 = start_x

    def height_at(self, x: float, y: float) -> float:
        if x < self.x0:
            return 0.0
        step_idx = int((x - self.x0) / self.step_d)
        step_idx = max(0, min(step_idx, self.n - 1))
        # Within-step interpolation (smooth ramp)
        x_in_step = (x - self.x0) - step_idx * self.step_d
        if x_in_step < 0.02:  # riser smoothing
            return step_idx * self.step_h
        elif x_in_step > self.step_d - 0.02:
            return (step_idx + 1) * self.step_h
        return (step_idx + x_in_step / self.step_d) * self.step_h

    def name(self) -> str:
        return f"stairs_h{int(self.step_h*1000)}_d{int(self.step_d*100)}"


# ═══════════════════════════════════════════════════════════════════════
# Rough (random undulating terrain)
# ═══════════════════════════════════════════════════════════════════════
class Rough(BaseTerrain):
    """Random rough terrain using summed sinusoids (no sharp edges)."""

    def __init__(self, amplitude: float = 0.03, frequency: float = 2.0,
                 seed: int = 42, config: WorldConfig = None):
        super().__init__(config)
        self.amplitude = amplitude
        self.freq = frequency
        self.seed = seed

    def height_at(self, x: float, y: float) -> float:
        rng = np.random.RandomState(self.seed + int(x * 100) % 1000 + int(y * 1000) % 10000)
        h = 0.0
        for k in range(1, 5):
            phase_x = rng.uniform(0, 2 * math.pi)
            phase_y = rng.uniform(0, 2 * math.pi)
            h += (self.amplitude / k) * (
                math.sin(self.freq * k * x + phase_x) +
                math.cos(self.freq * k * y + phase_y)
            )
        return h

    def name(self) -> str:
        return f"rough_a{int(self.amplitude*1000)}"


# ═══════════════════════════════════════════════════════════════════════
# Varied (multi-segment composite terrain)
# ═══════════════════════════════════════════════════════════════════════
class Varied(BaseTerrain):
    """Composite terrain with multiple segments — e.g. flat → slope → rough."""

    def __init__(self, segments: list = None, config: WorldConfig = None):
        """
        segments: list of (terrain_obj, x_start, x_end) tuples
        """
        super().__init__(config)
        self.segments = segments or [
            (Flat(), -20, -5),
            (Slope(angle_deg=10), -5, 2),
            (Rough(amplitude=0.02), 2, 20),
        ]

    def height_at(self, x: float, y: float) -> float:
        for terrain, x0, x1 in self.segments:
            if x0 <= x <= x1:
                return terrain.height_at(x, y)
        return 0.0  # outside all segments

    def name(self) -> str: return "varied"


# ═══════════════════════════════════════════════════════════════════════
# Convenience: generate all terrains at once
# ═══════════════════════════════════════════════════════════════════════
ALL_TERRAINS = {
    "flat":    Flat(),
    "slope10": Slope(angle_deg=10),
    "slope20": Slope(angle_deg=20),
    "stairs":  Stairs(step_height=0.04, step_depth=0.20, n_steps=12),
    "rough":   Rough(amplitude=0.03, frequency=2.0),
    "varied":  Varied(),
}


def generate_all(output_dir: str = None):
    """Generate all terrain world files."""
    paths = {}
    for key, terrain in ALL_TERRAINS.items():
        print(f"Generating {key}...")
        paths[key] = terrain.generate_world(output_dir)
    return paths


if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "output"
    generate_all(out)
