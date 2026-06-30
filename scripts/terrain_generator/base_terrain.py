"""BaseTerrain — Abstract terrain interface + SDF world builder.

All terrain types inherit from this and only need to implement height_at(x,y).
"""

import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class WorldConfig:
    """Gazebo world physics / visual settings."""
    name: str = "terrain_world"
    gravity: float = -9.81
    ode_step_size: float = 0.001
    ode_update_rate: float = 1000
    ground_size: float = 40.0          # total ground plane (m)
    terrain_mesh_resolution: float = 0.05  # grid resolution (m)
    terrain_color: str = "0.6 0.6 0.6 1.0"  # RGBA


class BaseTerrain(ABC):
    """Abstract terrain.

    Subclasses must provide height_at(x, y) → float.
    """

    def __init__(self, config: WorldConfig = None):
        self.config = config or WorldConfig()

    # ── Abstract ──────────────────────────────────────────────────────
    @abstractmethod
    def height_at(self, x: float, y: float) -> float:
        """Return terrain height at world coordinates (x, y)."""
        ...

    @abstractmethod
    def name(self) -> str:
        """Human-readable terrain name (used in file name)."""
        ...

    # ── SDF generation ────────────────────────────────────────────────
    def generate_world(self, output_dir: str = None) -> str:
        """Generate a complete SDF world file with this terrain as heightmap.

        Returns the path to the generated .world file.
        """
        cfg = self.config
        output_dir = output_dir or os.path.join(os.path.dirname(__file__), "output")
        os.makedirs(output_dir, exist_ok=True)

        # Generate heightmap image
        hmap_path = self._write_heightmap(output_dir)

        world_text = self._sdf_template(hmap_path)
        world_path = os.path.join(output_dir, f"{self.name()}.world")
        with open(world_path, "w") as f:
            f.write(world_text)
        return world_path

    # ── Heightmap generation ──────────────────────────────────────────
    def _write_heightmap(self, output_dir: str) -> str:
        """Build a grayscale heightmap PNG and return the path."""
        import numpy as np
        from PIL import Image

        cfg = self.config
        res = cfg.terrain_mesh_resolution
        half = cfg.ground_size / 2
        pixels = int(cfg.ground_size / res)

        # Sample heights on a grid
        heights = np.zeros((pixels, pixels), dtype=np.float32)
        for i in range(pixels):
            y = half - i * res
            for j in range(pixels):
                x = j * res - half
                heights[i, j] = self.height_at(x, y)

        # Normalize to [0, 255] (reserve a small floor above zero)
        h_min = heights.min()
        h_max = max(heights.max(), h_min + 0.01)
        grey = ((heights - h_min) / (h_max - h_min) * 255).astype(np.uint8)

        img = Image.fromarray(grey, mode="L")
        path = os.path.join(output_dir, f"{self.name()}_heightmap.png")
        img.save(path)

        # Write metadata (Gazebo needs size info)
        self._h_min = float(h_min)
        self._h_max = float(h_max)
        return path

    # ── SDF template ──────────────────────────────────────────────────
    def _sdf_template(self, heightmap_path: str) -> str:
        cfg = self.config
        hmin = getattr(self, "_h_min", 0.0)
        hmax = getattr(self, "_h_max", 1.0)
        size = cfg.ground_size
        hmap_uri = os.path.abspath(heightmap_path)

        return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{cfg.name}">
    <physics type="ode">
      <max_step_size>{cfg.ode_step_size}</max_step_size>
      <real_time_update_rate>{cfg.ode_update_rate}</real_time_update_rate>
      <gravity>0 0 {cfg.gravity}</gravity>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
      <shadows>1</shadows>
    </scene>

    <include><uri>model://sun</uri></include>

    <!-- Heightmap terrain -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{hmap_uri}</uri>
              <size>{size} {size} {hmax - hmin}</size>
              <pos>0 0 {hmin}</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://{hmap_uri}</uri>
              <size>{size} {size} {hmax - hmin}</size>
              <pos>0 0 {hmin}</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>{cfg.terrain_color}</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>'''
