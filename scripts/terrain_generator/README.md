# Terrain Generator

Modular Gazebo terrain generation for Go2 quadruped simulation.

## Usage

```bash
# Install deps (host machine)
pip install numpy pillow

# Generate all terrains
python scripts/terrain_generator/terrains.py output_dir/

# Or from Python
from terrain_generator.terrains import Slope, Stairs, generate_all
world_path = Slope(angle_deg=15).generate_world()
```

## Available Terrains

| Name | Class | Config |
|------|-------|--------|
| `flat` | `Flat()` | Baseline |
| `slope` | `Slope(angle_deg=15)` | 10°-30° incline |
| `stairs` | `Stairs(h=0.04, d=0.20, n=12)` | Step height/depth/count |
| `rough` | `Rough(amp=0.03, freq=2.0)` | Random undulating |
| `varied` | `Varied(segments=[...])` | Composite multi-segment |

## Adding a New Terrain

```python
from base_terrain import BaseTerrain

class MyTerrain(BaseTerrain):
    def height_at(self, x, y):
        return x * 0.1  # simple ramp
    def name(self):
        return "my_terrain"
```
