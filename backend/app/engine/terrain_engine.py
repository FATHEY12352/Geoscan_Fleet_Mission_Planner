"""
Synthetic 3D Terrain Engine for Geoscan Simulator
Generates mock elevation (Z-axis) based on mathematical noise (Perlin-like logic).
"""

import math
from typing import List, Tuple

def get_elevation(x: float, y: float) -> float:
    """
    Generates a synthetic terrain elevation (Z in meters) for a given X, Y coordinate.
    Uses overlapping sine waves to simulate hills and valleys.
    """
    # Base frequency for hills (large scale)
    freq1 = 0.001
    hill_z = math.sin(x * freq1) * math.cos(y * freq1) * 100.0
    
    # Secondary frequency for bumps (small scale)
    freq2 = 0.005
    bump_z = math.sin(x * freq2 + 10) * math.sin(y * freq2 + 20) * 15.0
    
    # Base ground level at 50m
    z = 50.0 + hill_z + bump_z
    
    # Ensure elevation never drops below 0
    return max(0.0, z)

def apply_terrain_to_waypoints(waypoints: List[Tuple[float, float]], agl_m: float) -> List[Tuple[float, float, float]]:
    """
    Takes 2D (X, Y) waypoints and converts them to 3D (X, Y, Z) waypoints 
    where Z = terrain_elevation + agl_m (Above Ground Level).
    """
    waypoint_3d = []
    for (x, y) in waypoints:
        terrain_z = get_elevation(x, y)
        flight_z = terrain_z + agl_m
        waypoint_3d.append((x, y, flight_z))
    return waypoint_3d
