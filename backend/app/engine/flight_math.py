"""
Photogrammetry and Geometric Coverage Path Planning (CPP) Math
Calculates GSD, flight altitudes, line spacing (межгалсовое расстояние),
and generates rotated lawnmower flight strips.
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Any
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely.affinity import rotate, translate

def calculate_flight_parameters(
    target_gsd_cm: float,
    sensor_spec: Dict[str, Any]
) -> Dict[str, float]:
    """
    Computes required flight altitude, footprint, and strip spacing based on sensor and GSD.
    """
    sensor_type = sensor_spec.get("type", "rgb")
    
    if sensor_type == "lidar":
        # LiDAR has recommended flight altitude and swath width factor
        alt_m = sensor_spec.get("optimal_altitude_m", 80.0)
        swath_width = alt_m * sensor_spec.get("swath_width_factor", 1.2)
        side_overlap = sensor_spec.get("recommended_overlap_side", 0.50)
        line_spacing_m = swath_width * (1.0 - side_overlap)
        return {
            "altitude_m": alt_m,
            "line_spacing_m": max(15.0, line_spacing_m),
            "swath_width_m": swath_width,
            "gsd_cm": 0.0 # LiDAR measures point density, not GSD
        }
    
    if sensor_type == "geophysical":
        alt_m = sensor_spec.get("optimal_altitude_m", 30.0)
        line_spacing_m = 50.0 # Standard parallel survey profiles
        return {
            "altitude_m": alt_m,
            "line_spacing_m": line_spacing_m,
            "swath_width_m": line_spacing_m,
            "gsd_cm": 0.0
        }
    
    # Standard Optical (RGB, Multispectral, Thermal)
    focal_length_mm = sensor_spec.get("focal_length_mm", 35.0)
    pixel_size_um = sensor_spec.get("pixel_size_um", 4.51)
    sensor_width_mm = sensor_spec.get("sensor_width_mm", 35.9)
    sensor_height_mm = sensor_spec.get("sensor_height_mm", 24.0)
    side_overlap = sensor_spec.get("recommended_overlap_side", 0.65)
    
    # H = (GSD * f) / pixel_size
    # GSD in m/px, f in mm, pixel_size in um -> H in meters:
    # H = (GSD_cm * 0.01 * focal_length_mm * 1e-3) / (pixel_size_um * 1e-6)
    alt_m = (target_gsd_cm * 0.01 * (focal_length_mm * 1e-3)) / (pixel_size_um * 1e-6)
    alt_m = max(30.0, min(alt_m, 350.0)) # Practical safety limits
    
    ground_width_m = alt_m * (sensor_width_mm / focal_length_mm)
    ground_height_m = alt_m * (sensor_height_mm / focal_length_mm)
    
    line_spacing_m = ground_width_m * (1.0 - side_overlap)
    
    forward_overlap = sensor_spec.get("recommended_overlap_forward", 0.75)
    trigger_dist_m = max(5.0, ground_height_m * (1.0 - forward_overlap))
    
    return {
        "altitude_m": round(alt_m, 1),
        "line_spacing_m": round(max(10.0, line_spacing_m), 1),
        "ground_width_m": round(ground_width_m, 1),
        "ground_height_m": round(ground_height_m, 1),
        "trigger_distance_m": round(trigger_dist_m, 1),
        "gsd_cm": target_gsd_cm
    }

def generate_serpentine_grid(
    polygon: Polygon,
    line_spacing_m: float,
    strip_angle_deg: float = 0.0,
    drone_type: str = "fixed_wing",
    min_turning_radius_m: float = 45.0
) -> List[Tuple[float, float]]:
    """
    Generates a coverage path (галсы) over the polygon rotated to strip_angle_deg.
    Returns ordered waypoints list [(x, y), ...] in meters.
    """
    if polygon.is_empty or not polygon.is_valid:
        polygon = polygon.buffer(0)
        
    # Rotate polygon by -strip_angle so strips are aligned horizontally along X axis
    rot_poly = rotate(polygon, -strip_angle_deg, origin=(0, 0))
    minx, miny, maxx, maxy = rot_poly.bounds
    
    y_coords = np.arange(miny + line_spacing_m * 0.5, maxy, line_spacing_m)
    if len(y_coords) == 0:
        y_coords = np.array([(miny + maxy) / 2.0])
        
    raw_lines: List[List[Tuple[float, float]]] = []
    
    for y in y_coords:
        scanline = LineString([(minx - 100, y), (maxx + 100, y)])
        inter = rot_poly.intersection(scanline)
        if inter.is_empty:
            continue
            
        if inter.geom_type == "LineString":
            coords = list(inter.coords)
            if len(coords) >= 2:
                raw_lines.append(coords)
        elif inter.geom_type == "MultiLineString":
            for seg in inter.geoms:
                coords = list(seg.coords)
                if len(coords) >= 2:
                    raw_lines.append(coords)
                    
    if not raw_lines:
        centroid = rot_poly.centroid
        raw_lines = [[(centroid.x - 50, centroid.y), (centroid.x + 50, centroid.y)]]
        
    # Alternate directions (Boustrophedon serpentine pattern)
    ordered_rot_points: List[Tuple[float, float]] = []
    for i, line in enumerate(raw_lines):
        if i % 2 == 1:
            line = list(reversed(line))
        ordered_rot_points.extend(line)
        
    # Rotate points back to original world frame
    final_points: List[Tuple[float, float]] = []
    cos_a = math.cos(math.radians(strip_angle_deg))
    sin_a = math.sin(math.radians(strip_angle_deg))
    
    for px, py in ordered_rot_points:
        orig_x = px * cos_a - py * sin_a
        orig_y = px * sin_a + py * cos_a
        final_points.append((orig_x, orig_y))
        
    return final_points
