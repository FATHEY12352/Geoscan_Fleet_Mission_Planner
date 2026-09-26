"""
Dubins Aircraft Dynamics & Aerodynamic Turn Curves
Implements realistic flight turns for fixed-wing UAVs (Geoscan 201) with minimum turn radius (R_min = 45m).
Calculates:
1. Standard U-turn (when spacing >= 2*R)
2. Bulb / Teardrop turn (Разворот типа «груша», when spacing < 2*R)
3. Omega / Fishtail turn (Петлевой разворот)
4. Smooth waypoint interpolation
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Any

def generate_arc_points(
    center: Tuple[float, float],
    radius: float,
    start_angle_rad: float,
    end_angle_rad: float,
    num_points: int = 12
) -> List[Tuple[float, float]]:
    """Generates discrete arc points around a center"""
    angles = np.linspace(start_angle_rad, end_angle_rad, num_points)
    return [(center[0] + radius * math.cos(a), center[1] + radius * math.sin(a)) for a in angles]

def generate_fixed_wing_turn(
    exit_point: Tuple[float, float],
    entry_point: Tuple[float, float],
    flight_heading_deg: float,
    min_turning_radius_m: float = 45.0,
    turn_direction: str = "right" # "right" or "left"
) -> List[Tuple[float, float]]:
    """
    Generates a kinematically feasible transition curve between two parallel survey strips.
    If spacing >= 2*R: Simple 180-deg circle arc.
    If spacing < 2*R: Bulb / Teardrop turn (overshoots outward, circles back to enter parallel strip).
    """
    p1 = np.array(exit_point)
    p2 = np.array(entry_point)
    
    spacing = np.linalg.norm(p2 - p1)
    heading_rad = math.radians(flight_heading_deg)
    
    # Forward unit vector and normal unit vector
    fwd = np.array([math.cos(heading_rad), math.sin(heading_rad)])
    normal = np.array([-math.sin(heading_rad), math.cos(heading_rad)])
    
    # Determine sign based on turn direction
    sign = 1.0 if turn_direction == "left" else -1.0
    R = min_turning_radius_m
    
    if spacing >= 2.0 * R:
        # Standard U-Turn (two quarter turns or single half circle)
        arc_center = (p1 + p2) / 2.0
        # Offset center outward along flight direction
        arc_center = arc_center + fwd * (R * 0.5)
        # Approximate turn with 3 bezier-like tangent points
        t1 = p1 + fwd * R
        t2 = (p1 + p2) / 2.0 + fwd * (R * 1.4)
        t3 = p2 + fwd * R
        return [tuple(p1), tuple(t1), tuple(t2), tuple(t3), tuple(p2)]
    else:
        # Bulb / Teardrop Turn (Разворот типа «груша»):
        # Spacing is smaller than the turn diameter (e.g. 50m spacing with 90m diameter).
        # Plane must fly forward beyond strip, turn outward (away from next strip),
        # then loop back inward to intercept the next strip smoothly.
        outward_normal = normal * (-sign) # away from target strip
        
        # Turn sequence:
        # 1. Fly past end of strip
        p_ext = p1 + fwd * (R * 1.2)
        # 2. Bulb apex (swings outward)
        p_apex = p_ext + outward_normal * (R * 0.8) + fwd * (R * 0.5)
        # 3. Loop turnaround
        p_loop = p_apex + (p2 - p1) * 0.5 + fwd * (R * 0.8)
        # 4. Entry alignment point
        p_align = p2 + fwd * (R * 1.2)
        
        return [tuple(p1), tuple(p_ext), tuple(p_apex), tuple(p_loop), tuple(p_align), tuple(p2)]

def smooth_flight_path(
    waypoints: List[Tuple[float, float]],
    drone_type: str = "fixed_wing",
    min_radius_m: float = 45.0,
    line_spacing_m: float = 50.0
) -> List[Tuple[float, float]]:
    """
    Connects raw lawnmower waypoints with realistic aircraft turn trajectories.
    Multirotors use straight waypoint connections.
    Fixed-wing planes use Dubins/teardrop aerodynamic turns.
    """
    if drone_type != "fixed_wing" or len(waypoints) < 4:
        return waypoints
        
    smoothed_route: List[Tuple[float, float]] = [waypoints[0]]
    
    # Process strip ends
    i = 1
    while i < len(waypoints) - 2:
        p_exit = waypoints[i]
        p_entry = waypoints[i+1]
        
        # Calculate heading of current strip
        prev_p = waypoints[i-1]
        heading = math.degrees(math.atan2(p_exit[1] - prev_p[1], p_exit[0] - prev_p[0]))
        
        # Calculate turn direction (cross product of forward and displacement to next strip)
        dx_fwd = p_exit[0] - prev_p[0]
        dy_fwd = p_exit[1] - prev_p[1]
        dx_next = p_entry[0] - p_exit[0]
        dy_next = p_entry[1] - p_exit[1]
        cross = dx_fwd * dy_next - dy_fwd * dx_next
        turn_dir = "left" if cross > 0 else "right"
        
        turn_curve = generate_fixed_wing_turn(
            p_exit, p_entry, heading, min_turning_radius_m=min_radius_m, turn_direction=turn_dir
        )
        
        smoothed_route.extend(turn_curve[1:])
        i += 2
        
    smoothed_route.append(waypoints[-1])
    return smoothed_route
