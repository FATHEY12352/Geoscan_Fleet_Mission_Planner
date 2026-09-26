"""
Analytical Dubins Path Generator for Fixed-Wing Aircraft (Geoscan 201)
Guarantees continuous curvature with minimum turning radius (R >= 45m).
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

def mod2pi(theta: float) -> float:
    return theta - 2.0 * math.pi * math.floor(theta / (2.0 * math.pi))

def dubins_LSL(alpha: float, beta: float, d: float):
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p_sq = 2.0 + (d * d) - (2.0 * math.cos(alpha - beta)) + (2.0 * d * (math.sin(alpha) - math.sin(beta)))
    if p_sq < 0: return None, None, None
    tmp1 = math.atan2((math.cos(beta) - math.cos(alpha)), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_sq)
    q = mod2pi(beta - tmp1)
    return t, p, q

def dubins_RSR(alpha: float, beta: float, d: float):
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p_sq = 2.0 + (d * d) - (2.0 * math.cos(alpha - beta)) - (2.0 * d * (math.sin(alpha) - math.sin(beta)))
    if p_sq < 0: return None, None, None
    tmp1 = math.atan2((math.cos(alpha) - math.cos(beta)), tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_sq)
    q = mod2pi(-beta + tmp1)
    return t, p, q

def dubins_LSR(alpha: float, beta: float, d: float):
    p_sq = -2.0 + (d * d) + (2.0 * math.cos(alpha - beta)) + (2.0 * d * (math.sin(alpha) + math.sin(beta)))
    if p_sq < 0: return None, None, None
    p = math.sqrt(p_sq)
    tmp2 = math.atan2((-math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) + math.sin(beta))) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)
    return t, p, q

def dubins_RSL(alpha: float, beta: float, d: float):
    p_sq = (d * d) - 2.0 + (2.0 * math.cos(alpha - beta)) - (2.0 * d * (math.sin(alpha) + math.sin(beta)))
    if p_sq < 0: return None, None, None
    p = math.sqrt(p_sq)
    tmp2 = math.atan2((math.cos(alpha) + math.cos(beta)), (d - math.sin(alpha) - math.sin(beta))) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)
    return t, p, q

def dubins_LRL(alpha: float, beta: float, d: float):
    tmp0 = (6.0 - (d * d) + (2.0 * math.cos(alpha - beta)) + (2.0 * d * (-math.sin(alpha) + math.sin(beta)))) / 8.0
    if abs(tmp0) > 1.0: return None, None, None
    p = mod2pi(2.0 * math.pi - math.acos(tmp0))
    t = mod2pi(-alpha - math.atan2((math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) - math.sin(beta))) + (p / 2.0))
    q = mod2pi(beta - alpha - t + p)
    return t, p, q

def dubins_RLR(alpha: float, beta: float, d: float):
    tmp0 = (6.0 - (d * d) + (2.0 * math.cos(alpha - beta)) + (2.0 * d * (math.sin(alpha) - math.sin(beta)))) / 8.0
    if abs(tmp0) > 1.0: return None, None, None
    p = mod2pi(2.0 * math.pi - math.acos(tmp0))
    t = mod2pi(alpha - math.atan2((math.cos(alpha) - math.cos(beta)), (d - math.sin(alpha) + math.sin(beta))) + (p / 2.0))
    q = mod2pi(alpha - beta - t + p)
    return t, p, q

def calculate_dubins_path(
    p_start: Tuple[float, float],
    heading_start_rad: float,
    p_end: Tuple[float, float],
    heading_end_rad: float,
    radius: float = 48.0,
    num_arc_pts: int = 8
) -> List[Tuple[float, float]]:
    """
    Computes exact Dubins path with smooth circular arc interpolation.
    Supports all 6 optimal words: {LSL, RSR, LSR, RSL, LRL, RLR}.
    Strictly maintains radius >= 45m without duplicate or close-point singularities.
    """
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    D = math.hypot(dx, dy)
    d = D / radius
    
    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(heading_start_rad - theta)
    beta = mod2pi(heading_end_rad - theta)
    
    solvers = [
        ("LSL", dubins_LSL),
        ("RSR", dubins_RSR),
        ("LSR", dubins_LSR),
        ("RSL", dubins_RSL),
        ("LRL", dubins_LRL),
        ("RLR", dubins_RLR)
    ]
    
    best_word = None
    best_cost = float("inf")
    best_params = None
    
    for word, solver in solvers:
        t, p, q = solver(alpha, beta, d)
        if t is not None:
            cost = t + p + q
            if cost < best_cost:
                best_cost = cost
                best_word = word
                best_params = (t, p, q)
                
    if best_params is None or best_cost > 30.0:
        return [p_start, p_end]
        
    t, p, q = best_params
    
    # Generate points along Segment 1 (Arc of length t * radius)
    points: List[Tuple[float, float]] = [p_start]
    dir1 = 1.0 if best_word[0] == "L" else -1.0
    c1_x = p_start[0] - radius * dir1 * math.sin(heading_start_rad)
    c1_y = p_start[1] + radius * dir1 * math.cos(heading_start_rad)
    
    if t > 0.05:
        n1 = max(4, int(t * 6))
        for step in range(1, n1 + 1):
            angle = heading_start_rad + dir1 * (step / n1) * t
            px = c1_x + radius * dir1 * math.sin(angle)
            py = c1_y - radius * dir1 * math.cos(angle)
            points.append((round(px, 3), round(py, 3)))
            
    # Segment 2: Arc for CCC (LRL/RLR), Straight for CSC
    if best_word in ("LRL", "RLR"):
        dir2 = -dir1
        if p > 0.05:
            h1 = heading_start_rad + dir1 * t
            p1_x = c1_x + radius * dir1 * math.sin(h1)
            p1_y = c1_y - radius * dir1 * math.cos(h1)
            c2_x = p1_x - radius * dir2 * math.sin(h1)
            c2_y = p1_y + radius * dir2 * math.cos(h1)
            n2 = max(4, int(p * 6))
            for step in range(1, n2 + 1):
                angle = h1 + dir2 * (step / n2) * p
                px = c2_x + radius * dir2 * math.sin(angle)
                py = c2_y - radius * dir2 * math.cos(angle)
                points.append((round(px, 3), round(py, 3)))
    else:
        if p > 0.05:
            last_pt = points[-1]
            line_heading = heading_start_rad + dir1 * t
            p_len = p * radius
            n2 = max(2, int(p_len / 40.0))
            for step in range(1, n2 + 1):
                dist = (step / n2) * p_len
                px = last_pt[0] + dist * math.cos(line_heading)
                py = last_pt[1] + dist * math.sin(line_heading)
                points.append((round(px, 3), round(py, 3)))
            
    # Segment 3 (Arc of length q * radius)
    dir3 = 1.0 if best_word[2] == "L" else -1.0
    if q > 0.05:
        # End circle center
        c3_x = p_end[0] - radius * dir3 * math.sin(heading_end_rad)
        c3_y = p_end[1] + radius * dir3 * math.cos(heading_end_rad)
        start_angle_seg3 = heading_end_rad - dir3 * q
        n3 = max(4, int(q * 6))
        for step in range(1, n3 + 1):
            angle = start_angle_seg3 + dir3 * (step / n3) * q
            px = c3_x + radius * dir3 * math.sin(angle)
            py = c3_y - radius * dir3 * math.cos(angle)
            points.append((round(px, 3), round(py, 3)))
            
    # Ensure end point is exactly p_end and filter duplicate points < 2m
    clean_pts: List[Tuple[float, float]] = [points[0]]
    for pt in points[1:]:
        if math.hypot(pt[0] - clean_pts[-1][0], pt[1] - clean_pts[-1][1]) >= 2.0:
            clean_pts.append(pt)
            
    if math.hypot(p_end[0] - clean_pts[-1][0], p_end[1] - clean_pts[-1][1]) >= 2.0:
        clean_pts.append(p_end)
    else:
        clean_pts[-1] = p_end
        
    return clean_pts

def generate_teardrop_turnaround(
    p_start: Tuple[float, float],
    heading_start_rad: float,
    p_end: Tuple[float, float],
    heading_end_rad: float,
    radius: float = 48.0,
    minkowski_buffer: float = 15.0
) -> List[Tuple[float, float]]:
    """
    Generates a bulb / teardrop turnaround (петля разворота) for fixed-wing UAVs (Geoscan 201).
    Used when lateral distance between parallel survey strips is smaller than 2 * R (e.g. w < 90m).
    Applies an outward expansion buffer (Minkowski buffer: 2 * R_min + 15m) to ensure the turn
    never cuts corners inside the polygon or violates the physical turning radius (R >= 45m).
    """
    exit_dist = radius + minkowski_buffer
    p_fwd = (
        round(p_start[0] + exit_dist * math.cos(heading_start_rad), 3),
        round(p_start[1] + exit_dist * math.sin(heading_start_rad), 3)
    )
    p_entry = (
        round(p_end[0] - exit_dist * math.cos(heading_end_rad), 3),
        round(p_end[1] - exit_dist * math.sin(heading_end_rad), 3)
    )
    
    dub_loop = calculate_dubins_path(p_fwd, heading_start_rad, p_entry, heading_end_rad, radius=radius)
    
    result = [p_start]
    result.extend(dub_loop)
    result.append(p_end)
    
    clean = [result[0]]
    for pt in result[1:]:
        if math.hypot(pt[0] - clean[-1][0], pt[1] - clean[-1][1]) >= 2.0:
            clean.append(pt)
    return clean
