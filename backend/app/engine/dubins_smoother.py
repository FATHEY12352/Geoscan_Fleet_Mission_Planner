import math
from typing import List, Tuple

def smooth_dubins_trajectory(
    pts: List[Tuple[float, float]],
    min_radius: float = 48.0,
    max_pts_per_arc: int = 10
) -> List[Tuple[float, float]]:
    """
    Enforces minimum turning radius R >= min_radius on fixed-wing waypoint paths.
    Replaces sharp corners with tangent circular fillets or Dubins arc sequences.
    Preserves 3D coordinates (z) if present.
    """
    if len(pts) < 3:
        return pts

    has_z = len(pts[0]) >= 3
    
    # 1. Clean collinear or micro-step points (< 1.5m)
    cleaned = [pts[0]]
    for p in pts[1:]:
        d = math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1])
        if d >= 1.5:
            cleaned.append(p)
            
    if len(cleaned) < 3:
        return cleaned

    # Check curvature of each triple
    result = [cleaned[0]]
    idx = 1
    while idx < len(cleaned) - 1:
        p_prev = result[-1]
        p_curr = cleaned[idx]
        p_next = cleaned[idx + 1]
        
        # Vectors
        v1 = (p_prev[0] - p_curr[0], p_prev[1] - p_curr[1])
        v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
        d1 = math.hypot(v1[0], v1[1])
        d2 = math.hypot(v2[0], v2[1])
        
        if d1 < 1e-3 or d2 < 1e-3:
            idx += 1
            continue
            
        u1 = (v1[0] / d1, v1[1] / d1)
        u2 = (v2[0] / d2, v2[1] / d2)
        
        dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        gamma = math.acos(dot) # Angle between incoming and outgoing directions
        turn_angle = math.pi - gamma
        
        # If nearly collinear (< 1.5 degrees), keep p_curr directly
        if turn_angle < math.radians(1.5):
            result.append(p_curr)
            idx += 1
            continue
            
        # Compute circumradius of the existing discrete triple
        a = math.hypot(p_curr[0] - p_next[0], p_curr[1] - p_next[1])
        b = math.hypot(p_prev[0] - p_next[0], p_prev[1] - p_next[1])
        c = math.hypot(p_prev[0] - p_curr[0], p_prev[1] - p_curr[1])
        area = abs(p_prev[0]*(p_curr[1]-p_next[1]) + p_curr[0]*(p_next[1]-p_prev[1]) + p_next[0]*(p_prev[1]-p_curr[1])) / 2.0
        R_exist = (a * b * c) / (4.0 * area) if area > 1e-4 else 9999.0
        
        if R_exist >= min_radius - 0.5:
            # Already compliant!
            result.append(p_curr)
            idx += 1
            continue
            
        # Fillet needed! Tangent distance from p_curr
        tan_half = math.tan(turn_angle / 2.0)
        L = min_radius * tan_half
        
        # Max feasible tangent distance without eating adjacent segments
        L_max = min(d1 * 0.45, d2 * 0.45)
        
        if L <= L_max:
            # Symmetrical tangent points
            t1 = (p_curr[0] + u1[0] * L, p_curr[1] + u1[1] * L)
            t2 = (p_curr[0] + u2[0] * L, p_curr[1] + u2[1] * L)
            
            # Bisector vector pointing to center
            bis = (u1[0] + u2[0], u1[1] + u2[1])
            bis_len = math.hypot(bis[0], bis[1])
            if bis_len > 1e-4:
                bis_u = (bis[0] / bis_len, bis[1] / bis_len)
                c_dist = min_radius / math.cos(turn_angle / 2.0)
                center = (p_curr[0] + bis_u[0] * c_dist, p_curr[1] + bis_u[1] * c_dist)
                
                ang1 = math.atan2(t1[1] - center[1], t1[0] - center[0])
                ang2 = math.atan2(t2[1] - center[1], t2[0] - center[0])
                
                # Determine turn direction (cross product)
                cross = u1[0] * u2[1] - u1[1] * u2[0]
                # If cross > 0, turning in one direction; normalize angle span
                diff = (ang2 - ang1) % (2 * math.pi)
                if cross < 0:
                    diff = diff - 2 * math.pi
                    
                n_pts = max(4, min(max_pts_per_arc, int(abs(diff) * 8)))
                
                # Interpolate z altitude if 3D
                z_curr = p_curr[2] if has_z else None
                
                # Append fillet arc
                for step in range(n_pts + 1):
                    theta = ang1 + (step / n_pts) * diff
                    ax = center[0] + min_radius * math.cos(theta)
                    ay = center[1] + min_radius * math.sin(theta)
                    if has_z:
                        result.append((round(ax, 3), round(ay, 3), z_curr))
                    else:
                        result.append((round(ax, 3), round(ay, 3)))
                idx += 1
                continue
                
        # If L > L_max (segments too tight for direct fillet), insert Dubins connector
        from app.engine.dubins_curves import calculate_dubins_path
        h_in = math.atan2(p_curr[1] - p_prev[1], p_curr[0] - p_prev[0])
        h_out = math.atan2(p_next[1] - p_curr[1], p_next[0] - p_curr[0])
        dub = calculate_dubins_path(p_prev, h_in, p_next, h_out, radius=min_radius)
        z_curr = p_curr[2] if has_z else None
        for dp in dub[1:]:
            if has_z:
                result.append((round(dp[0], 3), round(dp[1], 3), z_curr))
            else:
                result.append((round(dp[0], 3), round(dp[1], 3)))
        idx += 2 # Consumed p_curr and p_next
        
    if len(result) == 1 or result[-1][:2] != cleaned[-1][:2]:
        result.append(cleaned[-1])
        
    return result

if __name__ == "__main__":
    import random
    from mega_industrial_test import run_single_algo_test
    print("Testing smoothing on Seed 6...")
