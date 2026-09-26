"""
Multi-Sortie Mission Planner and Battery Swapping Scheduler
Splits large-scale missions that exceed a single battery cycle into sequential sorties,
inserting return-to-base, 10-minute battery swap cycles, and generating a full Gantt timeline.
"""

import math
from typing import List, Tuple, Dict, Any

BATTERY_SWAP_TIME_MIN = 10.0 # Standard field battery replacement duration

def calculate_nonlinear_battery_drain(flight_time_min: float, max_duration_min: float, power_factor: float, temp_c: float = 15.0) -> float:
    """
    Non-linear battery degradation model factoring in temperature and discharge curve.
    Returns the percentage of battery used.
    """
    base_drain_pct = (flight_time_min / max_duration_min) * 100.0 * power_factor
    temp_penalty = 1.0
    if temp_c < 10.0:
        temp_penalty = 1.0 + ((10.0 - temp_c) * 0.02)
    elif temp_c > 35.0:
        temp_penalty = 1.0 + ((temp_c - 35.0) * 0.01)
    curve_factor = 1.0 + 0.15 * math.pow(flight_time_min / max_duration_min, 2)
    return min(100.0, base_drain_pct * temp_penalty * curve_factor)

def plan_multi_sortie_schedule(
    waypoints: List[Tuple[float, float]],
    base_point: Tuple[float, float],
    drone_spec: Dict[str, Any],
    speed_ms: float,
    power_factor: float = 1.0,
    min_reserve_pct: float = 20.0,
    temperature_c: float = 15.0
) -> Dict[str, Any]:
    """
    Evaluates whether waypoints fit in a single battery cycle.
    If not, breaks the mission into sequential sorties with ground battery swap intervals.
    """
    max_duration_min = drone_spec["max_flight_time_min"]
    safe_budget_pct = 100.0 - min_reserve_pct # Typically 80% usable energy
    
    # We estimate max safe flight time assuming a linear rate for scheduling boundaries,
    # but we will calculate actual non-linear drain for the final report.
    battery_rate_per_min = (100.0 / max_duration_min) * power_factor
    max_safe_flight_min = safe_budget_pct / (battery_rate_per_min * 1.15) # 1.15 buffer for non-linear curve
    
    if len(waypoints) < 1:
        return {
            "sorties": [],
            "total_sorties": 1,
            "is_multi_sortie": False,
            "total_airtime_min": 0.0,
            "total_ground_swap_time_min": 0.0,
            "total_elapsed_mission_min": 0.0
        }
        
    sorties: List[Dict[str, Any]] = []
    current_sortie_pts: List[Tuple[float, float]] = [base_point]
    current_flight_time_min = 0.0
    current_dist_m = 0.0
    
    # Iterate through survey waypoints
    survey_pts = waypoints[1:-1] if len(waypoints) > 2 else waypoints
    
    for i in range(len(survey_pts)):
        p = survey_pts[i]
        last_p = current_sortie_pts[-1]
        step_dist = math.hypot(p[0] - last_p[0], p[1] - last_p[1])
        step_time_min = (step_dist / max(5.0, speed_ms)) / 60.0
        
        # Calculate return distance to base from p
        return_dist = math.hypot(base_point[0] - p[0], base_point[1] - p[1])
        return_time_min = (return_dist / max(5.0, speed_ms)) / 60.0
        
        projected_total_time = current_flight_time_min + step_time_min + return_time_min
        
        # If adding this point exceeds safe battery endurance, finalize current sortie
        if projected_total_time >= max_safe_flight_min and len(current_sortie_pts) > 2:
            # Complete current sortie
            current_sortie_pts.append(base_point)
            final_dist = current_dist_m + math.hypot(base_point[0] - last_p[0], base_point[1] - last_p[1])
            final_time = (final_dist / max(5.0, speed_ms)) / 60.0
            batt_used = round(calculate_nonlinear_battery_drain(final_time, max_duration_min, power_factor, temperature_c), 1)
            
            sorties.append({
                "sortie_index": len(sorties) + 1,
                "waypoints": current_sortie_pts,
                "flight_duration_min": round(final_time, 1),
                "distance_km": round(final_dist / 1000.0, 2),
                "battery_used_pct": batt_used,
                "battery_reserve_pct": round(100.0 - batt_used, 1),
                "battery_swap_after_min": BATTERY_SWAP_TIME_MIN
            })
            
            # Start new sortie
            current_sortie_pts = [base_point, p]
            transit_out_dist = math.hypot(p[0] - base_point[0], p[1] - base_point[1])
            current_dist_m = transit_out_dist
            current_flight_time_min = (transit_out_dist / max(5.0, speed_ms)) / 60.0
        else:
            current_sortie_pts.append(p)
            current_dist_m += step_dist
            current_flight_time_min += step_time_min
            
    # Finalize last sortie
    if len(current_sortie_pts) > 1:
        current_sortie_pts.append(base_point)
        last_step = math.hypot(base_point[0] - current_sortie_pts[-2][0], base_point[1] - current_sortie_pts[-2][1])
        current_dist_m += last_step
        final_time = (current_dist_m / max(5.0, speed_ms)) / 60.0
        batt_used = round(calculate_nonlinear_battery_drain(final_time, max_duration_min, power_factor, temperature_c), 1)
        
        sorties.append({
            "sortie_index": len(sorties) + 1,
            "waypoints": current_sortie_pts,
            "flight_duration_min": round(final_time, 1),
            "distance_km": round(current_dist_m / 1000.0, 2),
            "battery_used_pct": batt_used,
            "battery_reserve_pct": round(100.0 - batt_used, 1),
            "battery_swap_after_min": 0.0 # No swap after last mission
        })
        
    total_airtime_min = sum(s["flight_duration_min"] for s in sorties)
    total_swaps = max(0, len(sorties) - 1)
    total_mission_elapsed_min = total_airtime_min + (total_swaps * BATTERY_SWAP_TIME_MIN)
    
    return {
        "is_multi_sortie": len(sorties) > 1,
        "total_sorties": len(sorties),
        "total_airtime_min": round(total_airtime_min, 1),
        "total_ground_swap_time_min": round(total_swaps * BATTERY_SWAP_TIME_MIN, 1),
        "total_elapsed_mission_min": round(total_mission_elapsed_min, 1),
        "sorties": sorties
    }
