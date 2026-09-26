"""
Wind Vector and Aerodynamic Effects Engine
Calculates wind impact on groundspeed, battery consumption, optimal strip orientation,
and parachute drift for fixed-wing aircraft (Geoscan 201).
"""

import math
from typing import Dict, Any, Tuple

def normalize_angle_deg(deg: float) -> float:
    """Normalize angle to 0..360 degrees"""
    return deg % 360.0

def calculate_wind_components(
    flight_heading_deg: float,
    wind_speed_ms: float,
    wind_direction_deg: float  # Direction wind is coming FROM (standard meteorological)
) -> Dict[str, float]:
    """
    Computes along-track (headwind/tailwind) and cross-track wind components.
    Positive headwind means flying INTO the wind (slows groundspeed).
    Positive tailwind means flying WITH the wind (boosts groundspeed).
    """
    if wind_speed_ms <= 0.01:
        return {
            "headwind_ms": 0.0,
            "tailwind_ms": 0.0,
            "crosswind_ms": 0.0,
            "drift_angle_deg": 0.0
        }
    
    # Angle between flight track and wind source
    rel_angle_rad = math.radians(wind_direction_deg - flight_heading_deg)
    
    # Positive = headwind, Negative = tailwind
    along_component = wind_speed_ms * math.cos(rel_angle_rad)
    cross_component = wind_speed_ms * math.sin(rel_angle_rad)
    
    headwind = max(0.0, along_component)
    tailwind = max(0.0, -along_component)
    
    return {
        "headwind_ms": headwind,
        "tailwind_ms": tailwind,
        "crosswind_ms": abs(cross_component),
        "along_signed_ms": -along_component # Positive = speed boost
    }

def adjust_groundspeed(
    cruise_airspeed_ms: float,
    flight_heading_deg: float,
    wind_speed_ms: float,
    wind_direction_deg: float
) -> float:
    """
    Calculates realistic directional ground speed considering wind vector along a single track.
    Tailwind boosts groundspeed, headwind slows groundspeed.
    Ensures groundspeed does not drop below safe stall margin for planes.
    """
    if wind_speed_ms <= 0.01:
        return float(cruise_airspeed_ms)
        
    components = calculate_wind_components(flight_heading_deg, wind_speed_ms, wind_direction_deg)
    groundspeed = cruise_airspeed_ms + components["along_signed_ms"]
    
    # Ground speed minimum safety threshold
    min_safe_speed = max(5.0, cruise_airspeed_ms * 0.35)
    return float(max(min_safe_speed, groundspeed))

def compute_survey_average_groundspeed(
    cruise_airspeed_ms: float,
    strip_heading_deg: float,
    wind_speed_ms: float,
    wind_direction_deg: float
) -> float:
    """
    Calculates realistic average groundspeed for a bidirectional survey grid (lawnmower pattern)
    where aircraft flies alternating forward (strip_heading) and reverse (strip_heading + 180).
    Uses harmonic mean of back-and-forth speeds with crosswind crab penalty.
    """
    if wind_speed_ms <= 0.05:
        return float(cruise_airspeed_ms)
        
    v_forward = adjust_groundspeed(cruise_airspeed_ms, strip_heading_deg, wind_speed_ms, wind_direction_deg)
    v_return = adjust_groundspeed(cruise_airspeed_ms, (strip_heading_deg + 180.0) % 360.0, wind_speed_ms, wind_direction_deg)
    
    v_harmonic = 2.0 / ((1.0 / v_forward) + (1.0 / v_return))
    
    # Crosswind component penalty
    rel_angle_rad = math.radians(wind_direction_deg - strip_heading_deg)
    cross_ms = abs(wind_speed_ms * math.sin(rel_angle_rad))
    cross_ratio = min(0.9, cross_ms / max(5.0, cruise_airspeed_ms))
    crab_factor = math.sqrt(max(0.1, 1.0 - (cross_ratio ** 2)))
    
    return float(max(max(5.0, cruise_airspeed_ms * 0.35), v_harmonic * crab_factor))

def compute_wind_optimal_strip_angle(wind_direction_deg: float) -> float:
    """
    Computes the optimal survey strip heading (галсы) aligned with the wind.
    Aligning flight strips along the wind axis minimizes lateral drift (crab angle),
    maximizes photo overlap stability, and saves up to 20% energy during turns.
    Returns optimal strip angle in degrees (0..180).
    """
    return normalize_angle_deg(wind_direction_deg) % 180.0

def calculate_parachute_drift_offset(
    altitude_m: float,
    descent_rate_ms: float,
    wind_speed_ms: float,
    wind_direction_deg: float
) -> Tuple[float, float, float]:
    """
    Calculates the landing displacement (drift) of a parachute due to wind.
    Used for Geoscan 201 landing calculations.
    Returns: (drift_distance_meters, drift_bearing_deg, descent_time_sec)
    """
    if descent_rate_ms <= 0.1:
        descent_rate_ms = 5.0
        
    descent_time_sec = altitude_m / descent_rate_ms
    drift_distance_m = wind_speed_ms * descent_time_sec
    
    # Parachute drifts in the direction wind is blowing TO (wind_direction_deg + 180)
    drift_bearing_deg = normalize_angle_deg(wind_direction_deg + 180.0)
    
    return drift_distance_m, drift_bearing_deg, descent_time_sec

def compute_wind_power_factor(
    cruise_airspeed_ms: float,
    flight_heading_deg: float,
    wind_speed_ms: float,
    wind_direction_deg: float
) -> float:
    """
    Calculates power consumption multiplier based on wind resistance and motor compensation.
    Baseline = 1.0 (calm).
    Turbulence and continuous headwind/crosswind corrections increase battery power draw.
    """
    if wind_speed_ms <= 0.5:
        return 1.0
        
    # Empirical aerospace formula for UAV energy drain in gusty/wind conditions
    power_mult = 1.0 + (0.038 * wind_speed_ms) + (0.0022 * (wind_speed_ms ** 2))
    return float(min(2.2, max(1.0, power_mult)))

def validate_crosswind_safety(
    cruise_airspeed_ms: float,
    strip_heading_deg: float,
    wind_speed_ms: float,
    wind_direction_deg: float,
    max_crosswind_ratio: float = 0.40
) -> Dict[str, Any]:
    """
    Aviation Safety Check: Verifies that crosswind component does not exceed 40% of cruise airspeed (0.40 Va).
    Exceeding this threshold forces the crab angle chi_crab > 23.57 deg, saturating 3-axis camera gimbals
    and causing aerodynamic roll instability in fixed-wing UAVs (Geoscan 201).
    """
    if wind_speed_ms <= 0.1:
        return {
            "is_safe": True,
            "crosswind_ms": 0.0,
            "crab_angle_deg": 0.0,
            "max_safe_crab_deg": 23.57,
            "warning": None,
            "recommended_heading_deg": round(strip_heading_deg, 1)
        }
        
    rel_angle_rad = math.radians(wind_direction_deg - strip_heading_deg)
    cross_ms = abs(wind_speed_ms * math.sin(rel_angle_rad))
    ratio = cross_ms / max(1.0, cruise_airspeed_ms)
    crab_rad = math.asin(min(1.0, ratio))
    crab_deg = math.degrees(crab_rad)
    
    is_safe = ratio <= max_crosswind_ratio
    optimal_heading = compute_wind_optimal_strip_angle(wind_direction_deg)
    
    warning = None
    if not is_safe:
        warning = (
            f"ОПАСНЫЙ БОКОВОЙ ВЕТЕР: V_cross={cross_ms:.1f} м/с превышает 40% скорости полёта "
            f"({cruise_airspeed_ms:.1f} м/с). Угол сноса {crab_deg:.1f}° > 23.5°. "
            f"Рекомендуется разворот галсов на курс {optimal_heading:.0f}° по оси ветра."
        )
        
    return {
        "is_safe": is_safe,
        "crosswind_ms": round(cross_ms, 2),
        "crab_angle_deg": round(crab_deg, 2),
        "max_safe_crab_deg": 23.57,
        "crosswind_ratio": round(ratio, 3),
        "warning": warning,
        "recommended_heading_deg": round(optimal_heading, 1)
    }

def calculate_arctic_battery_derating(
    temp_celsius: float,
    nominal_capacity_mah: float = 22000.0
) -> Dict[str, Any]:
    """
    Sub-Zero Siberian / Arctic Battery Derating Model (Peukert's Law & Arrhenius Kinetics).
    In freezing conditions (-15°C to -35°C in Surgut, Yamal, Novy Urengoy), internal resistance
    surges (R_int), causing sharp discharge voltage sag.
    - Below 0°C: Capacity degrades by ~1.16% per °C down to -35% at -30°C.
    - Below -15°C: Return-to-base safety reserve is automatically raised from 20% to 35%.
    """
    if temp_celsius >= 15.0:
        derating_factor = 1.0
        reserve_pct = 20.0
    elif temp_celsius >= 0.0:
        # Mild cold: 0% to 5% loss
        derating_factor = 1.0 - ((15.0 - temp_celsius) / 15.0) * 0.05
        reserve_pct = 20.0
    else:
        # Negative temperatures: up to 35% capacity loss at -30°C
        temp_drop = abs(temp_celsius)
        derating_pct = min(35.0, temp_drop * 1.166)
        derating_factor = (100.0 - derating_pct) / 100.0
        
        # Raise reserve margin to 35% if extreme sub-zero (-15°C or lower)
        reserve_pct = 35.0 if temp_celsius <= -15.0 else 25.0
        
    effective_capacity = nominal_capacity_mah * derating_factor
    
    return {
        "temperature_celsius": temp_celsius,
        "derating_factor": round(derating_factor, 3),
        "capacity_loss_pct": round((1.0 - derating_factor) * 100.0, 1),
        "effective_capacity_mah": round(effective_capacity, 0),
        "mandatory_reserve_pct": reserve_pct,
        "is_arctic_mode": temp_celsius <= -15.0
    }

