"""
Official Geoscan Drone Fleet & Sensor Specifications
Extracted from official Geoscan datasheets for Hackathon 2026
"""

from typing import Dict, Any, List

GEOSCAN_FLEET: Dict[str, Dict[str, Any]] = {
    "geoscan_201": {
        "id": "geoscan_201",
        "name": "Геоскан 201",
        "type": "fixed_wing",  # Самолёт / летающее крыло
        "wingspan_m": 2.24,
        "takeoff_weight_kg": 8.5,
        "max_flight_time_min": 180,  # 3 hours max endurance
        "cruise_speed_ms": 23.6,     # ~85 km/h (range 72-130 km/h)
        "max_speed_ms": 36.1,        # 130 km/h
        "min_turning_radius_m": 45.0, # Minimum turning radius for plane
        "takeoff_type": "catapult",  # Катапульта
        "landing_type": "parachute", # Парашют
        "parachute_descent_rate_ms": 5.0, # Rate of descent for parachute
        "max_wind_resistance_ms": 12.0,   # Max allowable wind speed
        "supported_sensors": ["rgb_sony_rx1r", "multispectral_pollux", "thermal_flir", "lidar_agm"],
        "battery_capacity_mah": 32000,
        "base_power_consumption_w": 180.0,
        "operating_cost_rub_per_hour": 3500.0,
        "maintenance_interval_hours": 50.0,
        "description": "Дальнемагистральный комплекс для аэрофотосъёмки больших площадей."
    },
    "geoscan_801": {
        "id": "geoscan_801",
        "name": "Геоскан 801",
        "type": "multirotor",  # Тяжёлый промышленный квадрокоптер
        "takeoff_weight_kg": 15.0,
        "max_payload_kg": 4.0,
        "max_flight_time_min": 45,   # 45 minutes
        "cruise_speed_ms": 12.5,     # 45 km/h
        "max_speed_ms": 16.7,        # 60 km/h
        "min_turning_radius_m": 0.0, # Can turn on the spot / hover
        "takeoff_type": "vtol",      # Вертикальный взлёт и посадка
        "landing_type": "vtol",
        "max_wind_resistance_ms": 10.0,
        "supported_sensors": ["lidar_agm", "rgb_sony_rx1r", "thermal_flir", "geophysical_mag"],
        "battery_capacity_mah": 22000,
        "base_power_consumption_w": 450.0,
        "operating_cost_rub_per_hour": 5000.0,
        "maintenance_interval_hours": 30.0,
        "description": "Тяжёлый коптер для лазерного сканирования (LiDAR) и геофизики."
    },
    "geoscan_gemini": {
        "id": "geoscan_gemini",
        "name": "Геоскан Gemini",
        "type": "multirotor",  # Компактный геодезический квадрокоптер
        "takeoff_weight_kg": 2.2,
        "max_flight_time_min": 40,   # 40 minutes
        "cruise_speed_ms": 10.0,     # 36 km/h
        "max_speed_ms": 14.0,        # 50 km/h
        "min_turning_radius_m": 0.0, # Can turn on spot / hover
        "takeoff_type": "vtol",
        "landing_type": "vtol",
        "max_wind_resistance_ms": 8.0,
        "supported_sensors": ["rgb_sony_rx1r", "multispectral_pollux"],
        "battery_capacity_mah": 10000,
        "base_power_consumption_w": 220.0,
        "operating_cost_rub_per_hour": 2500.0,
        "maintenance_interval_hours": 40.0,
        "description": "Компактный квадрокоптер высокой точности с геодезическим приёмником."
    }
}

SENSOR_CATALOG: Dict[str, Dict[str, Any]] = {
    "rgb_sony_rx1r": {
        "id": "rgb_sony_rx1r",
        "name": "Sony RX1R II (RGB 42 Мп)",
        "type": "rgb",
        "focal_length_mm": 35.0,
        "sensor_width_mm": 35.9,
        "sensor_height_mm": 24.0,
        "image_width_px": 7952,
        "image_height_px": 5304,
        "pixel_size_um": 4.51,
        "recommended_overlap_forward": 0.75,
        "recommended_overlap_side": 0.65,
        "weight_g": 507
    },
    "multispectral_pollux": {
        "id": "multispectral_pollux",
        "name": "Геоскан Поллукс (Мультиспектральная 5 каналов)",
        "type": "multispectral",
        "focal_length_mm": 12.0,
        "sensor_width_mm": 12.8,
        "sensor_height_mm": 9.6,
        "pixel_size_um": 3.45,
        "recommended_overlap_forward": 0.80,
        "recommended_overlap_side": 0.70,
        "weight_g": 350
    },
    "thermal_flir": {
        "id": "thermal_flir",
        "name": "FLIR Duo Pro R (Тепловизор + RGB)",
        "type": "thermal",
        "focal_length_mm": 19.0,
        "sensor_width_mm": 10.8,
        "sensor_height_mm": 8.6,
        "pixel_size_um": 17.0,
        "recommended_overlap_forward": 0.80,
        "recommended_overlap_side": 0.75,
        "weight_g": 325
    },
    "lidar_agm": {
        "id": "lidar_agm",
        "name": "АГМ-МС3 (Лазерный сканер LiDAR)",
        "type": "lidar",
        "fov_deg": 360.0,
        "pulse_rate_khz": 300,
        "optimal_altitude_m": 80.0,
        "max_altitude_m": 150.0,
        "swath_width_factor": 1.2, # Width = 1.2 * Altitude
        "recommended_overlap_side": 0.50,
        "weight_g": 1800
    },
    "geophysical_mag": {
        "id": "geophysical_mag",
        "name": "Квантовый магнитометр (Геофизика)",
        "type": "geophysical",
        "optimal_altitude_m": 30.0, # Low terrain following
        "max_altitude_m": 50.0,
        "max_survey_speed_ms": 12.0,
        "recommended_overlap_side": 0.30, # Narrow parallel lines
        "weight_g": 2200
    }
}
