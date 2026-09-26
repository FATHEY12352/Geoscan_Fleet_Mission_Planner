"""
Export Module: KML 2.2, GeoJSON, and QGroundControl / Geoscan Planner .plan
Generates industry-standard flight mission files.
Supports sovereign georeferencing locked to actual site origins (e.g. Surgut oilfields, Primorsky, Yamal).
"""

import json
from typing import Dict, Any, List

# Primary Siberian Industrial Flight Operations Hub (Surgut Oilfield Basin)
DEFAULT_SURGUT_LAT = 61.2540
DEFAULT_SURGUT_LON = 73.4140
MOSCOW_CENTER_LAT = 55.7558
MOSCOW_CENTER_LON = 37.6173

def resolve_mission_origin(data_dict: Dict[str, Any], default_lat: float = DEFAULT_SURGUT_LAT, default_lon: float = DEFAULT_SURGUT_LON) -> tuple[float, float]:
    """
    Strict Geodetic Reference Resolver:
    Extracts origin from mission dictionary. Prevents silent, dangerous fallback to Moscow center
    when processing external regional flight tasks (Siberia, Yamal, Far East).
    """
    lat = float(data_dict.get("origin_lat", default_lat))
    lon = float(data_dict.get("origin_lon", default_lon))
    
    target_region = str(data_dict.get("target_region", "")).lower()
    if target_region and target_region not in ("moscow", "москва"):
        # Prohibit Moscow coordinates on regional tasks
        if abs(lat - MOSCOW_CENTER_LAT) < 0.005 and abs(lon - MOSCOW_CENTER_LON) < 0.005:
            lat = DEFAULT_SURGUT_LAT
            lon = DEFAULT_SURGUT_LON
            
    return lat, lon

def export_to_geojson(mission_plan: Dict[str, Any], origin_lat: float = DEFAULT_SURGUT_LAT, origin_lon: float = DEFAULT_SURGUT_LON) -> Dict[str, Any]:
    """
    Exports mission routes and waypoints as GeoJSON FeatureCollection.
    Converts local metric coordinates (x, y) to WGS84 (lon, lat) using equirectangular approximation.
    """
    origin_lat, origin_lon = resolve_mission_origin(mission_plan, origin_lat, origin_lon)
    features: List[Dict[str, Any]] = []
    
    # Approx conversion: 1 deg lat ~ 111320m, 1 deg lon ~ 111320 * cos(lat)
    m_per_deg_lat = 111320.0
    import math
    m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    
    for drone in mission_plan.get("assignments", []):
        d_id = drone["drone_id"]
        d_name = drone["drone_name"]
        alt = drone["altitude_m"]
        wpts = drone["waypoints"]
        
        # Coordinates in [lon, lat, alt]
        geo_coords = []
        for pt in wpts:
            x, y = pt[0], pt[1]
            pt_alt = pt[2] if len(pt) > 2 else alt
            geo_coords.append([
                round(origin_lon + (x / m_per_deg_lon), 6),
                round(origin_lat + (y / m_per_deg_lat), 6),
                round(pt_alt, 1)
            ])
        
        # LineString for trajectory
        features.append({
            "type": "Feature",
            "properties": {
                "drone_id": d_id,
                "drone_name": d_name,
                "flight_duration_min": drone["flight_duration_min"],
                "battery_used_pct": drone["battery_used_pct"],
                "total_distance_km": drone["total_distance_km"]
            },
            "geometry": {
                "type": "LineString",
                "coordinates": geo_coords
            }
        })
        
        # Point for start and landing
        if geo_coords:
            features.append({
                "type": "Feature",
                "properties": {"name": f"{d_name} Takeoff/Landing", "type": "base"},
                "geometry": {"type": "Point", "coordinates": geo_coords[0]}
            })
            
    return {
        "type": "FeatureCollection",
        "metadata": {
            "generator": "Geoscan Fleet Planner 2026",
            "criterion": mission_plan.get("criterion"),
            "survey_type": mission_plan.get("survey_type"),
            "origin_wgs84": [origin_lat, origin_lon],
            "total_drones": len(mission_plan.get("assignments", []))
        },
        "features": features
    }

def export_to_kml(mission_plan: Dict[str, Any], origin_lat: float = DEFAULT_SURGUT_LAT, origin_lon: float = DEFAULT_SURGUT_LON) -> str:
    """
    Generates KML 2.2 document with 3D paths and waypoints.
    """
    origin_lat, origin_lon = resolve_mission_origin(mission_plan, origin_lat, origin_lon)
    import math
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    
    kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '  <Document>',
        f'    <name>Geoscan Mission - {mission_plan.get("criterion", "Plan")}</name>',
        '    <Style id="dronePath">',
        '      <LineStyle>',
        '        <color>ff00aaff</color>',
        '        <width>3</width>',
        '      </LineStyle>',
        '    </Style>'
    ]
    
    for drone in mission_plan.get("assignments", []):
        d_name = drone["drone_name"]
        alt = drone["altitude_m"]
        wpts = drone["waypoints"]
        
        coord_strings = []
        for pt in wpts:
            x, y = pt[0], pt[1]
            pt_alt = pt[2] if len(pt) > 2 else alt
            coord_strings.append(
                f"{round(origin_lon + (x / m_per_deg_lon), 6)},{round(origin_lat + (y / m_per_deg_lat), 6)},{round(pt_alt, 1)}"
            )
        
        kml.extend([
            '    <Placemark>',
            f'      <name>{d_name} Flight Trajectory</name>',
            f'      <description>Duration: {drone["flight_duration_min"]} min, Battery: {drone["battery_used_pct"]}%</description>',
            '      <styleUrl>#dronePath</styleUrl>',
            '      <LineString>',
            '        <altitudeMode>relativeToGround</altitudeMode>',
            f'        <coordinates>{" ".join(coord_strings)}</coordinates>',
            '      </LineString>',
            '    </Placemark>'
        ])
        
    kml.extend([
        '  </Document>',
        '</kml>'
    ])
    
    return "\n".join(kml)

def export_to_qgc_plan(drone_assignment: Dict[str, Any], origin_lat: float = DEFAULT_SURGUT_LAT, origin_lon: float = DEFAULT_SURGUT_LON) -> Dict[str, Any]:
    """
    Exports a single drone's flight assignment to QGroundControl / Geoscan Planner .plan format.
    """
    origin_lat, origin_lon = resolve_mission_origin(drone_assignment, origin_lat, origin_lon)
    import math
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    
    wpts = drone_assignment.get("waypoints", [])
    alt = drone_assignment.get("altitude_m", 100.0)
    
    mission_items = []
    
    trig_dist = float(drone_assignment.get("trigger_distance_m", 25.0))
    
    # 1. Takeoff command (MAV_CMD_NAV_TAKEOFF = 22)
    mission_items.append({
        "autoContinue": True,
        "command": 22,
        "doJumpId": 1,
        "frame": 3,
        "params": [0, 0, 0, None, origin_lat, origin_lon, alt],
        "type": "SimpleItem"
    })
    
    # 1b. Camera Trigger Activation (MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206)
    # Automatically triggers photogrammetry camera shutter every trig_dist meters
    mission_items.append({
        "autoContinue": True,
        "command": 206,
        "doJumpId": len(mission_items) + 1,
        "frame": 2,
        "params": [trig_dist, 0, 1, 0, 0, 0, 0],
        "type": "SimpleItem"
    })
    
    # 2. Survey Grid and Dubins Waypoints (MAV_CMD_NAV_WAYPOINT = 16)
    for pt in wpts:
        x, y = pt[0], pt[1]
        pt_alt = pt[2] if len(pt) > 2 else alt
        lat = round(origin_lat + (y / m_per_deg_lat), 7)
        lon = round(origin_lon + (x / m_per_deg_lon), 7)
        mission_items.append({
            "autoContinue": True,
            "command": 16,
            "doJumpId": len(mission_items) + 1,
            "frame": 3,
            "params": [0, 0, 0, None, lat, lon, pt_alt],
            "type": "SimpleItem"
        })
        
    # 2b. Camera Trigger Stop before Landing (MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206 with 0m)
    mission_items.append({
        "autoContinue": True,
        "command": 206,
        "doJumpId": len(mission_items) + 1,
        "frame": 2,
        "params": [0, 0, 0, 0, 0, 0, 0],
        "type": "SimpleItem"
    })
    
    # 3. Land / RTL command (MAV_CMD_NAV_RETURN_TO_LAUNCH = 20)
    mission_items.append({
        "autoContinue": True,
        "command": 20,
        "doJumpId": len(mission_items) + 1,
        "frame": 2,
        "params": [0, 0, 0, 0, 0, 0, 0],
        "type": "SimpleItem"
    })
    
    return {
        "fileType": "Plan",
        "version": 1,
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": float(drone_assignment.get("cruise_speed_ms", 21.0)),
            "firmwareType": 12,
            "hoverSpeed": 5.0,
            "items": mission_items,
            "plannedHomePosition": [origin_lat, origin_lon, alt],
            "vehicleType": 1 if drone_assignment.get("drone_type") == "fixed_wing" else 2,
            "version": 2
        },
        "geoFence": {
            "circles": [],
            "polygons": [],
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        }
    }
