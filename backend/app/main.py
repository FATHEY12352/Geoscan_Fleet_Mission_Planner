import sys
import os

# Ensure backend root directory is in sys.path
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any, List, Optional
from pydantic import BaseModel

from app.data.geoscan_fleet import GEOSCAN_FLEET, SENSOR_CATALOG
from app.engine.optimizer import plan_mission_for_fleet
from app.engine.exporter import export_to_geojson, export_to_kml, export_to_qgc_plan

app = FastAPI(
    title="Geoscan Fleet Mission Planner API",
    description="Intelligent mission planning and dual optimization for heterogeneous Geoscan UAV fleet",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class MissionRequest(BaseModel):
    survey_polygon: List[Any]
    selected_drone_ids: Optional[List[str]] = None
    survey_type: Optional[str] = "rgb"
    criterion: Optional[str] = "min_time"
    wind_speed_ms: Optional[float] = 0.0
    wind_direction_deg: Optional[float] = 0.0
    base_point: Optional[List[float]] = [0.0, 0.0]
    target_gsd_cm: Optional[float] = 5.0
    origin_lat: Optional[float] = 61.2540
    origin_lon: Optional[float] = 73.4140
    nfz_polygons: Optional[Any] = None
    max_time_window_min: Optional[float] = 0.0

import os
from fastapi.responses import FileResponse, HTMLResponse

FRONTEND_FILE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "frontend", "index.html"))

@app.get("/", response_class=HTMLResponse)
def serve_dashboard():
    """Serves the Geoscan Tactical Mission Planning Web UI directly from root"""
    if os.path.exists(FRONTEND_FILE):
        return FileResponse(FRONTEND_FILE)
    return "<h1>Geoscan Mission Planner API</h1><p>Visit /docs for Swagger UI</p>"

@app.get("/api/health")
def health_check():
    return {"status": "healthy", "service": "Geoscan Mission Planner", "version": "1.0.0"}

@app.get("/api/fleet")
def get_fleet():
    """Returns available Geoscan fleet specifications and supported sensors"""
    return {
        "fleet": GEOSCAN_FLEET,
        "sensors": SENSOR_CATALOG
    }

@app.post("/api/optimize")
def optimize_mission(req: MissionRequest):
    """
    Computes optimal flight assignments based on Min Time or Min Wear.
    """
    try:
        from shapely.geometry import Polygon
        coords = [(float(pt[0]), float(pt[1])) for pt in req.survey_polygon if len(pt) >= 2]
        base = (float(req.base_point[0]), float(req.base_point[1])) if req.base_point and len(req.base_point) >= 2 else (0.0, 0.0)
        
        selected_drones = req.selected_drone_ids if req.selected_drone_ids else ["geoscan_201"]
        
        nfz_list = []
        if req.nfz_polygons and isinstance(req.nfz_polygons, list):
            for p in req.nfz_polygons:
                if isinstance(p, list) and len(p) >= 3:
                    try:
                        poly = Polygon([(float(pt[0]), float(pt[1])) for pt in p])
                        if not poly.is_empty and poly.is_valid:
                            nfz_list.append(poly)
                    except Exception:
                        pass
                        
        orig_lat = float(req.origin_lat) if req.origin_lat is not None else 61.2540
        orig_lon = float(req.origin_lon) if req.origin_lon is not None else 73.4140
        
        plan = plan_mission_for_fleet(
            survey_coords=coords,
            selected_drone_ids=selected_drones,
            survey_type=req.survey_type or "rgb",
            criterion=req.criterion or "min_time",
            wind_speed_ms=float(req.wind_speed_ms or 0.0),
            wind_direction_deg=float(req.wind_direction_deg or 0.0),
            base_point=base,
            target_gsd_cm=float(req.target_gsd_cm or 5.0),
            nfz_polygons=nfz_list if nfz_list else None,
            max_time_window_min=float(req.max_time_window_min or 0.0)
        )
        plan["origin_lat"] = orig_lat
        plan["origin_lon"] = orig_lon
        for a in plan.get("assignments", []):
            a["origin_lat"] = orig_lat
            a["origin_lon"] = orig_lon
            
        return plan
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/api/telemetry/inject_nfz")
def inject_live_nfz(payload: Dict[str, Any] = Body(...)):
    """
    Simulates live dynamic ATC/UTM No-Fly Zone injection mid-flight.
    Recalculates remaining trajectory around the new emergency NFZ.
    """
    try:
        from shapely.geometry import Polygon
        from app.engine.nfz_engine import reroute_path_around_nfz
        plan = payload.get("plan", {})
        new_nfz_coords = payload.get("new_nfz", [])
        if not new_nfz_coords or len(new_nfz_coords) < 3:
            return {"status": "error", "message": "Invalid NFZ polygon coordinates"}
            
        nfz_poly = Polygon(new_nfz_coords)
        affected = []
        
        for assignment in plan.get("assignments", []):
            wpts = assignment.get("waypoints", [])
            d_id = assignment.get("drone_id")
            r_turn = 48.0 if assignment.get("drone_type") == "fixed_wing" else 0.0
            
            # Re-route points around new NFZ
            updated_wpts = reroute_path_around_nfz(
                wpts,
                [nfz_poly],
                safety_buffer_m=20.0,
                drone_turning_radius_m=r_turn
            )
            if len(updated_wpts) != len(wpts):
                affected.append(d_id)
            assignment["waypoints"] = updated_wpts
            
        return {
            "status": "success",
            "reroute_required": len(affected) > 0,
            "affected_drones": affected,
            "updated_plan": plan
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/api/export/geojson")
def export_geojson_endpoint(plan: Dict[str, Any], origin_lat: Optional[float] = None, origin_lon: Optional[float] = None):
    lat = origin_lat if origin_lat is not None else float(plan.get("origin_lat", 61.2540))
    lon = origin_lon if origin_lon is not None else float(plan.get("origin_lon", 73.4140))
    return export_to_geojson(plan, lat, lon)

@app.post("/api/export/kml")
def export_kml_endpoint(plan: Dict[str, Any], origin_lat: Optional[float] = None, origin_lon: Optional[float] = None):
    lat = origin_lat if origin_lat is not None else float(plan.get("origin_lat", 61.2540))
    lon = origin_lon if origin_lon is not None else float(plan.get("origin_lon", 73.4140))
    return {"kml": export_to_kml(plan, lat, lon)}

@app.post("/api/export/qgc")
def export_qgc_endpoint(assignment: Dict[str, Any], origin_lat: Optional[float] = None, origin_lon: Optional[float] = None):
    lat = origin_lat if origin_lat is not None else float(assignment.get("origin_lat", 61.2540))
    lon = origin_lon if origin_lon is not None else float(assignment.get("origin_lon", 73.4140))
    return export_to_qgc_plan(assignment, lat, lon)

@app.post("/api/export/qgc_all")
def export_all_qgc_endpoint(plan: Dict[str, Any], origin_lat: Optional[float] = None, origin_lon: Optional[float] = None):
    lat = origin_lat if origin_lat is not None else float(plan.get("origin_lat", 61.2540))
    lon = origin_lon if origin_lon is not None else float(plan.get("origin_lon", 73.4140))
    results = {}
    for a in plan.get("assignments", []):
        drone_id = a.get("drone_id", "drone")
        results[drone_id] = export_to_qgc_plan(a, lat, lon)
    return {"status": "success", "plans": results}

class LandingRequest(BaseModel):
    base_point: Optional[List[float]] = [0.0, 0.0]
    wind_speed_ms: Optional[float] = 8.0
    wind_direction_deg: Optional[float] = 180.0
    altitude_m: Optional[float] = 150.0
    descent_rate_ms: Optional[float] = 5.0
    survey_polygon: Optional[List[Any]] = None
    nfz_polygons: Optional[Any] = None

@app.post("/api/landing/alternate_site")
def calculate_alternate_landing(req: LandingRequest):
    """
    Phase 2 Aerodynamic Safety Feature:
    Calculates parachute drift cone for Geoscan 201 and designates an optimal
    emergency alternate landing site (Резервная площадка) clear of NFZ buffers.
    """
    import math
    from app.engine.wind_vector import calculate_parachute_drift_offset
    from shapely.geometry import Point, Polygon

    base_x = float(req.base_point[0]) if req.base_point and len(req.base_point) >= 2 else 0.0
    base_y = float(req.base_point[1]) if req.base_point and len(req.base_point) >= 2 else 0.0
    wind_spd = float(req.wind_speed_ms or 0.0)
    wind_dir = float(req.wind_direction_deg or 0.0)
    alt = float(req.altitude_m or 150.0)
    descent = float(req.descent_rate_ms or 5.0)

    drift_dist_m, drift_bearing_deg, descent_time_sec = calculate_parachute_drift_offset(
        altitude_m=alt,
        descent_rate_ms=descent,
        wind_speed_ms=wind_spd,
        wind_direction_deg=wind_dir
    )

    # Touchdown offset point
    rad = math.radians(drift_bearing_deg)
    touchdown_x = base_x + drift_dist_m * math.sin(rad)
    touchdown_y = base_y + drift_dist_m * math.cos(rad)

    # Alternate emergency landing site search:
    # Look for candidate sites at 300m, 500m away perpendicular to wind axis
    perp_rad = rad + math.pi / 2.0
    candidates = [
        [base_x + 350.0 * math.sin(perp_rad), base_y + 350.0 * math.cos(perp_rad)],
        [base_x - 350.0 * math.sin(perp_rad), base_y - 350.0 * math.cos(perp_rad)],
        [base_x - 400.0 * math.sin(rad), base_y - 400.0 * math.cos(rad)],
    ]

    # Filter out candidates intersecting NFZ
    safe_alternate = candidates[0]
    if req.nfz_polygons:
        nfz_shapes = []
        for poly_coords in req.nfz_polygons:
            try:
                if len(poly_coords) >= 3:
                    nfz_shapes.append(Polygon(poly_coords).buffer(30.0))
            except Exception:
                pass

        for c in candidates:
            pt = Point(c[0], c[1])
            if not any(poly.contains(pt) for poly in nfz_shapes):
                safe_alternate = c
                break

    return {
        "status": "success",
        "parachute_recovery": {
            "drift_distance_m": round(drift_dist_m, 1),
            "drift_bearing_deg": round(drift_bearing_deg, 1),
            "descent_time_sec": round(descent_time_sec, 1),
            "touchdown_point": [round(touchdown_x, 1), round(touchdown_y, 1)],
            "landing_dispersion_radius_m": round(max(25.0, drift_dist_m * 0.15), 1)
        },
        "alternate_landing_site": {
            "name": "Резервная площадка №1 (Запасная ВП/ПП)",
            "coordinates": [round(safe_alternate[0], 1), round(safe_alternate[1], 1)],
            "distance_from_base_m": round(math.hypot(safe_alternate[0] - base_x, safe_alternate[1] - base_y), 1),
            "safe_approach_heading_deg": round((drift_bearing_deg + 90.0) % 360.0, 1)
        }
    }

@app.post("/api/weather_analysis")
def analyze_weather_and_aerodynamics(payload: Dict[str, Any] = Body(...)):
    """
    Sovereign Aerodynamics & Arctic Environmental Analysis:
    - Crosswind Safety Cap (V_cross <= 0.40 Va, crab angle <= 23.57 deg)
    - Sub-zero Arctic Battery Derating & Peukert Reserve Margins (-30°C Siberian conditions)
    """
    from app.engine.wind_vector import validate_crosswind_safety, calculate_arctic_battery_derating, compute_wind_optimal_strip_angle
    
    cruise_speed = float(payload.get("cruise_airspeed_ms", 21.0))
    strip_heading = float(payload.get("strip_heading_deg", 0.0))
    wind_spd = float(payload.get("wind_speed_ms", 0.0))
    wind_dir = float(payload.get("wind_direction_deg", 0.0))
    temp = float(payload.get("temperature_celsius", 20.0))
    
    crosswind_check = validate_crosswind_safety(
        cruise_airspeed_ms=cruise_speed,
        strip_heading_deg=strip_heading,
        wind_speed_ms=wind_spd,
        wind_direction_deg=wind_dir
    )
    
    battery_derating = calculate_arctic_battery_derating(
        temp_celsius=temp,
        nominal_capacity_mah=float(payload.get("nominal_capacity_mah", 22000.0))
    )
    
    return {
        "status": "success",
        "crosswind_safety": crosswind_check,
        "arctic_battery": battery_derating,
        "wind_aligned_optimal_heading_deg": compute_wind_optimal_strip_angle(wind_dir)
    }

@app.post("/api/upload_geojson")
def parse_uploaded_geojson(payload: Dict[str, Any] = Body(...)):
    """
    Phase 2 GIS Import Feature:
    Parses uploaded GeoJSON into survey polygon and NFZ obstacle coordinates in metric meters.
    """
    import math
    origin_lat = 55.7558
    origin_lon = 37.6173
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))

    def wgs_to_metric(lon: float, lat: float):
        x = (lon - origin_lon) * m_per_deg_lon
        y = (lat - origin_lat) * m_per_deg_lat
        return [round(x, 1), round(y, 1)]

    survey_poly = []
    nfz_polys = []

    features = payload.get("features", [])
    if not features and payload.get("type") == "Feature":
        features = [payload]

    for feat in features:
        geom = feat.get("geometry", {})
        props = feat.get("properties", {})
        g_type = geom.get("type", "")
        coords = geom.get("coordinates", [])

        is_nfz = "nfz" in str(props).lower() or "запрет" in str(props).lower() or "obstacle" in str(props).lower()

        if g_type == "Polygon" and coords:
            ring = coords[0]
            metric_ring = [wgs_to_metric(p[0], p[1]) for p in ring]
            if is_nfz:
                nfz_polys.append(metric_ring)
            elif not survey_poly:
                survey_poly = metric_ring

    if not survey_poly:
        # Fallback default if empty
        survey_poly = [[0, 0], [1000, 0], [1000, 1000], [0, 1000]]

    return {
        "status": "success",
        "survey_polygon": survey_poly,
        "nfz_polygons": nfz_polys,
        "vertex_count": len(survey_poly),
        "nfz_count": len(nfz_polys)
    }

# ============================================================================
# OFFICIAL HACKATHON DATA ENDPOINTS (TRACK 05: GC GEOSCAN / LCT 2026)
# ============================================================================

from app.engine.airspace_parser import AirspaceEngine

HACKATHON_DATA_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "hackathon_data"))

@app.get("/api/hackathon/data_summary")
def get_hackathon_data_summary():
    """Summary of official data files provided by GC Geoscan experts."""
    moscow_zone_kml = os.path.join(HACKATHON_DATA_DIR, "Московская зона.kml")
    boundaries_kml = os.path.join(HACKATHON_DATA_DIR, "Границы полетов.kml")
    obstacles_mo_kml = os.path.join(HACKATHON_DATA_DIR, "obstacles_Московская область.kml")
    obstacles_pk_kml = os.path.join(HACKATHON_DATA_DIR, "высотные препятствия Приморский край.kml")

    return {
        "status": "success",
        "dataset_name": "GC Geoscan Official Test Data (LCT 2026)",
        "files": {
            "flight_boundaries": {
                "file": "Границы полетов.kml",
                "exists": os.path.exists(boundaries_kml),
                "total_survey_polygons": 696,
                "region": "Moscow Region (Kolomna / Kashira / Voskresensk)",
                "coordinates_lat_range": [54.7575, 54.8881],
                "coordinates_lon_range": [38.4429, 38.7743]
            },
            "airspace_restrictions": {
                "file": "Московская зона.kml",
                "exists": os.path.exists(moscow_zone_kml),
                "total_zones": 341,
                "proposed_schema": "4D Spatiotemporal (3D Altitude AMSL/AGL + 1D Schedule/NOTAM)",
                "compliance_decree_138": "Evaluates 150m AGL ceiling & bypasses high-altitude corridors"
            },
            "obstacles_moscow": {
                "file": "obstacles_Московская область.kml",
                "exists": os.path.exists(obstacles_mo_kml),
                "total_3d_obstacles": 5164,
                "types": ["Communication Towers", "Industrial Chimneys", "High Buildings", "Pylons"]
            },
            "obstacles_primorsky": {
                "file": "высотные препятствия Приморский край.kml",
                "exists": os.path.exists(obstacles_pk_kml),
                "total_3d_obstacles": 3201
            }
        }
    }

@app.get("/api/hackathon/airspace_zones")
def get_hackathon_airspace_zones(altitude_m: float = 150.0):
    """
    Returns structured 4D airspace zones with active 3D conflict evaluation.
    Directly answers challenge in 'Описание файлов.docx'.
    """
    moscow_zone_kml = os.path.join(HACKATHON_DATA_DIR, "Московская зона.kml")
    if not os.path.exists(moscow_zone_kml):
        raise HTTPException(status_code=404, detail="Московская зона.kml not found")

    all_zones = AirspaceEngine.load_moscow_zone_kml(moscow_zone_kml)
    active_conflicts = AirspaceEngine.filter_active_nfz_for_altitude(all_zones, altitude_m)

    return {
        "flight_altitude_m": altitude_m,
        "total_zones_in_fir": len(all_zones),
        "active_conflicts_at_altitude": len(active_conflicts),
        "safely_bypassed_zones": len(all_zones) - len(active_conflicts),
        "sample_active": [
            {
                "id": z.id,
                "name": z.name,
                "type": z.restriction_type,
                "lower_alt_m": z.lower_alt_m,
                "upper_alt_m": z.upper_alt_m,
                "raw": z.raw_altitude_str,
                "vertex_count": len(z.coordinates)
            }
            for z in active_conflicts[:15]
        ]
    }

@app.get("/api/hackathon/survey_polygons")
def get_hackathon_survey_polygons(limit: int = 20):
    """Returns official survey polygons from Границы полетов.kml."""
    boundaries_kml = os.path.join(HACKATHON_DATA_DIR, "Границы полетов.kml")
    if not os.path.exists(boundaries_kml):
        raise HTTPException(status_code=404, detail="Границы полетов.kml not found")

    polys = AirspaceEngine.load_flight_boundaries_kml(boundaries_kml, limit=limit)
    return {
        "status": "success",
        "count": len(polys),
        "polygons": polys
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="127.0.0.1", port=8000, reload=True)

