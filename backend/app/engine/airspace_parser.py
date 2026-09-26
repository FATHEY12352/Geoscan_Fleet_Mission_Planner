"""
GEOSCAN AIRSPACE RESTRICTION & 3D OBSTACLE PARSER (4D AIRSPACE MODEL)
Official Geoscan Hackathon (LCT 2026) Airspace Schema & Ingestion Engine
"""

from __future__ import annotations
import os
import re
import xml.etree.ElementTree as ET
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass, asdict


@dataclass
class TimeWindow:
    """Standard ISO 8601 schedule / NOTAM active time window."""
    start_utc: Optional[str] = None
    end_utc: Optional[str] = None
    days_of_week: Optional[List[int]] = None  # 1 = Mon, 7 = Sun
    is_permanent: bool = False


@dataclass
class AirspaceRestriction:
    """
    Standard proposed format for 4D Airspace Restrictions
    Addresses the challenge in 'Описание файлов.docx':
    'В файле нет стандартного формата записи данных о диапазоне высот и времени действия ограничения. Предложите свой.'
    """
    id: str
    name: str
    restriction_type: str  # 'PROHIBITED' (запретная_зона), 'RESTRICTED' (врем_ограничение), 'DANGER' (опасная)
    lower_alt_m: float     # Lower altitude in meters AMSL
    upper_alt_m: float     # Upper altitude in meters AMSL
    is_surface: bool       # True if restriction starts from ground level (GND)
    raw_altitude_str: str  # Original Russian AIP string
    time_windows: List[TimeWindow]
    coordinates: List[List[float]]  # [[lon, lat], ...]
    is_3d_active: bool = True       # Evaluated dynamically against drone flight altitude


@dataclass
class Obstacle3D:
    """High-altitude obstacle (towers, chimneys, masts, high buildings)."""
    id: str
    name: str
    obstacle_type: str
    height_amsl_m: float
    coordinates: List[List[float]]  # footprint coordinates
    center_lon: float
    center_lat: float


class AirspaceEngine:
    """Parses, structures, and manages official Geoscan airspace and obstacle data."""

    KML_NS = '{http://www.opengis.net/kml/2.2}'

    @staticmethod
    def parse_russian_altitude_string(alt_str: str) -> Tuple[float, float, bool]:
        """
        Converts human-readable Russian AIP altitude strings into structured meters.
        
        Handles:
          - 'От земли до 900 м (3000 фут) AMSL' -> (0.0, 900.0, True)
          - 'От 800 м (2700 фут) AMSL до FL90' -> (800.0, 2743.2, False)
          - 'От FL280 до FL400' -> (8534.4, 12192.0, False)
          - 'От земли до FL160' -> (0.0, 4876.8, True)
          - 'От земли до 450 м (1500 фут) AMSL' -> (0.0, 450.0, True)
        """
        if not alt_str:
            return 0.0, 10000.0, True

        s = alt_str.lower().replace('\xa0', ' ')
        is_surface = ('от земли' in s or 'gnd' in s or 'земли' in s)
        lower_m = 0.0 if is_surface else 0.0
        upper_m = 10000.0

        # Flight Level matching (FL100 = 10,000 ft = 3048 m)
        fl_matches = re.findall(r'fl\s*(\d+)', s)
        
        # Lower bound parsing
        from_m = re.search(r'от\s+(\d+)\s*м', s)
        if from_m:
            lower_m = float(from_m.group(1))
            is_surface = False
        elif 'от fl' in s and fl_matches:
            lower_m = float(fl_matches[0]) * 30.48
            is_surface = False

        # Upper bound parsing
        to_m = re.search(r'до\s+(\d+)\s*м', s)
        if to_m:
            upper_m = float(to_m.group(1))
        elif 'до fl' in s:
            fl_after = re.search(r'до\s+fl\s*(\d+)', s)
            if fl_after:
                upper_m = float(fl_after.group(1)) * 30.48

        return round(lower_m, 1), round(upper_m, 1), is_surface

    @classmethod
    def load_moscow_zone_kml(cls, file_path: str) -> List[AirspaceRestriction]:
        """Loads and structures all 341 restricted zones from Московская зона.kml."""
        if not os.path.exists(file_path):
            return []

        tree = ET.parse(file_path)
        root = tree.getroot()
        restrictions = []

        for pm in root.findall(f'.//{cls.KML_NS}Placemark'):
            ext = pm.find(f'{cls.KML_NS}ExtendedData')
            data_map = {}
            if ext is not None:
                for d in ext.findall(f'.//{cls.KML_NS}Data'):
                    val = d.find(f'{cls.KML_NS}value')
                    data_map[d.get('name')] = val.text if val is not None else ''

            p_name = data_map.get('Name', '')
            raw_alt = data_map.get('Altitudes', '')
            raw_type = data_map.get('Type', '')

            # Normalize type
            rest_type = 'PROHIBITED' if 'запрет' in raw_type else 'RESTRICTED'
            lower_m, upper_m, is_surf = cls.parse_russian_altitude_string(raw_alt)

            # Coordinates
            coords_pts = []
            coords_node = pm.find(f'.//{cls.KML_NS}coordinates')
            if coords_node is not None and coords_node.text:
                for pt in coords_node.text.strip().split():
                    parts = pt.split(',')
                    if len(parts) >= 2:
                        try:
                            coords_pts.append([float(parts[0]), float(parts[1])])
                        except ValueError:
                            pass

            if coords_pts:
                # Default permanent or NOTAM active schedule
                time_win = TimeWindow(is_permanent=('запрет' in raw_type))
                restrictions.append(
                    AirspaceRestriction(
                        id=p_name or f"ZONE_{len(restrictions)+1}",
                        name=p_name,
                        restriction_type=rest_type,
                        lower_alt_m=lower_m,
                        upper_alt_m=upper_m,
                        is_surface=is_surf,
                        raw_altitude_str=raw_alt,
                        time_windows=[time_win],
                        coordinates=coords_pts,
                    )
                )

        return restrictions

    @classmethod
    def filter_active_nfz_for_altitude(
        cls, restrictions: List[AirspaceRestriction], flight_alt_m: float = 150.0
    ) -> List[AirspaceRestriction]:
        """
        CRITICAL HACKATHON DIFFERENTIATOR:
        Evaluates 3D vertical conflicts per Russian Airspace Regulation №138.
        If the restriction's lower altitude is strictly above the drone's flight altitude
        (e.g., restriction is 800m - FL90, and drone is at 150m),
        the drone safely flies underneath without any violation!
        """
        active_conflicts = []
        for r in restrictions:
            # Active conflict if flight altitude falls between lower and upper bounds
            # Or if it starts from ground and upper bound is above flight altitude
            if r.lower_alt_m <= flight_alt_m <= r.upper_alt_m:
                r.is_3d_active = True
                active_conflicts.append(r)
            else:
                r.is_3d_active = False

        return active_conflicts

    @classmethod
    def load_flight_boundaries_kml(cls, file_path: str, limit: int = 50) -> List[Dict[str, Any]]:
        """Loads official survey boundary polygons from Границы полетов.kml."""
        if not os.path.exists(file_path):
            return []

        tree = ET.parse(file_path)
        root = tree.getroot()
        polygons = []

        for idx, pm in enumerate(root.findall(f'.//{cls.KML_NS}Placemark')):
            if limit and idx >= limit:
                break

            fid = f"FID_{idx+1}"
            ext = pm.find(f'{cls.KML_NS}ExtendedData')
            if ext is not None:
                for sd in ext.findall(f'.//{cls.KML_NS}SimpleData'):
                    if sd.get('name') == 'FID' and sd.text:
                        fid = sd.text

            coords_pts = []
            coords_node = pm.find(f'.//{cls.KML_NS}coordinates')
            if coords_node is not None and coords_node.text:
                for pt in coords_node.text.strip().split():
                    parts = pt.split(',')
                    if len(parts) >= 2:
                        try:
                            coords_pts.append([float(parts[0]), float(parts[1])])
                        except ValueError:
                            pass

            if len(coords_pts) >= 3:
                polygons.append({
                    "id": fid,
                    "index": idx,
                    "vertex_count": len(coords_pts),
                    "coordinates": coords_pts,
                    "centroid": [
                        sum(p[0] for p in coords_pts) / len(coords_pts),
                        sum(p[1] for p in coords_pts) / len(coords_pts),
                    ]
                })

        return polygons

    @classmethod
    def load_obstacles_kml(cls, file_path: str, limit: int = 100) -> List[Obstacle3D]:
        """Loads 3D high-altitude obstacle towers/structures from KML."""
        if not os.path.exists(file_path):
            return []

        tree = ET.parse(file_path)
        root = tree.getroot()
        obstacles = []

        for idx, pm in enumerate(root.findall(f'.//{cls.KML_NS}Placemark')):
            if limit and idx >= limit:
                break

            name_node = pm.find(f'{cls.KML_NS}name')
            p_name = name_node.text if name_node is not None else f"OBSTACLE_{idx+1}"

            coords_pts = []
            alts = []
            coords_node = pm.find(f'.//{cls.KML_NS}coordinates')
            if coords_node is not None and coords_node.text:
                for pt in coords_node.text.strip().split():
                    parts = pt.split(',')
                    if len(parts) >= 3:
                        try:
                            lon, lat, alt = float(parts[0]), float(parts[1]), float(parts[2])
                            coords_pts.append([lon, lat])
                            alts.append(alt)
                        except ValueError:
                            pass

            if coords_pts:
                height = max(alts) if alts else 50.0
                c_lon = sum(p[0] for p in coords_pts) / len(coords_pts)
                c_lat = sum(p[1] for p in coords_pts) / len(coords_pts)
                obstacles.append(
                    Obstacle3D(
                        id=f"OBS_{idx+1}",
                        name=p_name,
                        obstacle_type="TOWER" if "TOWER" in p_name else "BUILDING",
                        height_amsl_m=round(height, 1),
                        coordinates=coords_pts,
                        center_lon=round(c_lon, 6),
                        center_lat=round(c_lat, 6),
                    )
                )

        return obstacles
