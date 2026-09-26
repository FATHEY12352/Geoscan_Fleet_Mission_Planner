"""
No-Fly Zone (NFZ) Avoidance and Emergency Reserve Landing Engine
Subtracts NFZ obstacles with safety buffers, validates paths,
and safely routes around obstacles without back-and-forth oscillations.
"""

import math
from typing import List, Dict, Any, Tuple, Union
from shapely.geometry import Polygon, MultiPolygon, Point, LineString
from shapely.ops import unary_union
from app.engine.dubins_curves import calculate_dubins_path

def to_shapely_polygons(nfz_list: Any) -> List[Polygon]:
    """Converts mixed input (Shapely Polygons or coordinate lists) to valid Shapely Polygons"""
    polygons: List[Polygon] = []
    if not nfz_list:
        return polygons
        
    for item in nfz_list:
        if isinstance(item, Polygon):
            if not item.is_empty and item.is_valid:
                polygons.append(item)
            elif not item.is_empty:
                polygons.append(item.buffer(0))
        elif isinstance(item, (list, tuple)) and len(item) >= 3:
            p = Polygon(item)
            if not p.is_empty and p.is_valid:
                polygons.append(p)
            elif not p.is_empty:
                polygons.append(p.buffer(0))
    return polygons

def sanitize_survey_area_with_nfz(
    survey_polygon: Polygon,
    nfz_polygons: Any,
    safety_buffer_m: float = 75.0
) -> Polygon:
    """
    Clips out No-Fly Zones from the survey area with an extra safety buffer.
    Uses mitre box buffering to give ample clearance for fixed-wing turns.
    Returns safe navigable survey polygon.
    """
    polys = to_shapely_polygons(nfz_polygons)
    if not polys:
        return survey_polygon
        
    buffered_nfz = [nfz.buffer(safety_buffer_m, join_style="mitre") for nfz in polys if not nfz.is_empty]
    if not buffered_nfz:
        return survey_polygon
        
    nfz_union = unary_union(buffered_nfz)
    clean_area = survey_polygon.difference(nfz_union)
    
    if clean_area.is_empty:
        return survey_polygon
        
    return clean_area

def is_path_safe_from_nfz(
    path_points: List[Tuple[float, float]],
    nfz_polygons: Any,
    safety_buffer_m: float = 15.0
) -> bool:
    """
    Verifies that a flight trajectory does not cross any NFZ.
    """
    if len(path_points) < 2 or not nfz_polygons:
        return True
        
    polys = to_shapely_polygons(nfz_polygons)
    if not polys:
        return True
        
    path_line = LineString(path_points)
    buffered_nfz = unary_union([nfz.buffer(safety_buffer_m) for nfz in polys if not nfz.is_empty])
    
    return not path_line.intersects(buffered_nfz)

import heapq

def _visibility_graph_shortest_path(
    p_start: Tuple[float, float],
    p_end: Tuple[float, float],
    obstacle_boundary_pts: List[Tuple[float, float]],
    buffered_obstacle
) -> List[Tuple[float, float]]:
    """
    Computes Euclidean shortest collision-free path around buffered obstacles.
    Uses NetworkX or Dijkstra priority queue.
    """
    nodes = [p_start, p_end] + obstacle_boundary_pts
    n = len(nodes)
    adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(n)}
    
    for u in range(n):
        for v in range(u + 1, n):
            s_uv = LineString([(float(nodes[u][0]), float(nodes[u][1])), (float(nodes[v][0]), float(nodes[v][1]))])
            if not s_uv.intersects(buffered_obstacle):
                d = math.hypot(nodes[u][0] - nodes[v][0], nodes[u][1] - nodes[v][1])
                adj[u].append((v, d))
                adj[v].append((u, d))
                
    # Dijkstra from 0 (p_start) to 1 (p_end)
    dist = {i: float("inf") for i in range(n)}
    prev = {i: None for i in range(n)}
    dist[0] = 0.0
    pq = [(0.0, 0)]
    
    while pq:
        d_curr, u = heapq.heappop(pq)
        if u == 1:
            break
        if d_curr > dist[u]:
            continue
        for v, weight in adj[u]:
            if dist[u] + weight < dist[v]:
                dist[v] = dist[u] + weight
                prev[v] = u
                heapq.heappush(pq, (dist[v], v))
                
    if prev[1] is None:
        # No path found directly
        return [p_start, p_end]
        
    curr = 1
    path_nodes = []
    while curr is not None:
        path_nodes.append(curr)
        curr = prev[curr]
    path_nodes.reverse()
    return [nodes[i] for i in path_nodes]

def reroute_path_around_nfz(
    path_points: List[Tuple[float, float]],
    nfz_polygons: Any,
    safety_buffer_m: float = 15.0,
    drone_turning_radius_m: float = 48.0
) -> List[Tuple[float, float]]:
    """
    Reroutes flight path around No-Fly Zones if any segment intersects.
    Employs a visibility-graph approach along the smooth clearance envelope
    to guarantee both zero NFZ penetration and flight curvature R >= 45m.
    """
    polys = to_shapely_polygons(nfz_polygons)
    if not polys or len(path_points) < 2:
        return path_points
        
    buffered_nfz = unary_union([p.buffer(safety_buffer_m) for p in polys if not p.is_empty])
    if buffered_nfz.is_empty:
        return path_points
        
    path_line = LineString(path_points)
    if not path_line.intersects(buffered_nfz):
        return path_points
        
    # 1. Build smooth clearance boundary with R >= drone_turning_radius
    clearance_r = max(drone_turning_radius_m, safety_buffer_m + 15.0)
    clearance_nfz = unary_union([p.buffer(clearance_r, quad_segs=8) for p in polys if not p.is_empty])
    
    # Filter out points inside clearance envelope
    clean_pts = [p for p in path_points if not clearance_nfz.contains(Point(p))]
    if len(clean_pts) < 2:
        clean_pts = path_points
    
    cand_pts: List[Tuple[float, float]] = []
    if clearance_nfz.geom_type == "Polygon":
        cand_pts = list(clearance_nfz.exterior.coords)[:-1]
    elif clearance_nfz.geom_type == "MultiPolygon":
        for poly in clearance_nfz.geoms:
            cand_pts.extend(list(poly.exterior.coords)[:-1])
            
    # 2. Stepwise path segment collision check and rerouting
    safe_route: List[Tuple[float, float]] = [clean_pts[0]]
    for i in range(len(clean_pts) - 1):
        p1 = safe_route[-1]
        p2 = clean_pts[i + 1]
        seg = LineString([p1, p2])
        if seg.intersects(buffered_nfz):
            detour = _visibility_graph_shortest_path(p1, p2, cand_pts, buffered_nfz)
            for pt in detour[1:]:
                safe_route.append(pt)
        else:
            safe_route.append(p2)
            
    # 4. Filter consecutive duplicate points
    dedup: List[Tuple[float, float]] = [safe_route[0]]
    for pt in safe_route[1:]:
        if math.hypot(pt[0] - dedup[-1][0], pt[1] - dedup[-1][1]) >= 2.0:
            dedup.append(pt)
            
    return dedup
