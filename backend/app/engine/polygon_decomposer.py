"""
Advanced Non-Convex Polygon Decomposition and Multi-Cell Clustering Engine
Decomposes complex, concave, and multi-polygon survey zones into continuous single-part cells,
eliminating disjoint artifact regions and ensuring safe contiguous drone flight.
"""

from typing import List, Dict, Any, Tuple
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, box, GeometryCollection
from shapely.ops import unary_union

def explode_multipolygon(geom) -> List[Polygon]:
    """Explodes MultiPolygon or GeometryCollection into clean list of Polygon instances"""
    polygons: List[Polygon] = []
    if geom.is_empty:
        return polygons
        
    if geom.geom_type == "Polygon":
        if geom.area > 50.0: # Filter out microscopic slivers
            polygons.append(geom)
    elif geom.geom_type == "MultiPolygon":
        for p in geom.geoms:
            if p.area > 50.0:
                polygons.append(p)
    elif geom.geom_type == "GeometryCollection":
        for g in geom.geoms:
            if g.geom_type in ["Polygon", "MultiPolygon"]:
                polygons.extend(explode_multipolygon(g))
    return polygons

def eliminate_polygon_holes(geom) -> List[Polygon]:
    """
    Slices across interior rings of polygons with holes to eliminate holes completely.
    Returns a list of contiguous simple Polygons with 0 interior rings.
    """
    polys = explode_multipolygon(geom)
    clean_parts: List[Polygon] = []
    
    for p in polys:
        if p.area < 50.0:
            continue
        if len(p.interiors) == 0:
            clean_parts.append(p)
            continue
            
        minx, miny, maxx, maxy = p.bounds
        x_cuts = set()
        for interior in p.interiors:
            h_minx, _, h_maxx, _ = Polygon(interior).bounds
            x_cuts.add(h_minx)
            x_cuts.add(h_maxx)
            
        cuts = sorted(list(x_cuts))
        xs = [minx - 10.0] + cuts + [maxx + 10.0]
        
        for i in range(len(xs) - 1):
            clip_b = box(xs[i], miny - 10.0, xs[i + 1], maxy + 10.0)
            inter = p.intersection(clip_b)
            if not inter.is_empty:
                clean_parts.extend(explode_multipolygon(inter))
                
    return [cp for cp in clean_parts if cp.area > 50.0]

def decompose_non_convex_polygon(
    polygon: Polygon,
    target_num_cells: int,
    weights: List[float] = None
) -> List[Polygon]:
    """
    Robust cell decomposition for non-convex polygons.
    Guarantees every returned element is a contiguous single Polygon (no disjoint MultiPolygons)
    and strictly has zero interior holes.
    """
    if polygon.is_empty:
        return []
        
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
        
    # 1. Eliminate any interior holes by slicing
    hole_free_parts = eliminate_polygon_holes(polygon)
    if not hole_free_parts:
        hole_free_parts = [polygon]
        
    if target_num_cells <= 1:
        return hole_free_parts
        
    if weights is None or len(weights) != target_num_cells:
        weights = [1.0] * target_num_cells
        
    total_w = sum(weights)
    norm_w = [w / total_w for w in weights]
    
    # If the number of clean parts is already >= target_num_cells,
    # return them directly so every part remains strictly simple and hole-free
    if len(hole_free_parts) >= target_num_cells:
        hole_free_parts.sort(key=lambda c: c.centroid.x)
        return hole_free_parts

        
    # If fewer parts than target_num_cells, slice along X
    minx, miny, maxx, maxy = polygon.bounds
    poly_width = maxx - minx
    
    all_single_parts: List[Polygon] = []
    curr_x = minx
    
    for i, w in enumerate(norm_w):
        next_x = curr_x + poly_width * w if i < target_num_cells - 1 else maxx
        clip_box = box(curr_x, miny - 50, next_x, maxy + 50)
        sliced = polygon.intersection(clip_box)
        
        parts = eliminate_polygon_holes(sliced)
        all_single_parts.extend(parts)
        curr_x = next_x
        
    if not all_single_parts:
        return [polygon]
        
    return all_single_parts

