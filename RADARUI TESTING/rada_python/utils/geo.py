"""Geographic calculation utilities for RADA LDRS radar."""

import math


def to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return (deg * math.pi) / 180.0


def get_latitude(p) -> float:
    """Extract latitude from dict or object."""
    if isinstance(p, dict):
        val = p.get("latitude")
        if val is None:
            val = p.get("latDeg")
        if val is None:
            val = p.get("lat")
        return float(val) if val is not None else 0.0
    for attr in ("latitude", "latDeg", "lat"):
        if hasattr(p, attr):
            val = getattr(p, attr)
            if val is not None:
                return float(val)
    return 0.0


def get_longitude(p) -> float:
    """Extract longitude from dict or object."""
    if isinstance(p, dict):
        val = p.get("longitude")
        if val is None:
            val = p.get("lonDeg")
        if val is None:
            val = p.get("lon")
        return float(val) if val is not None else 0.0
    for attr in ("longitude", "lonDeg", "lon"):
        if hasattr(p, attr):
            val = getattr(p, attr)
            if val is not None:
                return float(val)
    return 0.0


def get_altitude(p) -> float:
    """Extract altitude from dict or object, default 0.0."""
    if isinstance(p, dict):
        val = p.get("altitude")
        if val is None:
            val = p.get("altM")
        if val is None:
            val = p.get("alt")
        return float(val) if val is not None else 0.0
    for attr in ("altitude", "altM", "alt"):
        if hasattr(p, attr):
            val = getattr(p, attr)
            if val is not None:
                return float(val)
    return 0.0


def haversine_meters(a, b) -> float:
    """Compute ground distance between points a and b in meters."""
    r = 6371000.0  # Earth radius in meters
    lat1 = to_rad(get_latitude(a))
    lat2 = to_rad(get_latitude(b))
    d_lat = lat2 - lat1
    d_lon = to_rad(get_longitude(b) - get_longitude(a))

    s = (
        math.sin(d_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
    )

    return 2.0 * r * math.asin(math.sqrt(s))


def bearing_degrees(from_pt, to_pt) -> float:
    """Compute initial bearing from from_pt to to_pt in degrees [0, 360)."""
    lat1 = to_rad(get_latitude(from_pt))
    lat2 = to_rad(get_latitude(to_pt))
    d_lon = to_rad(get_longitude(to_pt) - get_longitude(from_pt))

    y = math.sin(d_lon) * math.cos(lat2)
    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    )

    return ((math.atan2(y, x) * 180.0) / math.pi + 360.0) % 360.0


def distance_from_radar(radar, target) -> dict:
    """Calculate ground and slant distance from radar to target in meters."""
    ground = haversine_meters(radar, target)
    d_alt = float(get_altitude(target)) - float(get_altitude(radar))
    return {
        "groundDistance_m": ground,
        "slantDistance_m": math.sqrt(ground * ground + d_alt * d_alt),
    }
