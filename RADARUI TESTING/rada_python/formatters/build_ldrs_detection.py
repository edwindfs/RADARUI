"""Build standard LDRS detection dictionary from raw track and radar position."""

from datetime import datetime, timezone
import math
import time
from typing import Any, Dict, Optional

from ..utils.geo import bearing_degrees, distance_from_radar
from ..utils.direction import get_direction_from_bearing


def format_active_time(ms_since_start: float) -> str:
    """Format active time duration string."""
    sec = max(0, int(ms_since_start // 1000))
    if sec < 60:
        return f"{sec}s"
    minutes = sec // 60
    rem = sec % 60
    return f"{minutes}min" if rem == 0 else f"{minutes}min {rem}s"


def threat_from_distance_m(m: float) -> str:
    """Determine threat level based on ground distance."""
    return "high" if m < 2000 else "low"


def normalized_position(p: Any) -> Dict[str, float]:
    """Normalize radar position object/dict into standard {latitude, longitude, altitude}."""
    if isinstance(p, dict):
        lat = p.get("latitude")
        if lat is None:
            lat = p.get("latDeg")
        if lat is None:
            lat = p.get("lat")

        lon = p.get("longitude")
        if lon is None:
            lon = p.get("lonDeg")
        if lon is None:
            lon = p.get("lon")

        alt = p.get("altitude")
        if alt is None:
            alt = p.get("altM")
        if alt is None:
            alt = p.get("alt", 0)
    else:
        lat = getattr(p, "latitude", None) or getattr(p, "latDeg", None) or getattr(p, "lat", 0)
        lon = getattr(p, "longitude", None) or getattr(p, "lonDeg", None) or getattr(p, "lon", 0)
        alt = getattr(p, "altitude", None) or getattr(p, "altM", None) or getattr(p, "alt", 0)

    return {
        "latitude": float(lat or 0.0),
        "longitude": float(lon or 0.0),
        "altitude": float(alt or 0.0),
    }


def classification_from_family(target_family: Optional[int]) -> str:
    """Map LDRS target family integer to classification name."""
    mapping = {
        0: "Unclassified",
        1: "Unknown",
        2: "ABT/UAV",
        3: "RAM",
        4: "SHORAD",
    }
    return mapping.get(target_family, "Unknown")


def build_ldrs_detection(
    track: Dict[str, Any],
    radar_position: Any,
    started_at_ms: Optional[float] = None,
) -> Dict[str, Any]:
    """Build LDRS detection dictionary matching formatters/buildLdrsDetection.js."""
    now_ms = time.time() * 1000.0
    if started_at_ms is None:
        started_at_ms = now_ms

    ts = track.get("timestamp") or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    radar = normalized_position(radar_position)

    dfr = distance_from_radar(radar, track)
    radar_to_drone_bearing = bearing_degrees(radar, track)

    velocity = track.get("velocity")
    if velocity is not None and not math.isfinite(float(velocity)):
        velocity = None
    elif velocity is not None:
        velocity = float(velocity)

    slant_dist = round(dfr["slantDistance_m"])
    ground_dist = dfr["groundDistance_m"]
    brg_round = round(radar_to_drone_bearing)

    track_alt = track.get("altitude", 0)
    alt_round = round(track_alt) if math.isfinite(float(track_alt)) else 0

    return {
        "radarType": "LDRS",
        "system": {
            "name": "LDRS RADAR",
            "status": "Active",
            "timestamp": ts,
            "activeTime": format_active_time(now_ms - started_at_ms),
        },
        "radarPosition": {
            "latituade": radar["latitude"],  # Preserve legacy spelling for FE compatibility
            "latitude": radar["latitude"],
            "longitude": radar["longitude"],
            "altitude": radar["altitude"],
        },
        "id": track.get("trackId"),
        "classification": classification_from_family(track.get("targetFamily")),
        "range": {"value": slant_dist, "unit": "m"},
        "velocity": {"value": velocity, "unit": "m/s"},
        "altitude": {"value": alt_round, "unit": "m"},
        "heading": {"value": None, "unit": "deg"},
        "climbRate": {"value": None, "unit": "m/s"},
        "flightPhase": None,
        "status": "tracked",
        "threatLevel": threat_from_distance_m(ground_dist),
        "bearing": {"value": brg_round, "unit": "deg"},
        "duration": {"value": None, "unit": "min"},
        "direction": get_direction_from_bearing(radar_to_drone_bearing),
        "azimuth": brg_round,
        "angle": None,
        "distance": round(ground_dist / 1000.0),
        "path": [],
        "metadata": {
            "version": "1.0",
            "dataSchema": "ldrs_cluster_v1",
            "units": {
                "distance": "meters",
                "range": "meters",
                "velocity": "meters_per_second",
                "altitude": "meters",
                "bearing": "degrees",
            },
            "ldrs": {
                "messageType": track.get("messageType"),
                "targetTypeRaw": track.get("targetTypeRaw"),
                "targetFamily": track.get("targetFamily"),
                "statusFlags": track.get("statusFlags"),
                "statusFlags2": track.get("statusFlags2"),
                "threatRaw": track.get("threatRaw"),
                "isValid": track.get("isValid"),
                "isTentative": track.get("isTentative"),
                "isHighPriority": track.get("isHighPriority"),
                "isValidTarget": track.get("isValidTarget"),
            },
        },
    }
