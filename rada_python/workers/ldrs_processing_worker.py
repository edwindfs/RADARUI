"""LDRS ICD message processing, coordinate transformations, and track decoding."""

from datetime import datetime, timezone
import math
import os
import struct
import time
from typing import Any, Dict, List, Optional, Tuple

from ..ws.ldrs_binary_protocol import create_tracks_frame

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
EARTH_RADIUS_M = 6371000.0
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
NS_PER_MS = 1_000_000


class LdrsProcessorConfig:
    """Configuration options for LDRS processing worker."""

    def __init__(self):
        self.msg_tracks = int(os.environ.get("MSG_TRACKS", 10))
        self.msg_tracks_extended = int(os.environ.get("MSG_TRACKS_EXTENDED", 20))
        self.msg_status = int(os.environ.get("MSG_STATUS", 7))
        self.msg_extended_status = int(
            os.environ.get("MSG_EXTENDED_STATUS")
            or os.environ.get("MSG_EXT_STATUS_SINGLE")
            or 12
        )
        self.msg_extended_status_mrs = int(
            os.environ.get("MSG_EXT_STATUS_MRS")
            or os.environ.get("MSG_EXT_STATUS")
            or 19
        )
        self.tracks_base_offset = int(os.environ.get("TRACKS_BASE_OFFSET", 48))
        self.track_slot_size = int(os.environ.get("TRACK_SLOT_SIZE", 232))
        self.track_lat_offset = int(os.environ.get("TRACK_LAT_OFFSET", 48))
        self.track_lon_offset = int(os.environ.get("TRACK_LON_OFFSET", 56))
        self.track_alt_offset = int(os.environ.get("TRACK_ALT_OFFSET", 64))
        self.track_doppler_offset = int(os.environ.get("TRACK_DOPPLER_OFFSET", 68))
        self.track_velocity_x_offset = int(os.environ.get("TRACK_VELOCITY_X_OFFSET", 156))
        self.track_velocity_y_offset = int(os.environ.get("TRACK_VELOCITY_Y_OFFSET", 160))
        self.track_velocity_z_offset = int(os.environ.get("TRACK_VELOCITY_Z_OFFSET", 164))
        self.track_status_flags_offset = int(os.environ.get("TRACK_STATUS_FLAGS_OFFSET", 120))
        self.only_valid_targets = bool(os.environ.get("LDRS_ONLY_VALID_TARGETS", "").lower() in ("1", "true", "yes"))
        self.include_enu = bool(os.environ.get("XCR1_TRACK_INCLUDE_ENU", "true").lower() in ("1", "true", "yes"))
        self.rebase_to_live_radar = bool(os.environ.get("LDRS_REBASE_TO_LIVE_RADAR", "").lower() in ("1", "true", "yes"))


config = LdrsProcessorConfig()
track_timings: Dict[str, Dict[str, Any]] = {}


def to_rad(deg: float) -> float:
    return deg * DEG2RAD


def timestamp_ms_to_ns(timestamp_ms: Optional[float]) -> int:
    if timestamp_ms is None or not math.isfinite(timestamp_ms):
        return int(time.time() * 1000.0) * NS_PER_MS
    return int(max(0.0, math.trunc(timestamp_ms))) * NS_PER_MS


def decode_tracks_time_tag_ms(msg: bytes) -> Optional[float]:
    """Decode year, month, day, hour, minute, second, millisecond from TRACKS header."""
    if len(msg) < 46:
        return None
    try:
        year, month, day, hour, minute, second, millisecond = struct.unpack_from(">7H", msg, 32)
        if (
            year < 1970
            or month < 1
            or month > 12
            or day < 1
            or day > 31
            or hour > 23
            or minute > 59
            or second > 59
            or millisecond > 999
        ):
            return None

        dt = datetime(year, month, day, hour, minute, second, millisecond * 1000, tzinfo=timezone.utc)
        return dt.timestamp() * 1000.0
    except Exception:
        return None


def lla_to_ecef(lla: Dict[str, float]) -> Dict[str, float]:
    """Convert WGS-84 LLA (lat, lon degrees, alt meters) to ECEF (meters)."""
    lat = to_rad(lla["latitude"])
    lon = to_rad(lla["longitude"])
    alt = float(lla["altitude"])

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    return {
        "x": (n + alt) * cos_lat * cos_lon,
        "y": (n + alt) * cos_lat * sin_lon,
        "z": (n * (1.0 - WGS84_E2) + alt) * sin_lat,
    }


def ecef_delta_to_enu(delta: Dict[str, float], origin: Dict[str, float]) -> Dict[str, float]:
    """Convert ECEF position delta to local ENU at origin."""
    lat = to_rad(origin["latitude"])
    lon = to_rad(origin["longitude"])

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    dx = delta["x"]
    dy = delta["y"]
    dz = delta["z"]

    return {
        "e": -sin_lon * dx + cos_lon * dy,
        "n": -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        "u": cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
    }


def enu_vector_to_ecef(vector: Dict[str, float], origin: Dict[str, float]) -> Dict[str, float]:
    """Convert local ENU vector to ECEF vector at origin."""
    lat = to_rad(origin["latitude"])
    lon = to_rad(origin["longitude"])

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    ve = vector["e"]
    vn = vector["n"]
    vu = vector["u"]

    return {
        "x": -sin_lon * ve - sin_lat * cos_lon * vn + cos_lat * cos_lon * vu,
        "y": cos_lon * ve - sin_lat * sin_lon * vn + cos_lat * sin_lon * vu,
        "z": cos_lat * vn + sin_lat * vu,
    }


def get_enu_m(radar: Optional[Dict[str, float]], target_ecef: Dict[str, float]) -> Dict[str, float]:
    """Calculate ENU coordinates of target relative to radar."""
    if not radar:
        return {"e": float("nan"), "n": float("nan"), "u": float("nan")}

    radar_ecef = lla_to_ecef(radar)
    return ecef_delta_to_enu(
        {
            "x": target_ecef["x"] - radar_ecef["x"],
            "y": target_ecef["y"] - radar_ecef["y"],
            "z": target_ecef["z"] - radar_ecef["z"],
        },
        radar,
    )


def enu_to_ecef_position(enu_m: Dict[str, float], origin: Dict[str, float]) -> Dict[str, float]:
    """Convert local ENU offset back to absolute ECEF position."""
    origin_ecef = lla_to_ecef(origin)
    delta = enu_vector_to_ecef(enu_m, origin)
    return {
        "x": origin_ecef["x"] + delta["x"],
        "y": origin_ecef["y"] + delta["y"],
        "z": origin_ecef["z"] + delta["z"],
    }


def get_ecef_velocity_mps(
    velocity: float, enu_m: Dict[str, float], radar: Optional[Dict[str, float]]
) -> Dict[str, float]:
    """Compute ECEF velocity vector from radial Doppler velocity."""
    if not radar or not math.isfinite(velocity):
        return {"x": float("nan"), "y": float("nan"), "z": float("nan")}

    range_m = math.hypot(enu_m["e"], enu_m["n"], enu_m["u"])
    if not math.isfinite(range_m) or range_m <= 0:
        return {"x": float("nan"), "y": float("nan"), "z": float("nan")}

    return enu_vector_to_ecef(
        {
            "e": (velocity * enu_m["e"]) / range_m,
            "n": (velocity * enu_m["n"]) / range_m,
            "u": (velocity * enu_m["u"]) / range_m,
        },
        radar,
    )


def nwu_vector_to_ecef(vector: Dict[str, float], origin: Optional[Dict[str, float]]) -> Dict[str, float]:
    """Convert North-West-Up vector to ECEF vector at origin."""
    if (
        not origin
        or not math.isfinite(vector.get("x", float("nan")))
        or not math.isfinite(vector.get("y", float("nan")))
        or not math.isfinite(vector.get("z", float("nan")))
    ):
        return {"x": float("nan"), "y": float("nan"), "z": float("nan")}

    # In NWU: x=North, y=West, z=Up. Thus ENU: e = -y, n = x, u = z.
    return enu_vector_to_ecef(
        {
            "e": -vector["y"],
            "n": vector["x"],
            "u": vector["z"],
        },
        origin,
    )


def has_finite_vector(vector: Dict[str, float]) -> bool:
    return (
        math.isfinite(vector.get("x", float("nan")))
        and math.isfinite(vector.get("y", float("nan")))
        and math.isfinite(vector.get("z", float("nan")))
    )


def get_track_timing(track_id: int, last_update_time_ns: int) -> Dict[str, Any]:
    key = str(track_id)
    timing = track_timings.get(key)
    if not timing:
        timing = {
            "acquiredTimeNs": last_update_time_ns,
            "informedTrackUpdateCount": 0,
        }
        track_timings[key] = timing

    timing["informedTrackUpdateCount"] += 1

    return {
        "acquiredTimeNs": timing["acquiredTimeNs"],
        "informedTrackUpdateCount": timing["informedTrackUpdateCount"],
        "lifetime": (last_update_time_ns - timing["acquiredTimeNs"]) / 1e9,
    }


def normalize_radar_position(p: Any) -> Optional[Dict[str, float]]:
    if not p:
        return None
    if isinstance(p, dict):
        lat = p.get("latitude") if p.get("latitude") is not None else p.get("latDeg") if p.get("latDeg") is not None else p.get("lat")
        lon = p.get("longitude") if p.get("longitude") is not None else p.get("lonDeg") if p.get("lonDeg") is not None else p.get("lon")
        alt = p.get("altitude") if p.get("altitude") is not None else p.get("altM") if p.get("altM") is not None else p.get("alt", 0)
    else:
        lat = getattr(p, "latitude", None) or getattr(p, "latDeg", None) or getattr(p, "lat", None)
        lon = getattr(p, "longitude", None) or getattr(p, "lonDeg", None) or getattr(p, "lon", None)
        alt = getattr(p, "altitude", None) or getattr(p, "altM", None) or getattr(p, "alt", 0)

    try:
        lat_f = float(lat)
        lon_f = float(lon)
        alt_f = float(alt)
    except (TypeError, ValueError):
        return None

    if not (math.isfinite(lat_f) and math.isfinite(lon_f) and math.isfinite(alt_f)):
        return None
    if abs(lat_f) > 90.0 or abs(lon_f) > 180.0:
        return None
    if abs(lat_f) < 0.0001 and abs(lon_f) < 0.0001:
        return None

    return {"latitude": lat_f, "longitude": lon_f, "altitude": alt_f}


def decode_status(msg: bytes) -> Dict[str, Any]:
    """Decode ICD STATUS message (type 7)."""
    if len(msg) < 68:
        return {"warning": "STATUS too short"}
    radar_sw_raw = struct.unpack_from(">I", msg, 16)[0]
    working_mode = struct.unpack_from(">I", msg, 24)[0]
    status_flags_raw = struct.unpack_from(">H", msg, 28)[0]
    bit_status = struct.unpack_from(">I", msg, 32)[0]
    return {
        "workingMode": working_mode,
        "radarSoftwareRaw": radar_sw_raw,
        "statusFlagsRaw": status_flags_raw,
        "bitStatus": bit_status,
    }


def decode_extended_status(msg: bytes, minimum_length: int) -> Optional[Dict[str, float]]:
    """Decode ICD EXTENDED_STATUS message (type 12 or 19)."""
    if len(msg) < minimum_length:
        return None
    lat_rad, lon_rad, alt_m = struct.unpack_from(">ddf", msg, 16)
    return normalize_radar_position({
        "latitude": lat_rad * RAD2DEG,
        "longitude": lon_rad * RAD2DEG,
        "altitude": alt_m,
    })


def decode_tracks_to_binary(msg: bytes, context: Dict[str, Any]) -> Dict[str, Any]:
    """Decode ICD TRACKS/TRACKS_EXTENDED message and encode to LDR1 binary frame."""
    if len(msg) < config.tracks_base_offset:
        return {"warning": "TRACKS too short"}

    msg_counter = struct.unpack_from(">I", msg, 0)[0]
    update_time_tag_usec = struct.unpack_from(">Q", msg, 16)[0]
    chunk_number = struct.unpack_from(">H", msg, 24)[0]
    total_tracks_in_burst = struct.unpack_from(">H", msg, 26)[0]
    num_tracks_in_message = struct.unpack_from(">H", msg, 30)[0]

    radar = normalize_radar_position(context.get("radarPosition"))
    ldrs_radar = normalize_radar_position(context.get("ldrsRadarPosition"))
    source_radar = ldrs_radar or radar
    radar_source = str(context.get("radarPosition", {}).get("source", "") if isinstance(context.get("radarPosition"), dict) else "")
    rebase_to_live_radar = (
        config.rebase_to_live_radar and bool(radar) and bool(ldrs_radar) and not radar_source.startswith("LDRS_")
    )
    tracks_time_tag_ms = decode_tracks_time_tag_ms(msg)
    ts_ms = tracks_time_tag_ms or context.get("timestampMs") or (time.time() * 1000.0)
    last_update_time_ns = timestamp_ms_to_ns(ts_ms)
    records = []

    if context.get("requireLiveRadarPosition") and not radar:
        return {
            "tracks": {
                "count": 0,
                "chunkNumber": chunk_number,
                "totalTracksInBurst": total_tracks_in_burst,
                "updateTimeTagUsec": update_time_tag_usec,
            }
        }

    for i in range(num_tracks_in_message):
        base = config.tracks_base_offset + i * config.track_slot_size
        min_needed = base + max(
            config.track_lon_offset + 8,
            config.track_alt_offset + 4,
            config.track_doppler_offset + 4,
            config.track_velocity_x_offset + 4,
            config.track_velocity_y_offset + 4,
            config.track_velocity_z_offset + 4,
            config.track_status_flags_offset + 2,
        )
        if min_needed > len(msg):
            break

        track_id = struct.unpack_from(">I", msg, base)[0]
        status_flags = struct.unpack_from(">H", msg, base + config.track_status_flags_offset)[0]
        is_valid_target = bool(status_flags & (1 << 13))

        if config.only_valid_targets and not is_valid_target:
            continue

        lat_rad = struct.unpack_from(">d", msg, base + config.track_lat_offset)[0]
        lon_rad = struct.unpack_from(">d", msg, base + config.track_lon_offset)[0]
        alt_m = struct.unpack_from(">f", msg, base + config.track_alt_offset)[0]
        velocity = struct.unpack_from(">f", msg, base + config.track_doppler_offset)[0]

        vel_x = struct.unpack_from(">f", msg, base + config.track_velocity_x_offset)[0]
        vel_y = struct.unpack_from(">f", msg, base + config.track_velocity_y_offset)[0]
        vel_z = struct.unpack_from(">f", msg, base + config.track_velocity_z_offset)[0]

        lat_deg = lat_rad * RAD2DEG
        lon_deg = lon_rad * RAD2DEG

        if not (math.isfinite(lat_deg) and math.isfinite(lon_deg) and abs(lat_deg) <= 90.0 and abs(lon_deg) <= 180.0):
            continue

        raw_target_ecef = lla_to_ecef({"latitude": lat_deg, "longitude": lon_deg, "altitude": alt_m})
        enu_m = get_enu_m(source_radar, raw_target_ecef)
        ecef = enu_to_ecef_position(enu_m, radar) if rebase_to_live_radar and radar else raw_target_ecef

        range_m = math.hypot(enu_m["e"], enu_m["n"], enu_m["u"])
        nwu_vel = {"x": vel_x, "y": vel_y, "z": vel_z}
        ecef_vel_from_track = nwu_vector_to_ecef(nwu_vel, radar if rebase_to_live_radar and radar else source_radar)
        ecef_vel_mps = (
            ecef_vel_from_track
            if has_finite_vector(ecef_vel_from_track)
            else get_ecef_velocity_mps(velocity, enu_m, radar)
        )
        timing = get_track_timing(track_id, last_update_time_ns)

        records.append({
            "generation": msg_counter,
            "trackId": track_id,
            "state": status_flags,
            "lifetime": timing["lifetime"],
            "informedTrackUpdateCount": timing["informedTrackUpdateCount"],
            "ecef": ecef,
            "ecefVelMps": ecef_vel_mps,
            "enuM": enu_m,
            "rangeM": range_m if math.isfinite(range_m) else math.hypot(enu_m["e"], enu_m["n"], enu_m["u"]),
            "lastUpdateTimeNs": last_update_time_ns,
            "lastAssocTimeNs": last_update_time_ns,
            "acquiredTimeNs": timing["acquiredTimeNs"],
            "trackUuid": f"ldrs-{track_id}",
            "positionId": f"ldrs-{msg_counter}-{track_id}",
        })

    if not records:
        return {
            "tracks": {
                "count": 0,
                "chunkNumber": chunk_number,
                "totalTracksInBurst": total_tracks_in_burst,
                "updateTimeTagUsec": update_time_tag_usec,
            }
        }

    frame_bytes = create_tracks_frame(
        timestamp_ms=context.get("timestampMs") or (time.time() * 1000.0),
        radar_position_valid=bool(radar),
        records=records,
        include_enu=config.include_enu,
    )

    return {
        "tracks": {
            "count": len(records),
            "chunkNumber": chunk_number,
            "totalTracksInBurst": total_tracks_in_burst,
            "updateTimeTagUsec": update_time_tag_usec,
            "buffer": frame_bytes,
        }
    }


def process_message(packet: bytes, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Process a raw ICD message buffer and return structured result."""
    if len(packet) < 16:
        return {"kind": "warning", "warning": f"Packet too short: {len(packet)} bytes"}

    msg_counter, msg_type, msg_version, msg_size = struct.unpack_from(">4I", packet, 0)

    if msg_size != len(packet):
        return {
            "kind": "warning",
            "warning": f"msgSize {msg_size} does not match packet length {len(packet)}",
        }

    ctx = context or {}

    if msg_type in (config.msg_tracks, config.msg_tracks_extended):
        result = decode_tracks_to_binary(packet, ctx)
        if "warning" in result:
            return {
                "kind": "warning",
                "msgCounter": msg_counter,
                "msgType": msg_type,
                "msgVersion": msg_version,
                "warning": result["warning"],
            }
        return {
            "kind": "tracks",
            "msgCounter": msg_counter,
            "msgType": msg_type,
            "msgVersion": msg_version,
            **result["tracks"],
        }

    if msg_type == config.msg_status:
        return {
            "kind": "status",
            "msgCounter": msg_counter,
            "msgType": msg_type,
            "msgVersion": msg_version,
            "status": decode_status(packet),
        }

    if msg_type in (config.msg_extended_status, config.msg_extended_status_mrs):
        is_mrs = msg_type == config.msg_extended_status_mrs
        source = "LDRS_EXTENDED_STATUS_MRS" if is_mrs else "LDRS_EXTENDED_STATUS"
        min_len = 96 if is_mrs else 60
        radar_pos = decode_extended_status(packet, min_len)
        if not radar_pos:
            return {
                "kind": "warning",
                "msgCounter": msg_counter,
                "msgType": msg_type,
                "msgVersion": msg_version,
                "warning": f"No valid {source} radar position",
            }
        return {
            "kind": "radarPosition",
            "msgCounter": msg_counter,
            "msgType": msg_type,
            "msgVersion": msg_version,
            "source": source,
            "radarPosition": radar_pos,
        }

    return {
        "kind": "ignored",
        "msgCounter": msg_counter,
        "msgType": msg_type,
        "msgVersion": msg_version,
    }
