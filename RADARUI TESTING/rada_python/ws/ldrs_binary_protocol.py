"""LDRS Binary WebSocket Protocol implementation (LDR1 Version 2)."""

import math
import struct
from typing import Any, Dict, List, Optional, Tuple

MAGIC = b"LDR1"
VERSION = 2

MESSAGE_TYPES = {
    "TRACKS": 1,
}

HEADER_LENGTH = 24
TRACK_RECORD_PREFIX_SIZE_WITHOUT_ENU = 82
TRACK_RECORD_PREFIX_SIZE_WITH_ENU = 94

HEADER_FLAGS = {
    "RADAR_POSITION_VALID": 1 << 0,
    "ENU_INCLUDED": 1 << 1,
}


def uint64_from_ns(value: Any) -> int:
    """Normalize value to unsigned 64-bit integer."""
    if value is None:
        return 0
    try:
        val = int(value)
        return max(0, val) & 0xFFFFFFFFFFFFFFFF
    except (ValueError, TypeError):
        return 0


def write_header(
    message_type: int,
    timestamp_ms: float,
    count: int,
    record_size: int,
    flags: int = 0,
) -> bytes:
    """Pack the 24-byte LDR1 frame header (all LE after magic)."""
    return struct.pack(
        "<4sBBHQHHI",
        MAGIC,
        VERSION,
        message_type,
        HEADER_LENGTH,
        int(math.trunc(timestamp_ms)),
        count,
        record_size,
        flags & 0xFFFFFFFF,
    )


def encode_var_string(value: Any) -> bytes:
    """Encode a string as 1 byte length prefix followed by UTF-8 bytes (max 255 bytes)."""
    string_val = "" if value is None else str(value)
    encoded = string_val.encode("utf-8")
    clipped = encoded[:255]
    return bytes([len(clipped)]) + clipped


def get_record_byte_length(record: Dict[str, Any], include_enu: bool) -> int:
    """Compute total byte length of a single track record."""
    prefix_size = (
        TRACK_RECORD_PREFIX_SIZE_WITH_ENU
        if include_enu
        else TRACK_RECORD_PREFIX_SIZE_WITHOUT_ENU
    )
    uuid_len = min(len(str(record.get("trackUuid") or "").encode("utf-8")), 255)
    pos_id_len = min(len(str(record.get("positionId") or "").encode("utf-8")), 255)
    return prefix_size + 1 + uuid_len + 1 + pos_id_len


def write_track_record(record: Dict[str, Any], include_enu: bool) -> bytes:
    """Pack a single track record into binary format."""
    generation = int(record.get("generation", 0)) & 0xFFFFFFFF
    track_id = int(record.get("trackId", 0)) & 0xFFFFFFFF
    state = int(record.get("state", 0)) & 0xFFFF
    lifetime = float(record.get("lifetime", 0.0))
    update_count = int(record.get("informedTrackUpdateCount", 0)) & 0xFFFFFFFF

    ecef = record.get("ecef") or {}
    ecef_x = float(ecef.get("x", 0.0))
    ecef_y = float(ecef.get("y", 0.0))
    ecef_z = float(ecef.get("z", 0.0))

    vel = record.get("ecefVelMps") or {}
    vel_x = float(vel.get("x", 0.0)) if math.isfinite(vel.get("x", float("nan"))) else float("nan")
    vel_y = float(vel.get("y", 0.0)) if math.isfinite(vel.get("y", float("nan"))) else float("nan")
    vel_z = float(vel.get("z", 0.0)) if math.isfinite(vel.get("z", float("nan"))) else float("nan")

    range_m = float(record.get("rangeM", 0.0))
    last_update = uint64_from_ns(record.get("lastUpdateTimeNs"))
    last_assoc = uint64_from_ns(record.get("lastAssocTimeNs"))
    acquired = uint64_from_ns(record.get("acquiredTimeNs"))

    if include_enu:
        enu = record.get("enuM") or {}
        enu_e = float(enu.get("e", 0.0))
        enu_n = float(enu.get("n", 0.0))
        enu_u = float(enu.get("u", 0.0))
        fixed_part = struct.pack(
            "<IIHfIdddfffffffQQQ",
            generation,
            track_id,
            state,
            lifetime,
            update_count,
            ecef_x,
            ecef_y,
            ecef_z,
            vel_x,
            vel_y,
            vel_z,
            enu_e,
            enu_n,
            enu_u,
            range_m,
            last_update,
            last_assoc,
            acquired,
        )
    else:
        fixed_part = struct.pack(
            "<IIHfIdddffffQQQ",
            generation,
            track_id,
            state,
            lifetime,
            update_count,
            ecef_x,
            ecef_y,
            ecef_z,
            vel_x,
            vel_y,
            vel_z,
            range_m,
            last_update,
            last_assoc,
            acquired,
        )

    var_uuid = encode_var_string(record.get("trackUuid"))
    var_pos_id = encode_var_string(record.get("positionId"))

    return fixed_part + var_uuid + var_pos_id


def create_tracks_frame(
    timestamp_ms: float,
    radar_position_valid: bool,
    records: List[Dict[str, Any]],
    include_enu: bool = False,
) -> bytes:
    """Create complete binary tracks frame with header and track records."""
    count = len(records)
    record_prefix_size = (
        TRACK_RECORD_PREFIX_SIZE_WITH_ENU
        if include_enu
        else TRACK_RECORD_PREFIX_SIZE_WITHOUT_ENU
    )

    flags = 0
    if radar_position_valid:
        flags |= HEADER_FLAGS["RADAR_POSITION_VALID"]
    if include_enu:
        flags |= HEADER_FLAGS["ENU_INCLUDED"]

    header = write_header(
        message_type=MESSAGE_TYPES["TRACKS"],
        timestamp_ms=timestamp_ms,
        count=count,
        record_size=record_prefix_size,
        flags=flags,
    )

    chunks = [header]
    for record in records:
        chunks.append(write_track_record(record, include_enu))

    return b"".join(chunks)


def decode_tracks_frame(data: bytes) -> Dict[str, Any]:
    """Decode a binary LDR1 tracks frame matching docs/ldrs-binary-protocol.md."""
    if len(data) < HEADER_LENGTH:
        raise ValueError(f"Data too short for LDRS header: {len(data)} bytes")

    magic, ver, msg_type, header_len, timestamp_ms, count, record_size, flags = (
        struct.unpack_from("<4sBBHQHHI", data, 0)
    )

    if magic != MAGIC:
        raise ValueError(f"Invalid magic: {magic}")
    if ver != VERSION:
        raise ValueError(f"Unsupported version: {ver}")

    radar_pos_valid = bool(flags & HEADER_FLAGS["RADAR_POSITION_VALID"])
    include_enu = bool(flags & HEADER_FLAGS["ENU_INCLUDED"])

    offset = header_len
    tracks = []

    for _ in range(count):
        if include_enu:
            if offset + TRACK_RECORD_PREFIX_SIZE_WITH_ENU > len(data):
                break
            (
                generation,
                track_id,
                state,
                lifetime,
                update_count,
                ecef_x,
                ecef_y,
                ecef_z,
                vel_x,
                vel_y,
                vel_z,
                enu_e,
                enu_n,
                enu_u,
                range_m,
                last_update,
                last_assoc,
                acquired,
            ) = struct.unpack_from("<IIHfIdddfffffffQQQ", data, offset)
            offset += TRACK_RECORD_PREFIX_SIZE_WITH_ENU
            enu_dict = {"e": enu_e, "n": enu_n, "u": enu_u}
        else:
            if offset + TRACK_RECORD_PREFIX_SIZE_WITHOUT_ENU > len(data):
                break
            (
                generation,
                track_id,
                state,
                lifetime,
                update_count,
                ecef_x,
                ecef_y,
                ecef_z,
                vel_x,
                vel_y,
                vel_z,
                range_m,
                last_update,
                last_assoc,
                acquired,
            ) = struct.unpack_from("<IIHfIdddffffQQQ", data, offset)
            offset += TRACK_RECORD_PREFIX_SIZE_WITHOUT_ENU
            enu_dict = None

        # Read trackUuid
        if offset >= len(data):
            break
        uuid_len = data[offset]
        offset += 1
        uuid_bytes = data[offset : offset + uuid_len]
        offset += uuid_len
        track_uuid = uuid_bytes.decode("utf-8", errors="replace")

        # Read positionId
        if offset >= len(data):
            break
        pos_id_len = data[offset]
        offset += 1
        pos_id_bytes = data[offset : offset + pos_id_len]
        offset += pos_id_len
        position_id = pos_id_bytes.decode("utf-8", errors="replace")

        record = {
            "generation": generation,
            "trackId": track_id,
            "state": state,
            "lifetime": lifetime,
            "informedTrackUpdateCount": update_count,
            "ecef": {"x": ecef_x, "y": ecef_y, "z": ecef_z},
            "ecefVelMps": {"x": vel_x, "y": vel_y, "z": vel_z},
            "enuM": enu_dict,
            "rangeM": range_m,
            "lastUpdateTimeNs": last_update,
            "lastAssocTimeNs": last_assoc,
            "acquiredTimeNs": acquired,
            "trackUuid": track_uuid,
            "positionId": position_id,
        }

        tracks.append(record)

    return {
        "version": ver,
        "messageType": msg_type,
        "timestampMs": timestamp_ms,
        "radarPositionValid": radar_pos_valid,
        "includeEnu": include_enu,
        "count": len(tracks),
        "tracks": tracks,
    }
