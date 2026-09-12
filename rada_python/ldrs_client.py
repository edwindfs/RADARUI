"""Standalone legacy LDRS TCP client with VectorNav NMEA hook (ldrsClient.js)."""

from datetime import datetime, timezone
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from typing import Any, Dict, Optional

from .formatters.build_ldrs_detection import build_ldrs_detection
from .utils.geo import bearing_degrees, distance_from_radar, haversine_meters, to_rad
from .utils.direction import get_direction_from_bearing

try:
    import serial
except ImportError:
    serial = None

RADAR_HOST = os.environ.get("RADAR_HOST", "192.168.170.1")
RADAR_PORT = int(os.environ.get("RADAR_PORT", 7002))

VN_PORT = os.environ.get("VN_PORT", "/dev/ttyUSB0")
VN_BAUD = int(os.environ.get("VN_BAUD", 115200))

MSG_TYPE = {
    "TRACKS": int(os.environ.get("MSG_TRACKS", 10)),
    "TRACKS_EXTENDED": int(os.environ.get("MSG_TRACKS_EXTENDED", 20)),
    "STATUS": int(os.environ.get("MSG_STATUS", 7)),
    "EXTENDED_STATUS_MRS": int(os.environ.get("MSG_EXT_STATUS", 19)),
}

RAD2DEG = 180.0 / math.pi
MIN_MSG_SIZE = 16
MAX_MSG_SIZE = int(os.environ.get("MAX_MSG_SIZE", 2_000_000))

TRACK_SLOT_SIZE = int(os.environ.get("TRACK_SLOT_SIZE", 232))
TRACKS_BASE_OFFSET = int(os.environ.get("TRACKS_BASE_OFFSET", 48))
TRACK_LAT_OFFSET = int(os.environ.get("TRACK_LAT_OFFSET", 48))
TRACK_LON_OFFSET = int(os.environ.get("TRACK_LON_OFFSET", 56))
TRACK_ALT_OFFSET = int(os.environ.get("TRACK_ALT_OFFSET", 64))

started_at_ms = time.time() * 1000.0


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def format_active_time(ms_since_start: float) -> str:
    sec = max(0, int(ms_since_start // 1000))
    if sec < 60:
        return f"{sec}s"
    minutes = sec // 60
    rem = sec % 60
    return f"{minutes}min" if rem == 0 else f"{minutes}min {rem}s"


def threat_from_distance_m(m: float) -> str:
    return "high" if m < 2000 else "low"


# -------------------- VECTORNAV NMEA --------------------
last_utc_ms: Optional[float] = None
last_monotonic_ns: Optional[int] = None

last_fix: Dict[str, Any] = {
    "lat": None,
    "lon": None,
    "alt": None,
    "quality": None,
    "sats": None,
    "updatedAt": None,
}
vn_lock = threading.Lock()


def set_utc_now(utc_ms: float) -> None:
    global last_utc_ms, last_monotonic_ns
    with vn_lock:
        last_utc_ms = utc_ms
        last_monotonic_ns = time.monotonic_ns()


def now_utc_ms() -> float:
    with vn_lock:
        if last_utc_ms is None or last_monotonic_ns is None:
            return time.time() * 1000.0
        elapsed_ns = time.monotonic_ns() - last_monotonic_ns
        return last_utc_ms + (elapsed_ns / 1_000_000.0)


def nmea_to_decimal(raw: Optional[str], hemi: Optional[str]) -> Optional[float]:
    if not raw or not hemi:
        return None
    clean = raw.split("*")[0].strip()
    if len(clean) < 4:
        return None

    is_lat = hemi in ("N", "S")
    deg_digits = 2 if is_lat else 3

    try:
        deg = float(clean[:deg_digits])
        minutes = float(clean[deg_digits:])
        if not math.isfinite(deg) or not math.isfinite(minutes):
            return None
        dec = deg + (minutes / 60.0)
        if hemi in ("S", "W"):
            dec *= -1.0
        return dec
    except ValueError:
        return None


def parse_gprmc(line: str) -> bool:
    if not (line.startswith("$GPRMC") or line.startswith("$GNRMC")):
        return False
    parts = line.split(",")
    if len(parts) < 10:
        return False
    time_str = parts[1]
    status = parts[2]
    date_str = parts[9]
    if status != "A" or len(time_str) < 6 or len(date_str) < 6:
        return False

    try:
        hh = int(time_str[:2])
        mm = int(time_str[2:4])
        ss = int(time_str[4:6])
        dd = int(date_str[:2])
        mo = int(date_str[2:4])
        yy = int(date_str[4:6]) + 2000
        dt = datetime(yy, mo, dd, hh, mm, ss, tzinfo=timezone.utc)
        set_utc_now(dt.timestamp() * 1000.0)
        return True
    except Exception:
        return False


def parse_gpzda(line: str) -> bool:
    if not (line.startswith("$GPZDA") or line.startswith("$GNZDA")):
        return False
    parts = line.split(",")
    if len(parts) < 5:
        return False
    time_str = parts[1]
    if len(time_str) < 6:
        return False
    try:
        hh = int(time_str[:2])
        mm = int(time_str[2:4])
        ss = int(time_str[4:6])
        dd = int(parts[2])
        mo = int(parts[3])
        yyyy = int(parts[4])
        dt = datetime(yyyy, mo, dd, hh, mm, ss, tzinfo=timezone.utc)
        set_utc_now(dt.timestamp() * 1000.0)
        return True
    except Exception:
        return False


def parse_gpgga(line: str) -> bool:
    global last_fix
    if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
        return False
    parts = line.split(",")
    if len(parts) < 10:
        return False

    lat = nmea_to_decimal(parts[2], parts[3])
    lon = nmea_to_decimal(parts[4], parts[5])
    try:
        quality = int(parts[6]) if parts[6] else 0
    except ValueError:
        quality = 0

    try:
        sats = int(parts[7]) if parts[7] else None
    except ValueError:
        sats = None

    try:
        alt = float(parts[9]) if parts[9] else None
    except ValueError:
        alt = None

    if lat is None or lon is None or quality <= 0:
        return False

    with vn_lock:
        last_fix = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "quality": quality,
            "sats": sats,
            "updatedAt": iso_now(),
        }
    return True


def get_radar_position_from_vn() -> Optional[Dict[str, Any]]:
    with vn_lock:
        if last_fix["lat"] is None or last_fix["lon"] is None:
            return None
        return {
            "latitude": last_fix["lat"],
            "longitude": last_fix["lon"],
            "altitude": last_fix["alt"] if last_fix["alt"] is not None else 0.0,
            "timestamp": iso_now(),
            "source": "VectorNav:NMEA",
            "quality": last_fix["quality"],
            "sats": last_fix["sats"],
        }


def start_vector_nav() -> None:
    if serial is None:
        print("[VN] SerialPort unavailable (pyserial not installed)")
        return

    def _reader():
        try:
            ser = serial.Serial(VN_PORT, VN_BAUD, timeout=1.0)
            print(f"[{iso_now()}] [VN] Serial open {VN_PORT} @ {VN_BAUD}")
            while True:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if parse_gprmc(line):
                    continue
                if parse_gpzda(line):
                    continue
                if parse_gpgga(line):
                    continue
        except Exception as e:
            print(f"[{iso_now()}] [VN] Serial error: {e}")

    threading.Thread(target=_reader, daemon=True).start()


# -------------------- LDRS TCP CLIENT --------------------
last_radar_geo_from_ldrs: Optional[Dict[str, Any]] = None


def decode_status(msg: bytes) -> None:
    if len(msg) < 68:
        return
    working_mode = struct.unpack_from(">I", msg, 24)[0]
    print(f"[LDRS][STATUS] workingMode={working_mode}")
    if working_mode == 6:
        print("[LDRS][STATUS] Looks like STANDBY (common). In standby, radar usually will NOT output TRACKS.")


def decode_extended_status_mrs(msg: bytes) -> None:
    global last_radar_geo_from_ldrs
    if len(msg) < 96:
        return
    radar_lat_rad, radar_lon_rad, radar_alt_m = struct.unpack_from(">ddf", msg, 16)
    lat_deg = radar_lat_rad * RAD2DEG
    lon_deg = radar_lon_rad * RAD2DEG

    last_radar_geo_from_ldrs = {
        "latitude": lat_deg,
        "longitude": lon_deg,
        "altitude": radar_alt_m,
        "timestamp": iso_now(),
        "source": "LDRS:EXTENDED_STATUS_MRS",
    }
    print(f"[LDRS][RADAR_GEO] lat={lat_deg:.6f} lon={lon_deg:.6f} alt={radar_alt_m:.2f}")
    print("[LDRS][RADAR_GEO] cached as fallback radarPosition.")


def decode_tracks(msg: bytes, msg_counter: int) -> None:
    if len(msg) < TRACKS_BASE_OFFSET:
        return

    num_tracks = struct.unpack_from(">H", msg, 30)[0]
    year, month, day, hour, minute, second, millis = struct.unpack_from(">7H", msg, 32)
    timestamp_str = f"{year:04d}-{month:02d}-{day:02d}T{hour:02d}:{minute:02d}:{second:02d}.{millis:03d}Z"

    print(f"[LDRS][TRACKS] numTracks={num_tracks} time={timestamp_str}")

    radar_position = get_radar_position_from_vn() or last_radar_geo_from_ldrs or None
    if not radar_position:
        print("[LDRS] No radarPosition available (VN no fix and no LDRS GEO cached).")

    for i in range(num_tracks):
        base = TRACKS_BASE_OFFSET + i * TRACK_SLOT_SIZE
        needed = base + TRACK_ALT_OFFSET + 4
        if needed > len(msg):
            break

        track_id = struct.unpack_from(">I", msg, base)[0]
        lat_rad = struct.unpack_from(">d", msg, base + TRACK_LAT_OFFSET)[0]
        lon_rad = struct.unpack_from(">d", msg, base + TRACK_LON_OFFSET)[0]
        alt_m = struct.unpack_from(">f", msg, base + TRACK_ALT_OFFSET)[0]

        track = {
            "trackId": track_id,
            "latitude": lat_rad * RAD2DEG,
            "longitude": lon_rad * RAD2DEG,
            "altitude": alt_m,
            "timestamp": timestamp_str,
            "messageCounter": msg_counter,
        }

        print("[LDRS] Track:", json.dumps(track, indent=2))

        if radar_position:
            det = build_ldrs_detection(track, radar_position, started_at_ms)
            print("[LDRS] Detection:", json.dumps(det, indent=2))


def handle_radar_message(msg: bytes) -> None:
    msg_counter, msg_type, _, msg_size = struct.unpack_from(">4I", msg, 0)
    print("--------------------------------------------------")
    print(f"[{iso_now()}] [LDRS] Message counter={msg_counter} type={msg_type} size={msg_size}")

    if msg_type in (MSG_TYPE["TRACKS"], MSG_TYPE["TRACKS_EXTENDED"]):
        decode_tracks(msg, msg_counter)
    elif msg_type == MSG_TYPE["STATUS"]:
        decode_status(msg)
    elif msg_type == MSG_TYPE["EXTENDED_STATUS_MRS"]:
        decode_extended_status_mrs(msg)
    else:
        preview = msg[: min(64, len(msg))].hex()
        print(f"[LDRS] Unhandled msgType={msg_type}. First 64 bytes hex:\n{preview}")


def start_ldrs_client() -> None:
    print(f"[{iso_now()}] [LDRS] Connecting to {RADAR_HOST}:{RADAR_PORT}...")

    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((RADAR_HOST, RADAR_PORT))
            print(f"[{iso_now()}] [LDRS] Connected to {RADAR_HOST}:{RADAR_PORT}")
            buffer = b""

            while True:
                data = s.recv(4096)
                if not data:
                    print(f"[{iso_now()}] [LDRS] Connection closed by remote")
                    break

                buffer += data
                while len(buffer) >= MIN_MSG_SIZE:
                    msg_size = struct.unpack_from(">I", buffer, 12)[0]
                    if msg_size < MIN_MSG_SIZE or msg_size > MAX_MSG_SIZE:
                        print(f"[{iso_now()}] [LDRS] Invalid msgSize={msg_size}. Dropping buffer.")
                        buffer = b""
                        break

                    if len(buffer) < msg_size:
                        break

                    msg = buffer[:msg_size]
                    buffer = buffer[msg_size:]
                    handle_radar_message(msg)

        except Exception as e:
            print(f"[{iso_now()}] [LDRS] Socket error: {e}")

        print(f"[{iso_now()}] [LDRS] Reconnecting in 3s...")
        time.sleep(3)


def main():
    start_vector_nav()
    start_ldrs_client()


if __name__ == "__main__":
    main()
