"""LDRS TCP client and WebSocket fanout service (v1.ldrClient.js).

Main thread: TCP framing, VectorNav state, control commands, WebSocket fanout.
Worker thread: LDRS decode, filtering, compact binary encoding.
"""

import asyncio
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import socket
import struct
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Set

from .services.vector_nav_service import VectorNavService
from .services.vector_nav_ws_service import VectorNavWsService
from .workers.ldrs_worker_client import LdrsWorkerClient

try:
    import websockets
except ImportError:
    websockets = None


def load_env_file(filepath: str = ".env") -> None:
    """Simple .env loader if python-dotenv is not installed."""
    if not os.path.exists(filepath):
        # Look in parent directories as well
        parent = os.path.join(os.path.dirname(__file__), "..", filepath)
        if os.path.exists(parent):
            filepath = parent
        else:
            return

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k = k.strip()
                v = v.strip().strip("'\"")
                if k not in os.environ:
                    os.environ[k] = v
    except Exception:
        pass


load_env_file()

RADAR_HOST = os.environ.get("RADAR_HOST", "192.168.170.1")
RADAR_PORT = int(os.environ.get("RADAR_PORT", 7002))
WS_PORT = int(os.environ.get("WS_PORT", 8070))
AUTO_SEARCH = bool(os.environ.get("AUTO_SEARCH", "").lower() in ("1", "true", "yes"))
DEBUG_LDRS = bool(os.environ.get("DEBUG_LDRS", "").lower() in ("1", "true", "yes"))
LDRS_LOG_ALL = bool(os.environ.get("LDRS_LOG_ALL", "").lower() in ("1", "true", "yes"))
LDRS_LOG_HEX_BYTES = int(os.environ.get("LDRS_LOG_HEX_BYTES", 64))
LDRS_DUMP_DIR = os.environ.get("LDRS_DUMP_DIR", "")

VN_PORT = os.environ.get("VN_PORT", "/dev/ttyUSB_VN")
VN_BAUD = int(os.environ.get("VN_BAUD", 115200))
VN_SOURCE = os.environ.get("VN_SOURCE", "serial").lower()
VN_WS_URL = os.environ.get("VN_WS_URL", "ws://localhost:8080/ws/vectornav")
REQUIRE_VECTORNAV_POSITION = bool(os.environ.get("REQUIRE_VECTORNAV_POSITION", "").lower() in ("1", "true", "yes"))
LOC_ATT_UPDATE_FROM_VN = bool(os.environ.get("LOC_ATT_UPDATE_FROM_VN", "").lower() in ("1", "true", "yes"))
LOC_ATT_RESTART_AFTER_UPDATE = bool(os.environ.get("LOC_ATT_RESTART_AFTER_UPDATE", "").lower() in ("1", "true", "yes"))
LOC_ATT_RESTART_DELAY_MS = int(os.environ.get("LOC_ATT_RESTART_DELAY_MS", 120_000))
MANUAL_RADAR_POSITION = bool(os.environ.get("MANUAL_RADAR_POSITION", "").lower() in ("1", "true", "yes"))

MIN_MSG_SIZE = 16
MAX_MSG_SIZE = int(os.environ.get("MAX_MSG_SIZE", 2_000_000))
MAX_PENDING_WORKER_JOBS = int(os.environ.get("MAX_PENDING_LDRS_JOBS", 128))
WS_MAX_BUFFERED_BYTES = int(os.environ.get("WS_MAX_BUFFERED_BYTES", 1_000_000))

MSG_TYPE = {
    "STATUS": int(os.environ.get("MSG_STATUS", 7)),
    "MODE_REQUEST": int(os.environ.get("MSG_MODE_REQUEST", 3)),
    "LOC_ATT_DATA": int(os.environ.get("MSG_LOC_ATT_DATA", 16)),
    "RESTART": int(os.environ.get("MSG_RESTART", 18)),
}

WORKING_MODE = {
    "SEARCH": 3,
    "STANDBY": 6,
}

es_message_counter = int(os.environ.get("ES_MSG_COUNTER_START", 150000))
active_client: Optional[socket.socket] = None
search_requested = False
last_radar_pos_from_ldrs: Optional[Dict[str, Any]] = None
last_radar_pos_from_vn: Optional[Dict[str, Any]] = None
last_used_radar_position: Optional[Dict[str, Any]] = None
loc_att_update_sent = False
loc_att_port_warning_logged = False
loc_att_restart_timer: Optional[threading.Timer] = None
state_lock = threading.Lock()

stats = {
    "total": 0,
    "byType": {},
    "submittedToWorker": 0,
    "workerDropped": 0,
    "binaryFrames": 0,
    "binaryBytes": 0,
    "wsClients": 0,
    "wsDroppedSlowClient": 0,
    "lastTracksAt": 0.0,
    "lastWorkerContext": None,
    "radarPositionSource": "none",
}

ws_clients: Set[Any] = set()
ws_loop: Optional[asyncio.AbstractEventLoop] = None


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def to_rad(deg: float) -> float:
    return (deg * math.pi) / 180.0


def sanitize_filename_part(value: Any) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]", "_", str(value))


def is_valid_pos(p: Optional[Dict[str, Any]]) -> bool:
    if not p:
        return False
    lat = p.get("latitude") if p.get("latitude") is not None else p.get("latDeg") if p.get("latDeg") is not None else p.get("lat")
    lon = p.get("longitude") if p.get("longitude") is not None else p.get("lonDeg") if p.get("lonDeg") is not None else p.get("lon")
    alt = p.get("altitude") if p.get("altitude") is not None else p.get("altM") if p.get("altM") is not None else p.get("alt", 0)

    try:
        lat_f = float(lat)
        lon_f = float(lon)
        alt_f = float(alt)
    except (TypeError, ValueError):
        return False

    if not (math.isfinite(lat_f) and math.isfinite(lon_f) and math.isfinite(alt_f)):
        return False
    if abs(lat_f) > 90.0 or abs(lon_f) > 180.0:
        return False
    if abs(lat_f) < 0.0001 and abs(lon_f) < 0.0001:
        return False
    return True


def normalize_pos(p: Optional[Dict[str, Any]], source: Optional[str] = None) -> Optional[Dict[str, Any]]:
    if not p:
        return None
    lat = p.get("latitude") if p.get("latitude") is not None else p.get("latDeg") if p.get("latDeg") is not None else p.get("lat")
    lon = p.get("longitude") if p.get("longitude") is not None else p.get("lonDeg") if p.get("lonDeg") is not None else p.get("lon")
    alt = p.get("altitude") if p.get("altitude") is not None else p.get("altM") if p.get("altM") is not None else p.get("alt", 0)

    heading = (
        p.get("headingDeg")
        if p.get("headingDeg") is not None
        else p.get("heading")
        if p.get("heading") is not None
        else p.get("yaw")
        if p.get("yaw") is not None
        else p.get("YAW")
    )
    pitch = p.get("pitchDeg") if p.get("pitchDeg") is not None else p.get("pitch") if p.get("pitch") is not None else p.get("PITCH")
    roll = p.get("rollDeg") if p.get("rollDeg") is not None else p.get("roll") if p.get("roll") is not None else p.get("ROLL")

    try:
        lat_f = float(lat)
        lon_f = float(lon)
        alt_f = float(alt)
    except (TypeError, ValueError):
        return None

    heading_f = float(heading) if heading is not None and math.isfinite(float(heading)) else None
    pitch_f = float(pitch) if pitch is not None and math.isfinite(float(pitch)) else None
    roll_f = float(roll) if roll is not None and math.isfinite(float(roll)) else None

    out = {
        "latitude": lat_f,
        "longitude": lon_f,
        "altitude": alt_f,
        "latDeg": lat_f,
        "lonDeg": lon_f,
        "altM": alt_f,
        "headingDeg": heading_f,
        "pitchDeg": pitch_f,
        "rollDeg": roll_f,
        "source": source or p.get("source", "unknown"),
        "at": time.time() * 1000.0,
    }
    return out if is_valid_pos(out) else None


manual_radar_position: Optional[Dict[str, Any]] = None
if MANUAL_RADAR_POSITION:
    try:
        manual_radar_position = normalize_pos(
            {
                "latitude": float(os.environ.get("MANUAL_RADAR_LAT", 0)),
                "longitude": float(os.environ.get("MANUAL_RADAR_LON", 0)),
                "altitude": float(os.environ.get("MANUAL_RADAR_ALT", 0)),
            },
            "MANUAL",
        )
    except Exception:
        manual_radar_position = None

    if manual_radar_position:
        print(f"[{iso_now()}] [RADAR] using manual position", manual_radar_position)
    else:
        print(f"[{iso_now()}] [RADAR] MANUAL_RADAR_POSITION=true but coordinates invalid; falling back to VN/LDRS")


def log_radar_message(msg: bytes) -> None:
    msg_counter, msg_type, msg_version, msg_size = struct.unpack_from(">4I", msg, 0)
    preview_bytes = max(0, min(LDRS_LOG_HEX_BYTES, len(msg)))
    hex_preview = msg[:preview_bytes].hex()

    if LDRS_LOG_ALL:
        print(
            f"[{iso_now()}] [LDRS][RAW] counter={msg_counter} type={msg_type} "
            f"version={msg_version} size={msg_size} bytes={len(msg)} "
            f"hex{preview_bytes}={hex_preview}"
        )

    if LDRS_DUMP_DIR:
        try:
            os.makedirs(LDRS_DUMP_DIR, exist_ok=True)
            file_name = f"{int(time.time() * 1000)}_{sanitize_filename_part(msg_counter)}_type{sanitize_filename_part(msg_type)}_{len(msg)}b.bin"
            file_path = os.path.join(LDRS_DUMP_DIR, file_name)
            with open(file_path, "wb") as f:
                f.write(msg)

            metadata = {
                "at": iso_now(),
                "msgCounter": msg_counter,
                "msgType": msg_type,
                "msgVersion": msg_version,
                "msgSize": msg_size,
                "actualBytes": len(msg),
                "hexPreview": hex_preview,
                "fileName": file_name,
            }
            jsonl_path = os.path.join(LDRS_DUMP_DIR, "ldrs-messages.jsonl")
            with open(jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(metadata) + "\n")
        except Exception as e:
            print(f"[{iso_now()}] [LDRS][DUMP] error: {e}")


def next_es_counter() -> int:
    global es_message_counter
    curr = es_message_counter
    es_message_counter += 1
    if es_message_counter > 0xFFFFFFFF:
        es_message_counter = 150000
    return curr


def build_message_header(msg_type: int, msg_size: int) -> bytearray:
    buf = bytearray(msg_size)
    struct.pack_into(">4I", buf, 0, next_es_counter(), msg_type, 0, msg_size)
    return buf


def send_mode_request(client: Optional[socket.socket], working_mode: int) -> bool:
    if not client:
        return False
    msg_size = 160
    buf = build_message_header(MSG_TYPE["MODE_REQUEST"], msg_size)
    struct.pack_into(">I", buf, 16, working_mode)
    try:
        client.sendall(buf)
        print(f"[{iso_now()}] [LDRS] Sent MODE_REQUEST workingMode={working_mode}")
        return True
    except Exception as e:
        print(f"[{iso_now()}] [LDRS] Failed to send MODE_REQUEST: {e}")
        return False


def build_loc_att_data_message(radar_pos: Dict[str, Any]) -> bytearray:
    msg_size = 56
    buf = build_message_header(MSG_TYPE["LOC_ATT_DATA"], msg_size)
    has_att = (
        radar_pos.get("headingDeg") is not None
        and radar_pos.get("pitchDeg") is not None
        and radar_pos.get("rollDeg") is not None
    )

    flags = 3 if has_att else 1
    struct.pack_into(">I", buf, 16, flags)
    struct.pack_into(">dd", buf, 24, to_rad(radar_pos["latitude"]), to_rad(radar_pos["longitude"]))
    struct.pack_into(">f", buf, 40, float(radar_pos["altitude"]))

    if has_att:
        struct.pack_into(
            ">fff",
            buf,
            44,
            to_rad(radar_pos["headingDeg"]),
            to_rad(radar_pos["pitchDeg"]),
            to_rad(radar_pos["rollDeg"]),
        )
    return buf


def build_restart_message() -> bytearray:
    msg_size = 20
    buf = build_message_header(MSG_TYPE["RESTART"], msg_size)
    struct.pack_into(">I", buf, 16, 1)
    return buf


def send_restart(client: Optional[socket.socket]) -> bool:
    if not client:
        return False
    try:
        client.sendall(build_restart_message())
        print(f"[{iso_now()}] [LDRS] Sent RESTART action=1")
        return True
    except Exception as e:
        print(f"[{iso_now()}] [LDRS] Failed to send RESTART: {e}")
        return False


def schedule_restart_after_loc_att_data() -> None:
    global loc_att_restart_timer
    if not LOC_ATT_RESTART_AFTER_UPDATE or loc_att_restart_timer is not None:
        return

    delay_sec = max(0.0, LOC_ATT_RESTART_DELAY_MS / 1000.0)

    def _do_restart():
        global loc_att_restart_timer
        loc_att_restart_timer = None
        if not send_restart(active_client):
            print(f"[{iso_now()}] [LDRS] RESTART not sent; radar TCP client is not connected")

    loc_att_restart_timer = threading.Timer(delay_sec, _do_restart)
    loc_att_restart_timer.daemon = True
    loc_att_restart_timer.start()

    print(f"[{iso_now()}] [LDRS] RESTART scheduled in {LOC_ATT_RESTART_DELAY_MS}ms after LOC_ATT_DATA")


def maybe_send_loc_att_data_from_vn(client: Optional[socket.socket], radar_pos: Optional[Dict[str, Any]]) -> None:
    global loc_att_update_sent, loc_att_port_warning_logged
    if not LOC_ATT_UPDATE_FROM_VN or loc_att_update_sent or not is_valid_pos(radar_pos):
        return

    if RADAR_PORT != 7002:
        if not loc_att_port_warning_logged:
            loc_att_port_warning_logged = True
            print(f"[{iso_now()}] [LDRS] LOC_ATT_DATA requires control port 7002; current RADAR_PORT={RADAR_PORT}")
        return

    if not client:
        return

    try:
        msg = build_loc_att_data_message(radar_pos)
        client.sendall(msg)
        loc_att_update_sent = True
        h_str = f"{radar_pos['headingDeg']:.2f}" if radar_pos.get("headingDeg") is not None else "none"
        p_str = f"{radar_pos['pitchDeg']:.2f}" if radar_pos.get("pitchDeg") is not None else "none"
        r_str = f"{radar_pos['rollDeg']:.2f}" if radar_pos.get("rollDeg") is not None else "none"
        print(
            f"[{iso_now()}] [LDRS] Sent LOC_ATT_DATA type={MSG_TYPE['LOC_ATT_DATA']} "
            f"lat={radar_pos['latitude']:.7f} lon={radar_pos['longitude']:.7f} "
            f"alt={radar_pos['altitude']:.2f} heading={h_str} pitch={p_str} roll={r_str} "
            f"source={radar_pos.get('source')}"
        )
        schedule_restart_after_loc_att_data()
    except Exception as e:
        print(f"[{iso_now()}] [LDRS] Failed to send LOC_ATT_DATA: {e}")


def get_best_radar_position() -> Optional[Dict[str, Any]]:
    if is_valid_pos(last_radar_pos_from_vn):
        return last_radar_pos_from_vn
    if REQUIRE_VECTORNAV_POSITION:
        return None
    if is_valid_pos(manual_radar_position):
        return manual_radar_position
    if is_valid_pos(last_radar_pos_from_ldrs):
        return last_radar_pos_from_ldrs
    return None


def create_vector_nav_service():
    if VN_SOURCE in ("ws", "websocket"):
        return VectorNavWsService(url=VN_WS_URL)

    if VN_SOURCE in ("none", "off", "disabled"):
        class _DisabledVN:
            def start(self):
                print("[VN] disabled; using manual/LDRS type19 radar position fallback only")
            def get_radar_position(self):
                return None
            def stop(self):
                pass
        return _DisabledVN()

    return VectorNavService(port_path=VN_PORT, baud_rate=VN_BAUD)


vn = create_vector_nav_service()
vn.start()

ldrs_worker = LdrsWorkerClient(max_pending=MAX_PENDING_WORKER_JOBS)


def broadcast_binary(payload: bytes) -> None:
    global stats
    with state_lock:
        stats["binaryFrames"] += 1
        stats["binaryBytes"] += len(payload)

    if not ws_clients or ws_loop is None or not ws_loop.is_running():
        return

    # Schedule sending to clients on the asyncio loop
    for client in list(ws_clients):
        try:
            asyncio.run_coroutine_threadsafe(client.send(payload), ws_loop)
        except Exception:
            pass


def on_worker_message(message: Dict[str, Any]) -> None:
    global last_radar_pos_from_ldrs, search_requested

    with state_lock:
        stats["lastWorkerContext"] = message.get("worker")

    kind = message.get("kind")

    if kind == "tracks":
        with state_lock:
            stats["lastTracksAt"] = time.time()
        buf = message.get("buffer")
        count = message.get("count", 0)
        if buf and count > 0:
            broadcast_binary(buf)
        if DEBUG_LDRS:
            print(
                f"[{iso_now()}] [LDRS][TRACKS] count={count} chunk={message.get('chunkNumber')} "
                f"workerThread={message.get('worker', {}).get('threadId')}"
            )
        return

    if kind == "radarPosition":
        pos = normalize_pos(message.get("radarPosition"), message.get("source") or "LDRS_EXTENDED_STATUS")
        last_radar_pos_from_ldrs = pos
        if DEBUG_LDRS and pos:
            print(f"[{iso_now()}] [LDRS] radar position", pos)
        return

    if kind == "status":
        status = message.get("status") or {}
        working_mode = status.get("workingMode")
        status_flags = status.get("statusFlagsRaw", 0)
        is_not_ready = bool(status_flags & (1 << 2))
        is_maintenance = bool(status_flags & (1 << 3))

        if (
            AUTO_SEARCH
            and RADAR_PORT == 7002
            and not search_requested
            and working_mode == WORKING_MODE["STANDBY"]
            and not is_not_ready
            and not is_maintenance
        ):
            search_requested = send_mode_request(active_client, WORKING_MODE["SEARCH"])

        if DEBUG_LDRS:
            flag_hex = f"0x{status_flags:04x}" if status_flags is not None else "????"
            print(
                f"[{iso_now()}] [LDRS][STATUS] workingMode={working_mode} "
                f"statusFlags={flag_hex} notReady={is_not_ready} maintenance={is_maintenance}"
            )
        return

    if kind == "warning":
        print(f"[{iso_now()}] [LDRS][WORKER] {message.get('warning')}")
        return

    if kind == "error":
        print(f"[{iso_now()}] [LDRS][WORKER] {message.get('error')}")
        return

    if DEBUG_LDRS and kind == "ignored":
        print(f"[{iso_now()}] [LDRS][IGNORED] type={message.get('msgType')} counter={message.get('msgCounter')}")


ldrs_worker.on("message", on_worker_message)


def stats_timer_loop() -> None:
    while True:
        time.sleep(5.0)
        with state_lock:
            last_tr = stats["lastTracksAt"]
            since_tracks = f"{int(time.time() - last_tr)}s ago" if last_tr else "never"
            worker_stats = ldrs_worker.get_stats()
            stats["workerDropped"] = worker_stats["dropped"]
            stats["wsClients"] = len(ws_clients)
            last_w = stats["lastWorkerContext"]
            w_str = f"threadId={last_w['threadId']}" if last_w else "none"

            print(
                f"[STATS] total={stats['total']} types={json.dumps(stats['byType'])} lastTracks={since_tracks} "
                f"wsClients={stats['wsClients']} binaryFrames={stats['binaryFrames']} binaryBytes={stats['binaryBytes']} "
                f"radarPositionSource={stats['radarPositionSource']} "
                f"workerPending={worker_stats['pending']} workerDropped={worker_stats['dropped']} "
                f"slowClientDrops={stats['wsDroppedSlowClient']} lastWorker={w_str}"
            )


threading.Thread(target=stats_timer_loop, daemon=True).start()


def submit_radar_message(msg: bytes) -> None:
    global last_radar_pos_from_vn, last_used_radar_position

    msg_type = struct.unpack_from(">I", msg, 4)[0]
    with state_lock:
        stats["total"] += 1
        stats["byType"][msg_type] = stats["byType"].get(msg_type, 0) + 1

    log_radar_message(msg)

    vn_pos_raw = vn.get_radar_position() if hasattr(vn, "get_radar_position") else None
    last_radar_pos_from_vn = normalize_pos(vn_pos_raw)
    maybe_send_loc_att_data_from_vn(active_client, last_radar_pos_from_vn)
    last_used_radar_position = get_best_radar_position()

    with state_lock:
        stats["radarPositionSource"] = last_used_radar_position.get("source", "none") if last_used_radar_position else "none"

    submitted = ldrs_worker.submit(
        msg,
        {
            "timestampMs": time.time() * 1000.0,
            "radarPosition": last_used_radar_position,
            "ldrsRadarPosition": last_radar_pos_from_ldrs,
            "requireLiveRadarPosition": REQUIRE_VECTORNAV_POSITION,
        },
    )

    with state_lock:
        if submitted:
            stats["submittedToWorker"] += 1
        else:
            stats["workerDropped"] = ldrs_worker.get_stats()["dropped"]


def start_radar_tcp_client() -> None:
    global active_client, search_requested

    print(f"[{iso_now()}] [LDRS] Connecting to {RADAR_HOST}:{RADAR_PORT}...")
    print("[LDRS] ICD byte order: BE/MSB-first")
    if AUTO_SEARCH:
        print("[LDRS] AUTO_SEARCH=true; will request SEARCH after STATUS reports STANDBY and ready.")
    if LOC_ATT_UPDATE_FROM_VN:
        print("[LDRS] LOC_ATT_UPDATE_FROM_VN=true; will send LOC_ATT_DATA from the first valid VectorNav fix.")
        if LOC_ATT_RESTART_AFTER_UPDATE:
            print(f"[LDRS] LOC_ATT_RESTART_AFTER_UPDATE=true; will send RESTART after {max(0, LOC_ATT_RESTART_DELAY_MS)}ms.")

    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((RADAR_HOST, RADAR_PORT))
            active_client = s
            search_requested = False
            print(f"[{iso_now()}] [LDRS] connected to {RADAR_HOST}:{RADAR_PORT}")
            buffer = b""

            while True:
                data = s.recv(4096)
                if not data:
                    print(f"[{iso_now()}] [LDRS] remote connection closed")
                    break

                buffer += data
                while len(buffer) >= MIN_MSG_SIZE:
                    msg_size = struct.unpack_from(">I", buffer, 12)[0]
                    if msg_size < MIN_MSG_SIZE or msg_size > MAX_MSG_SIZE:
                        print(f"[{iso_now()}] [LDRS] invalid msgSize={msg_size}; dropping buffered bytes")
                        buffer = b""
                        break

                    if len(buffer) < msg_size:
                        break

                    msg = buffer[:msg_size]
                    buffer = buffer[msg_size:]
                    submit_radar_message(msg)

        except Exception as e:
            print(f"[{iso_now()}] [LDRS] socket error: {e}")

        active_client = None
        print(f"[{iso_now()}] [LDRS] connection closed; reconnecting in 3s")
        time.sleep(3.0)


async def ws_handler(websocket):
    ws_clients.add(websocket)
    stats["wsClients"] = len(ws_clients)
    addr = websocket.remote_address
    print(f"[{iso_now()}] [WS] client connected {addr} clients={len(ws_clients)}")
    try:
        async for _ in websocket:
            pass
    except Exception:
        pass
    finally:
        ws_clients.discard(websocket)
        stats["wsClients"] = len(ws_clients)


async def run_ws_server():
    global ws_loop
    ws_loop = asyncio.get_running_loop()
    if websockets is None:
        print("[WS] 'websockets' library not installed. WebSocket broadcasting disabled.")
        return

    print(f"[{iso_now()}] [WS] Binary LDRS stream listening on ws://localhost:{WS_PORT}")
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        while True:
            await asyncio.sleep(3600)


def start_ws_thread():
    if websockets is None:
        return
    def _run():
        asyncio.run(run_ws_server())
    threading.Thread(target=_run, daemon=True).start()


def main():
    start_ws_thread()
    try:
        start_radar_tcp_client()
    except KeyboardInterrupt:
        print(f"[{iso_now()}] shutting down")
        if loc_att_restart_timer:
            loc_att_restart_timer.cancel()
        if active_client:
            active_client.close()
        if hasattr(vn, "stop"):
            vn.stop()
        ldrs_worker.terminate()
        sys.exit(0)


if __name__ == "__main__":
    main()
