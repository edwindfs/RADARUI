"""LDRS Track Simulator emitting LDR1 binary frames over WebSocket."""

import asyncio
import math
import os
import signal
import sys
import time
from typing import Any, Dict, List, Set

from ..ws.ldrs_binary_protocol import create_tracks_frame

try:
    import websockets
except ImportError:
    websockets = None

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
NS_PER_MS = 1_000_000

WS_PORT = int(os.environ.get("SIM_WS_PORT") or os.environ.get("WS_PORT") or 8090)
INTERVAL_MS = int(os.environ.get("SIM_INTERVAL_MS", 500))
INCLUDE_ENU = bool(os.environ.get("XCR1_TRACK_INCLUDE_ENU", "true").lower() in ("1", "true", "yes"))
RADAR = {
    "latitude": float(os.environ.get("SIM_RADAR_LAT", 39.19572022)),
    "longitude": float(os.environ.get("SIM_RADAR_LON", -77.2583686)),
    "altitude": float(os.environ.get("SIM_RADAR_ALT", 140.00999450683594)),
}

started_at_ms = time.time() * 1000.0
generation = int(os.environ.get("SIM_GENERATION_START", 900000))
frames_sent = 0
connected_clients: Set[Any] = set()


def to_rad(deg: float) -> float:
    return (deg * math.pi) / 180.0


def lla_to_ecef(lla: Dict[str, float]) -> Dict[str, float]:
    lat = to_rad(lla["latitude"])
    lon = to_rad(lla["longitude"])
    alt = lla["altitude"]

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


def enu_vector_to_ecef(vector: Dict[str, float], origin: Dict[str, float]) -> Dict[str, float]:
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


def enu_to_ecef_position(enu_m: Dict[str, float], origin: Dict[str, float]) -> Dict[str, float]:
    origin_ecef = lla_to_ecef(origin)
    delta = enu_vector_to_ecef(enu_m, origin)
    return {
        "x": origin_ecef["x"] + delta["x"],
        "y": origin_ecef["y"] + delta["y"],
        "z": origin_ecef["z"] + delta["z"],
    }


def circular_track(track_id: int, t: float, radius_m: float, speed_rad: float, phase: float, up_m: float) -> Dict[str, Any]:
    a = t * speed_rad + phase
    return {
        "trackId": track_id,
        "enuM": {
            "e": math.cos(a) * radius_m,
            "n": math.sin(a) * radius_m,
            "u": up_m + math.sin(a * 0.7) * 8.0,
        },
        "enuVelMps": {
            "e": -math.sin(a) * radius_m * speed_rad,
            "n": math.cos(a) * radius_m * speed_rad,
            "u": math.cos(a * 0.7) * 8.0 * 0.7 * speed_rad,
        },
    }


def pass_track(track_id: int, t: float, offset_m: float, speed_mps: float, axis: str, period_s: float, up_m: float) -> Dict[str, Any]:
    half_span = (speed_mps * period_s) / 2.0
    cycle = ((t % period_s) / period_s) * 2.0 - 1.0
    moving = cycle * half_span
    direction = 1.0 if cycle >= 0 else -1.0
    if axis == "east":
        enu_m = {"e": moving, "n": offset_m, "u": up_m}
        enu_vel = {"e": direction * speed_mps, "n": 0.0, "u": 0.0}
    else:
        enu_m = {"e": offset_m, "n": moving, "u": up_m}
        enu_vel = {"e": 0.0, "n": direction * speed_mps, "u": 0.0}

    return {"trackId": track_id, "enuM": enu_m, "enuVelMps": enu_vel}


def build_track(definition: Dict[str, Any], now_ms: float) -> Dict[str, Any]:
    global generation
    last_update_ns = int(now_ms) * NS_PER_MS
    acquired_ns = int(started_at_ms + definition["trackId"] * 250) * NS_PER_MS
    lifetime = max(0.0, (now_ms - (acquired_ns / NS_PER_MS)) / 1000.0)
    ecef = enu_to_ecef_position(definition["enuM"], RADAR)
    ecef_vel = enu_vector_to_ecef(definition["enuVelMps"], RADAR)
    range_m = math.hypot(definition["enuM"]["e"], definition["enuM"]["n"], definition["enuM"]["u"])
    update_count = max(1, int(lifetime * (1000.0 / INTERVAL_MS)))

    return {
        "generation": generation,
        "trackId": definition["trackId"],
        "state": 0x2001,
        "lifetime": lifetime,
        "informedTrackUpdateCount": update_count,
        "ecef": ecef,
        "ecefVelMps": ecef_vel,
        "enuM": definition["enuM"],
        "rangeM": range_m,
        "lastUpdateTimeNs": last_update_ns,
        "lastAssocTimeNs": last_update_ns,
        "acquiredTimeNs": acquired_ns,
        "trackUuid": f"sim-ldrs-{definition['trackId']}",
        "positionId": f"sim-{generation}-{definition['trackId']}",
    }


def get_track_defs(now_ms: float) -> List[Dict[str, Any]]:
    t = (now_ms - started_at_ms) / 1000.0
    return [
        circular_track(track_id=1001, t=t, radius_m=120.0, speed_rad=0.12, phase=0.0, up_m=35.0),
        circular_track(track_id=1002, t=t, radius_m=210.0, speed_rad=-0.08, phase=1.6, up_m=60.0),
        pass_track(track_id=1003, t=t, offset_m=90.0, speed_mps=18.0, axis="east", period_s=28.0, up_m=45.0),
        pass_track(track_id=1004, t=t + 9.0, offset_m=-130.0, speed_mps=14.0, axis="north", period_s=34.0, up_m=30.0),
    ]


def make_frame() -> bytes:
    global generation
    now_ms = time.time() * 1000.0
    generation += 1
    defs = get_track_defs(now_ms)
    records = [build_track(d, now_ms) for d in defs]
    return create_tracks_frame(
        timestamp_ms=now_ms,
        radar_position_valid=True,
        include_enu=INCLUDE_ENU,
        records=records,
    )


async def handle_client(websocket):
    connected_clients.add(websocket)
    print(f"[SIM] client connected {websocket.remote_address} clients={len(connected_clients)}")
    try:
        # Send initial frame
        await websocket.send(make_frame())
        async for _ in websocket:
            pass
    except Exception:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"[SIM] client disconnected clients={len(connected_clients)}")


async def broadcast_loop():
    global frames_sent
    interval_sec = max(0.05, INTERVAL_MS / 1000.0)
    log_cadence = max(1, round(5000 / INTERVAL_MS))

    while True:
        await asyncio.sleep(interval_sec)
        frame = make_frame()
        sent_clients = 0

        if connected_clients:
            tasks = []
            for client in list(connected_clients):
                try:
                    tasks.append(client.send(frame))
                    sent_clients += 1
                except Exception:
                    pass
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)

        frames_sent += 1
        if frames_sent % log_cadence == 0:
            print(f"[SIM] frames={frames_sent} clients={len(connected_clients)} tracks=4 bytes={len(frame)}")


async def main():
    if websockets is None:
        print("[SIM] ERROR: 'websockets' module is not installed. Please install it with: pip install websockets")
        # Run local generation demo loop
        print("[SIM] Running in standalone headless generator mode...")
        global frames_sent
        interval_sec = max(0.05, INTERVAL_MS / 1000.0)
        while True:
            await asyncio.sleep(interval_sec)
            frame = make_frame()
            frames_sent += 1
            if frames_sent % max(1, round(5000 / INTERVAL_MS)) == 0:
                print(f"[SIM HEADLESS] frames={frames_sent} tracks=4 bytes={len(frame)}")

    print(f"[SIM] LDRS track simulator listening on ws://localhost:{WS_PORT}")
    print(f"[SIM] radarPosition lat={RADAR['latitude']} lon={RADAR['longitude']} alt={RADAR['altitude']} includeEnu={INCLUDE_ENU}")

    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT):
        await broadcast_loop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[SIM] shutting down")
