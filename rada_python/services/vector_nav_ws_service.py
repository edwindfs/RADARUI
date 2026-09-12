"""VectorNav WebSocket communication service for remote telemetry feeds."""

import asyncio
from datetime import datetime, timezone
import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional

try:
    import websockets
    import websockets.sync.client as ws_sync
except ImportError:
    websockets = None
    ws_sync = None


class VectorNavWsService:
    """Service to connect to VectorNav WebSocket feed and parse position events."""

    def __init__(self, url: str, reconnect_ms: int = 2000):
        self.url = url
        self.reconnect_ms = reconnect_ms
        self.latest_position: Optional[Dict[str, Any]] = None
        self.latest_message: Optional[Dict[str, Any]] = None
        self.stopping = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.log_positions = bool(os.environ.get("VN_LOG_POSITIONS", "").lower() in ("1", "true", "yes"))

    def start(self) -> None:
        """Start background connection thread."""
        self.stopping = False
        self.thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.thread.start()

    def _connection_loop(self) -> None:
        if websockets is None:
            print("[VN:WS] WARNING: websockets library not installed. WebSocket feed disabled.")
            return

        reconnect_sec = max(0.5, self.reconnect_ms / 1000.0)

        while not self.stopping:
            try:
                # Use sync client if available in newer websockets, or run async loop
                if ws_sync is not None:
                    with ws_sync.connect(self.url) as ws:
                        print(f"[VN:WS] Connected: {self.url}")
                        for message in ws:
                            if self.stopping:
                                break
                            self.handle_message(message)
                else:
                    asyncio.run(self._async_connect())
            except Exception as e:
                if not self.stopping:
                    print(f"[VN:WS] connection closed or error ({e}); reconnecting in {self.reconnect_ms}ms")
            if not self.stopping:
                time.sleep(reconnect_sec)

    async def _async_connect(self) -> None:
        async with websockets.connect(self.url) as ws:
            print(f"[VN:WS] Connected: {self.url}")
            async for message in ws:
                if self.stopping:
                    break
                self.handle_message(message)

    def handle_message(self, data: Any) -> None:
        """Handle incoming raw JSON message and parse position."""
        try:
            message = json.loads(str(data))
        except Exception:
            return

        with self.lock:
            self.latest_message = message

        event = message.get("event")
        if event not in ("airwarden.position", "position"):
            return

        payload = message.get("payload") or {}
        info = payload.get("INFO") or {}

        # Latitude
        lat = payload.get("latitude")
        if lat is None:
            lat = payload.get("LAT")
        if lat is None:
            lat = info.get("ODID_loc_lat")

        # Longitude
        lon = payload.get("longitude")
        if lon is None:
            lon = payload.get("LON")
        if lon is None:
            lon = info.get("ODID_loc_lon")

        # Altitude
        alt = payload.get("altitude")
        if alt is None:
            alt = payload.get("ALTITUDE")
        if alt is None:
            alt = info.get("ODID_loc_geoAlt")
        if alt is None:
            alt = info.get("vectornav_altitude_orthometric_m", 0)

        # Heading
        heading = payload.get("heading")
        if heading is None:
            heading = payload.get("HEADING")
        if heading is None:
            heading = payload.get("yaw")
        if heading is None:
            heading = payload.get("YAW")
        if heading is None:
            heading = info.get("vectornav_heading_deg")

        pitch = payload.get("pitch") if payload.get("pitch") is not None else payload.get("PITCH")
        roll = payload.get("roll") if payload.get("roll") is not None else payload.get("ROLL")

        try:
            lat_f = float(lat)
            lon_f = float(lon)
            alt_f = float(alt)
        except (ValueError, TypeError):
            return

        if not (math.isfinite(lat_f) and math.isfinite(lon_f) and math.isfinite(alt_f)):
            return

        heading_f = float(heading) if heading is not None and math.isfinite(float(heading)) else None
        pitch_f = float(pitch) if pitch is not None and math.isfinite(float(pitch)) else None
        roll_f = float(roll) if roll is not None and math.isfinite(float(roll)) else None

        ts = (
            payload.get("receivedAt")
            or payload.get("TIME_STAMP")
            or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        )
        src = f"VectorNav:{payload['source']}" if payload.get("source") else "VectorNav:WS"

        pos = {
            "latDeg": lat_f,
            "lonDeg": lon_f,
            "altM": alt_f,
            "latitude": lat_f,
            "longitude": lon_f,
            "altitude": alt_f,
            "headingDeg": heading_f,
            "pitchDeg": pitch_f,
            "rollDeg": roll_f,
            "fix": payload.get("fix"),
            "satellites": payload.get("satellites"),
            "timestamp": ts,
            "source": src,
        }

        with self.lock:
            self.latest_position = pos

        if self.log_positions:
            print("[VN:WS] Position:", pos)

    def get_radar_position(self) -> Optional[Dict[str, Any]]:
        """Get latest parsed radar position."""
        with self.lock:
            return self.latest_position

    def get_last_message(self) -> Optional[Dict[str, Any]]:
        """Get last raw parsed message."""
        with self.lock:
            return self.latest_message

    def stop(self) -> None:
        """Stop connection thread."""
        self.stopping = True
