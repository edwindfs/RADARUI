"""VectorNav GNSS/INS serial communication and NMEA/VN parsing service."""

from datetime import datetime, timezone
import math
import os
import threading
import time
from typing import Any, Dict, Optional

try:
    import serial
except ImportError:
    serial = None


class VectorNavService:
    """Service to connect to VectorNav sensor via serial port and parse position data."""

    def __init__(self, port_path: str, baud_rate: int = 115200):
        self.port_path = port_path
        self.baud_rate = baud_rate

        self.latest_position: Optional[Dict[str, Any]] = None
        self.latest_line: Optional[str] = None
        self.port = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Optional mapping for VNINS
        vnins_lat = os.environ.get("VNINS_LAT_IDX")
        vnins_lon = os.environ.get("VNINS_LON_IDX")
        vnins_alt = os.environ.get("VNINS_ALT_IDX")
        self.vnins_lat_idx = int(vnins_lat) if vnins_lat else None
        self.vnins_lon_idx = int(vnins_lon) if vnins_lon else None
        self.vnins_alt_idx = int(vnins_alt) if vnins_alt else None

        self.counts: Dict[str, int] = {}
        self._last_stats_log = 0.0
        self.log_stats = bool(os.environ.get("VN_LOG_STATS", "").lower() in ("1", "true", "yes"))
        self.log_positions = bool(os.environ.get("VN_LOG_POSITIONS", "").lower() in ("1", "true", "yes"))

    def start(self) -> None:
        """Start serial reading thread."""
        if serial is None:
            print("[VN] WARNING: pyserial not installed. Serial communication disabled.")
            return

        self.running = True
        self.thread = threading.Thread(target=self._run_serial, daemon=True)
        self.thread.start()

    def _run_serial(self) -> None:
        try:
            self.port = serial.Serial(
                port=self.port_path,
                baudrate=self.baud_rate,
                timeout=1.0,
            )
            print(f"[VN] Connected: {self.port_path} @ {self.baud_rate}")
        except Exception as e:
            print(f"[VN] Serial connection error: {e}")
            return

        buffer = ""
        while self.running:
            try:
                line_bytes = self.port.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if not (line.startswith("$VN") or line.startswith("$GP") or line.startswith("$GN")):
                    continue

                with self.lock:
                    self.latest_line = line
                    msg_type = line.split("*")[0].split(",")[0]
                    self.counts[msg_type] = self.counts.get(msg_type, 0) + 1

                now = time.time()
                if self.log_stats and (now - self._last_stats_log > 5.0):
                    self._last_stats_log = now
                    print(f"[VN] msg counts: {self.counts}")
                    print(f"[VN] last line: {line}")

                pos = self.parse_position(line)
                if pos:
                    with self.lock:
                        self.latest_position = pos
                    if self.log_positions:
                        print(f"[VN] Position: {pos}")

            except Exception as e:
                if self.running:
                    print(f"[VN] Serial read error: {e}")
                time.sleep(0.5)

    def stop(self) -> None:
        """Stop serial thread and close port."""
        self.running = False
        if self.port:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None

    def get_radar_position(self) -> Optional[Dict[str, Any]]:
        """Get latest parsed radar position."""
        with self.lock:
            return self.latest_position

    def get_last_line(self) -> Optional[str]:
        """Get last raw line received."""
        with self.lock:
            return self.latest_line

    def parse_position(self, line: str) -> Optional[Dict[str, Any]]:
        """Parse position from NMEA or VectorNav sentence."""
        no_checksum = line.split("*")[0]
        parts = no_checksum.split(",")
        msg = parts[0]

        if msg in ("$GPGGA", "$GNGGA"):
            return self.parse_nmea_position(parts)

        # ---- A) $VNGPS
        if msg == "$VNGPS" and len(parts) > 5:
            try:
                lat = float(parts[3])
                lon = float(parts[4])
                alt = float(parts[5])
                if math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt):
                    return {
                        "latDeg": lat,
                        "lonDeg": lon,
                        "altM": alt,
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": alt,
                        "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                        "source": "VectorNav:$VNGPS",
                    }
            except (ValueError, IndexError):
                pass
            return None

        # ---- B) $VNINS
        if msg == "$VNINS":
            if self.vnins_lat_idx is None or self.vnins_lon_idx is None or self.vnins_alt_idx is None:
                return None

            try:
                lat = float(parts[self.vnins_lat_idx])
                lon = float(parts[self.vnins_lon_idx])
                alt = float(parts[self.vnins_alt_idx])
                if math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt):
                    return {
                        "latDeg": lat,
                        "lonDeg": lon,
                        "altM": alt,
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": alt,
                        "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                        "source": f"VectorNav:$VNINS(idx {self.vnins_lat_idx},{self.vnins_lon_idx},{self.vnins_alt_idx})",
                    }
            except (ValueError, IndexError):
                pass
            return None

        return None

    def nmea_to_decimal(self, raw: Optional[str], hemi: Optional[str]) -> Optional[float]:
        """Convert NMEA ddmm.mmmm / dddmm.mmmm to decimal degrees."""
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

    def parse_nmea_position(self, parts: list) -> Optional[Dict[str, Any]]:
        """Parse GPGGA / GNGGA parts into decimal coordinates."""
        if len(parts) < 10:
            return None

        lat = self.nmea_to_decimal(parts[2], parts[3])
        lon = self.nmea_to_decimal(parts[4], parts[5])

        try:
            quality = int(parts[6]) if parts[6] else 0
        except ValueError:
            quality = 0

        try:
            alt = float(parts[9]) if parts[9] else 0.0
        except ValueError:
            alt = 0.0

        if lat is None or lon is None or quality <= 0:
            return None

        return {
            "latDeg": lat,
            "lonDeg": lon,
            "altM": alt if math.isfinite(alt) else 0.0,
            "latitude": lat,
            "longitude": lon,
            "altitude": alt if math.isfinite(alt) else 0.0,
            "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "source": "GPS:NMEA:GGA",
            "quality": quality,
        }
