import socket
import struct
import threading
import time
import math
import os
import subprocess
import sys
import asyncio

# ensure the local directory is in the python path so rada_python can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import serial
except ImportError:
    serial = None

try:
    import websockets
except ImportError:
    websockets = None

from rada_python.ws.ldrs_binary_protocol import decode_tracks_frame
import pyproj

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

# RADA configuration
RADA_IP = os.getenv("RADA_IP", "192.168.170.1")
RADA_PORT = int(os.getenv("RADA_PORT", "7002"))
WS_URL = os.getenv("WS_URL", "ws://127.0.0.1:8070")
USE_ECHOSHIELD_GPS_FOR_RADA = os.getenv("USE_ECHOSHIELD_GPS_FOR_RADA", "0").lower() in ("1", "true", "yes")

VECTORNAV_COM_PORT = "COM5" 
VECTORNAV_BAUD_RATE = 115200

RADA_YAW = 0.0 # will be updated by VectorNav

class RadaManager:
    def __init__(self, c2_instance):
        self.c2 = c2_instance
        self.running = True
        self.vn_thread = None
        self.rada_thread = None
        self.cleanup_thread = None
        self.backend_process = None

    def start(self):
        # Start the rada_python backend subprocess
        env = os.environ.copy()
        env["VN_SOURCE"] = "none" # Let RADA.py handle VectorNav
        env["RADAR_HOST"] = RADA_IP
        env["RADAR_PORT"] = str(RADA_PORT)
        
        import echoshield
        if USE_ECHOSHIELD_GPS_FOR_RADA:
            env["MANUAL_RADAR_POSITION"] = "1"
            env["MANUAL_RADAR_LAT"] = str(echoshield.RADAR_LAT)
            env["MANUAL_RADAR_LON"] = str(echoshield.RADAR_LON)
            env["MANUAL_RADAR_ALT"] = str(echoshield.RADAR_ALT)
            print(f"[RADA] Using Echoshield GPS coordinates for RADA location: Lat={echoshield.RADAR_LAT}, Lon={echoshield.RADAR_LON}")
        
        print("[RADA] Starting rada_python backend subprocess...")
        cwd = os.path.dirname(os.path.abspath(__file__))
        self.backend_process = subprocess.Popen(
            [sys.executable, "-m", "rada_python.v1_ldr_client"],
            env=env,
            cwd=cwd
        )

        self.vn_thread = threading.Thread(target=self._vectornav_listener, daemon=True)
        self.vn_thread.start()

        self.rada_thread = threading.Thread(target=self._rada_listener_thread, daemon=True)
        self.rada_thread.start()

        self.cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
        self.cleanup_thread.start()

    def stop(self):
        self.running = False
        if self.backend_process:
            self.backend_process.terminate()
            try:
                self.backend_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.backend_process.kill()

    def _vectornav_listener(self):
        global RADA_YAW
        if serial is None:
            print("[RADA] pyserial not installed, cannot read VectorNav for heading.")
            return

        while self.running:
            try:
                with serial.Serial(VECTORNAV_COM_PORT, VECTORNAV_BAUD_RATE, timeout=1.0) as ser:
                    print(f"[RADA] Connected to VectorNav on {VECTORNAV_COM_PORT}")
                    while self.running:
                        line_bytes = ser.readline()
                        if not line_bytes:
                            continue
                        line = line_bytes.decode("utf-8", errors="replace").strip()
                        
                        # VNYMR: $VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ
                        if line.startswith("$VNYMR"):
                            parts = line.split("*")[0].split(",")
                            if len(parts) > 1:
                                try:
                                    yaw = float(parts[1])
                                    if yaw < 0:
                                        yaw += 360
                                    RADA_YAW = yaw
                                except ValueError:
                                    pass
                        # VNINS: $VNINS,Time,Mode,Yaw,Pitch,Roll,Lat,Lon,Alt,...
                        elif line.startswith("$VNINS"):
                            parts = line.split("*")[0].split(",")
                            if len(parts) > 3:
                                try:
                                    yaw = float(parts[3])
                                    if yaw < 0:
                                        yaw += 360
                                    RADA_YAW = yaw
                                except ValueError:
                                    pass
            except Exception as e:
                print(f"[RADA] VectorNav connection error on {VECTORNAV_COM_PORT}: {e}")
                time.sleep(3.0)

    def _rada_listener_thread(self):
        if websockets is None:
            print("[RADA] websockets library not installed. Cannot connect to backend WS.")
            return
            
        asyncio.run(self._ws_loop())

    async def _ws_loop(self):
        import echoshield
        geod = pyproj.Geod(ellps='WGS84')
        ecef_to_wgs84 = pyproj.Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)

        while self.running:
            print(f"[RADA] Connecting to WebSocket at {WS_URL}...")
            try:
                async with websockets.connect(WS_URL) as ws:
                    print(f"[RADA] Connected to rada_python backend via WebSocket.")
                    while self.running:
                        try:
                            data = await asyncio.wait_for(ws.recv(), timeout=1.0)
                        except asyncio.TimeoutError:
                            continue
                            
                        if isinstance(data, bytes):
                            self._process_ws_frame(data, geod, ecef_to_wgs84, echoshield)

            except Exception as e:
                print(f"[RADA] WebSocket connection error: {e}")
                
            if self.running:
                print(f"[RADA] Reconnecting in 3s...")
                await asyncio.sleep(3.0)

    def _cleanup_loop(self):
        while self.running:
            current_time = time.time()
            with self.c2.lock:
                stale_keys = [k for k, last_seen in list(self.c2.active_targets_last_seen.items()) if current_time - last_seen > 1.5 and k[0] == RADA_IP]
                for k in stale_keys:
                    self.c2.active_targets.pop(k, None)
                    self.c2.active_targets_last_seen.pop(k, None)
                    self.c2.target_kinematics.pop(k, None)
            time.sleep(0.5)
                
    def _process_ws_frame(self, data: bytes, geod, ecef_to_wgs84, echoshield):
        try:
            frame = decode_tracks_frame(data)
        except Exception as e:
            print(f"[RADA] Failed to decode WS frame: {e}")
            return
            
        for track in frame.get("tracks", []):
            track_id = track.get("trackId")
            if track_id is None:
                continue
                
            ecef = track.get("ecef")
            if not ecef:
                continue
                
            ecef_x, ecef_y, ecef_z = ecef.get("x", 0.0), ecef.get("y", 0.0), ecef.get("z", 0.0)
            if ecef_x == 0.0 and ecef_y == 0.0 and ecef_z == 0.0:
                continue
                
            lon, lat, alt_m = ecef_to_wgs84.transform(ecef_x, ecef_y, ecef_z)
            
            # compute relative distance and bearing to system center
            azimuth, _, distance = geod.inv(echoshield.RADAR_LON, echoshield.RADAR_LAT, lon, lat)
            bearing = (azimuth + 360) % 360
            rel_alt = alt_m - echoshield.RADAR_ALT
            
            classification = "Undeclared" # default fallback
            
            target_key = (RADA_IP, track_id)
            
            with self.c2.lock:
                self.c2.active_targets[target_key] = classification
                self.c2.active_targets_last_seen[target_key] = time.time()
                
                self.c2.target_kinematics[target_key] = {
                    "tid": track_id,
                    "ip": RADA_IP,
                    "lat": lat,
                    "lon": lon,
                    "alt": rel_alt,
                    "abs_alt": alt_m,
                    "range_m": distance,
                    "bearing_deg": bearing
                }
                
                if target_key == self.c2.selected_target_id:
                    self.c2.selected_target_details = self.c2.target_kinematics[target_key]

            self._log_track(RADA_IP, track_id, lat, lon, alt_m)

    def _log_track(self, ip, track_id, lat, lon, alt):
        try:
            log_path = "rada_output_log.csv"
            write_header = not os.path.exists(log_path)
            with open(log_path, "a") as log_file:
                if write_header:
                    log_file.write("Timestamp,Radar_IP,Track_ID,Latitude,Longitude,Altitude\n")
                timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S")
                log_file.write(f"{timestamp_str},{ip},{hex(track_id)},{lat:.7f},{lon:.7f},{alt:.1f}\n")
        except Exception:
            pass
