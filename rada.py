import socket
import struct
import threading
import time
import math
import os

try:
    import serial
except ImportError:
    serial = None

# RADA Configuration
RADA_IP = "192.168.170.1"
RADA_PORT = 7002

VECTORNAV_COM_PORT = "COM5" # Default Windows COM port, can be changed
VECTORNAV_BAUD_RATE = 115200

RADA_YAW = 0.0 # Will be updated by VectorNav

# LDRS Message constants
MSG_TRACKS = 10
MSG_TRACKS_EXTENDED = 20
MIN_MSG_SIZE = 16
TRACKS_BASE_OFFSET = 48
TRACK_SLOT_SIZE = 232
TRACK_LAT_OFFSET = 48
TRACK_LON_OFFSET = 56
TRACK_ALT_OFFSET = 64
RAD2DEG = 180.0 / math.pi

class RadaManager:
    def __init__(self, c2_instance):
        self.c2 = c2_instance
        self.running = True
        self.vn_thread = None
        self.rada_thread = None

    def start(self):
        self.vn_thread = threading.Thread(target=self._vectornav_listener, daemon=True)
        self.vn_thread.start()

        self.rada_thread = threading.Thread(target=self._rada_listener, daemon=True)
        self.rada_thread.start()

    def stop(self):
        self.running = False

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
                                    # convert to 0-360 true north heading format if necessary
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

    def _rada_listener(self):
        import pyproj
        import echoshield
        geod = pyproj.Geod(ellps='WGS84')

        while self.running:
            print(f"[RADA] Connecting to RADA radar at {RADA_IP}:{RADA_PORT}...")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((RADA_IP, RADA_PORT))
                s.settimeout(1.0)
                print(f"[RADA] Connected to RADA radar.")
                
                buffer = b""
                
                while self.running:
                    try:
                        data = s.recv(4096)
                        if not data:
                            print(f"[RADA] Connection closed by radar.")
                            break
                        
                        buffer += data
                        while len(buffer) >= MIN_MSG_SIZE:
                            msg_size = struct.unpack_from(">I", buffer, 12)[0]
                            if msg_size < MIN_MSG_SIZE or msg_size > 2000000:
                                buffer = b"" # drop invalid
                                break
                            
                            if len(buffer) < msg_size:
                                break
                            
                            msg = buffer[:msg_size]
                            buffer = buffer[msg_size:]
                            
                            self._process_rada_message(msg, geod, echoshield)
                            
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[RADA] Read error: {e}")
                        break
                        
            except Exception as e:
                print(f"[RADA] Connection error: {e}")
                
            if self.running:
                print(f"[RADA] Reconnecting in 3s...")
                time.sleep(3.0)
                
    def _process_rada_message(self, msg, geod, echoshield):
        if len(msg) < 16:
            return
            
        msg_counter, msg_type, _, msg_size = struct.unpack_from(">4I", msg, 0)
        
        if msg_type in (MSG_TRACKS, MSG_TRACKS_EXTENDED):
            if len(msg) < TRACKS_BASE_OFFSET:
                return
                
            num_tracks = struct.unpack_from(">H", msg, 30)[0]
            
            for i in range(num_tracks):
                base = TRACKS_BASE_OFFSET + i * TRACK_SLOT_SIZE
                needed = base + TRACK_ALT_OFFSET + 4
                if needed > len(msg):
                    break
                    
                track_id = struct.unpack_from(">I", msg, base)[0]
                lat_rad = struct.unpack_from(">d", msg, base + TRACK_LAT_OFFSET)[0]
                lon_rad = struct.unpack_from(">d", msg, base + TRACK_LON_OFFSET)[0]
                alt_m = struct.unpack_from(">f", msg, base + TRACK_ALT_OFFSET)[0]
                
                lat = lat_rad * RAD2DEG
                lon = lon_rad * RAD2DEG
                
                # compute relative distance and bearing to system center
                azimuth, _, distance = geod.inv(echoshield.RADAR_LON, echoshield.RADAR_LAT, lon, lat)
                bearing = (azimuth + 360) % 360
                
                classification = "UAV (Multi-Rotor)" # default fallback if not parsed
                
                target_key = (RADA_IP, track_id)
                
                with self.c2.lock:
                    self.c2.active_targets[target_key] = classification
                    self.c2.active_targets_last_seen[target_key] = time.time()
                    
                    self.c2.target_kinematics[target_key] = {
                        "tid": track_id,
                        "ip": RADA_IP,
                        "lat": lat,
                        "lon": lon,
                        "alt": alt_m,
                        "range_m": distance,
                        "bearing_deg": bearing
                    }
                    
                    if target_key == self.c2.selected_target_id:
                        self.c2.selected_target_details = self.c2.target_kinematics[target_key]

                # logging
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
