import socket
import struct
import pyproj
import time
import numpy as np
import serial
import json
import threading

from radario.echoshield.client import EchoShieldClient
from radario.parsers import Parser
from radario.base import RadarProduct

def is_reachable(ip, port=29982, timeout=0.5):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except:
        return False

# configuration
RADAR_IPS = ["192.168.1.150", "192.168.1.151", "192.168.1.153", "192.168.1.154"]
TRACK_PORT = 29982
ecef_to_wgs84 = pyproj.Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
geod = pyproj.Geod(ellps='WGS84')

# default location
RADAR_LAT = 41.861500417
RADAR_LON = -72.61243207
RADAR_ALT = 13.644
RADAR_YAW = 201.7164306640625
RADAR_PITCH = 19.672571182250977
RADAR_ROLL = -0.19416861236095428

# update with the correct COM port where radio is connected
RADIO_COM_PORT = "COM4"
RADIO_BAUD_RATE = 57600

class PrimaryC2:
    def __init__(self):
        self.authorized_target_id = None
        self.selected_target_id = None
        
        self.active_targets = {} # {(ip, id): classification}
        self.active_targets_last_seen = {} # {(ip, id): timestamp}
        self.selected_target_details = {} # {tid: track_id, ip: str, lat: float, lon: float, alt: float, range_m: float, bearing_deg: float}
        self.target_kinematics = {} # {(ip, id): {tid: track_id, ip: str, lat: float, lon: float, alt: float, range_m: float, bearing_deg: float}}
        self.fused_targets = {} # {fused_id: {'tids': set(), 'classification': str, 'lat': float, ...}}
        self.next_fused_id = 1
        self.fusion_threshold = 20.0
        self.target_locked = False
        
        self.running = True
        self.serial_conn = None
        self.lock = threading.Lock()
        self.clients = [] # keeps initilized clients alive
        self.radar_specific_yaws = {}
        
        self.radio_thread = threading.Thread(target=self.continuous_radio_worker, daemon=True)
        self.radio_thread.start()

    def fetch_and_update_gps(self): # extracts gps data from all radars
        global RADAR_LAT, RADAR_LON, RADAR_ALT, RADAR_YAW, RADAR_PITCH, RADAR_ROLL
        success = False
        radar_yaws = []
        self.radar_specific_yaws = {}
        for ip in RADAR_IPS:
            if not is_reachable(ip, TRACK_PORT, 0.5):
                print(f"[*] Skipping {ip} for GPS as it is unreachable.")
                continue
            print(f" Connecting to Echodyne radar at {ip} to fetch GPS data..")
            client = EchoShieldClient(host=ip)
            try:
                client.connect()
                ins_data = client.get_ins_data()
                print(f"\n[+] INS / GPS Data Extracted from {ip}:")
                print(ins_data)
                
                if not success:
                    RADAR_LAT = ins_data.get('latitude_deg', RADAR_LAT)
                    RADAR_LON = ins_data.get('longitude_deg', RADAR_LON)
                    RADAR_ALT = ins_data.get('altitude_m', RADAR_ALT)
                    RADAR_PITCH = ins_data.get('pitchback_deg', RADAR_PITCH)
                    RADAR_ROLL = ins_data.get('tilt_deg', RADAR_ROLL)
                    success = True
                
                heading = ins_data.get('heading_deg')
                if heading is not None:
                    radar_yaws.append(heading)
                    self.radar_specific_yaws[ip] = heading
                
                client.disconnect()
            except Exception as e:
                print(f" Failed to fetch INS data from {ip}: {e}")
            finally:
                if hasattr(client, "disconnect"):
                    try:
                        client.disconnect()
                    except:
                        pass
                        
        if radar_yaws: # averages the heading of all radars
            import math
            sum_sin = sum(math.sin(math.radians(y)) for y in radar_yaws)
            sum_cos = sum(math.cos(math.radians(y)) for y in radar_yaws)
            RADAR_YAW = math.degrees(math.atan2(sum_sin, sum_cos)) % 360
            print(f" Successfully updated System Center kinematics. Average Radar Heading: {RADAR_YAW:.2f}°")
        elif not success:
            print(" All radars failed to provide GPS. Falling back to default fixed coordinates.")

    def initialize_radar(self): #initializes all radars
        for i, ip in enumerate(RADAR_IPS):
            if not is_reachable(ip, TRACK_PORT, 0.5):
                print(f" Skipping initialization for {ip} as it is unreachable.")
                continue
            print(f" Connecting to Command Port via RadarIO at {ip}")
            client = EchoShieldClient(host=ip)
            try:
                client.connect()
                
                print(f" Configuring Radar Mission for {ip}")
                with client.service_mode():
                    client.configure_mission(mission="C-UAS_1")
                    channel_str = f"U{1+i*3}" # U1, U4, U7, U10 for 4 radars
                    print(f" Setting Radar Channel to {channel_str} for {ip}...")
                    client.set(tx_channel=channel_str)

                
                print(f" Setting Radar Kinematics for {ip}...")
                client.set_radar_kinematics(
                    latitude_deg=RADAR_LAT,
                    longitude_deg=RADAR_LON,
                    altitude_m=RADAR_ALT,
                    heading_deg=self.radar_specific_yaws.get(ip, RADAR_YAW),
                    pitchback_deg=RADAR_PITCH,
                    tilt_deg=RADAR_ROLL,
                    altitude_type="orthometric",
                    unix_time_ns=time.time_ns()
                )

                print(f" Starting Radar Transmission on {ip}...")
                client.mode_set_start() 
                print(f"Radar {ip} is now transmitting.")
                self.clients.append(client)
            except Exception as e:
                print(f"[Warning] Failed to initialize radar {ip} fully: {e}")
                
        # starts fusion thread
        threading.Thread(target=self.track_fusion_thread, daemon=True).start()

    def track_fusion_thread(self):
        import math
        import pyproj
        geod = pyproj.Geod(ellps='WGS84')
        
        while self.running:
            time.sleep(0.1)
            with self.lock:
                # clean up stale raw targets from existing fused groups
                for fid, fdata in list(self.fused_targets.items()):
                    active_tids = [tid for tid in fdata['tids'] if tid in self.active_targets]
                    
                    # if members drift too far from the group center, kick them out
                    valid_tids = set()
                    for tid in active_tids:
                        k1 = self.target_kinematics.get(tid)
                        if k1:
                            dx = k1['range_m'] * math.sin(math.radians(k1['bearing_deg'])) - fdata['range_m'] * math.sin(math.radians(fdata['bearing_deg']))
                            dy = k1['range_m'] * math.cos(math.radians(k1['bearing_deg'])) - fdata['range_m'] * math.cos(math.radians(fdata['bearing_deg']))
                            dz = k1['alt'] - fdata['alt']
                            dist = math.sqrt(dx**2 + dy**2 + dz**2)
                            
                            # allow them to be 1.5x the threshold from the center before splitting
                            if dist <= self.fusion_threshold * 1.5:
                                valid_tids.add(tid)
                                
                    fdata['tids'] = valid_tids
                    if not fdata['tids']:
                        del self.fused_targets[fid]
                        
                # merge existing fused groups that are within the threshold
                fids = list(self.fused_targets.keys())
                for i in range(len(fids)):
                    fid1 = fids[i]
                    if fid1 not in self.fused_targets: continue
                    f1 = self.fused_targets[fid1]
                    
                    for j in range(i + 1, len(fids)):
                        fid2 = fids[j]
                        if fid2 not in self.fused_targets: continue
                        f2 = self.fused_targets[fid2]
                        
                        dx = f1['range_m'] * math.sin(math.radians(f1['bearing_deg'])) - f2['range_m'] * math.sin(math.radians(f2['bearing_deg']))
                        dy = f1['range_m'] * math.cos(math.radians(f1['bearing_deg'])) - f2['range_m'] * math.cos(math.radians(f2['bearing_deg']))
                        dz = f1['alt'] - f2['alt']
                        dist = math.sqrt(dx**2 + dy**2 + dz**2)
                        
                        if dist <= self.fusion_threshold:
                            f1['tids'].update(f2['tids'])
                            del self.fused_targets[fid2]
                        
                # find which raw targets are already assigned
                assigned_tids = set()
                for fdata in self.fused_targets.values():
                    assigned_tids.update(fdata['tids'])
                    
                unassigned = set(self.active_targets.keys()) - assigned_tids
                
                # assigns or creates fused tracks for unassigned raw targets
                for tid in unassigned:
                    k1 = self.target_kinematics.get(tid)
                    if not k1: continue
                    
                    best_fid = None
                    best_dist = self.fusion_threshold
                    
                    for fid, fdata in self.fused_targets.items():
                        # calculates euclidean distance relative to radar center
                        dx = k1['range_m'] * math.sin(math.radians(k1['bearing_deg'])) - fdata['range_m'] * math.sin(math.radians(fdata['bearing_deg']))
                        dy = k1['range_m'] * math.cos(math.radians(k1['bearing_deg'])) - fdata['range_m'] * math.cos(math.radians(fdata['bearing_deg']))
                        dz = k1['alt'] - fdata['alt']
                        dist = math.sqrt(dx**2 + dy**2 + dz**2)
                        
                        if dist <= best_dist:
                            best_dist = dist
                            best_fid = fid
                            
                    if best_fid is not None:
                        self.fused_targets[best_fid]['tids'].add(tid)
                    else:
                        self.fused_targets[self.next_fused_id] = {
                            'tids': {tid},
                            'lat': k1['lat'], 'lon': k1['lon'], 'alt': k1['alt'],
                            'range_m': k1['range_m'], 'bearing_deg': k1['bearing_deg'],
                            'classification': self.active_targets[tid]
                        }
                        self.next_fused_id += 1
                        
                # updates kinematics for all fused tracks based on their raw members
                for fid, fdata in self.fused_targets.items():
                    valid_tids = [t for t in fdata['tids'] if t in self.target_kinematics]
                    if not valid_tids: continue
                    
                    avg_lat = sum(self.target_kinematics[t]['lat'] for t in valid_tids) / len(valid_tids)
                    avg_lon = sum(self.target_kinematics[t]['lon'] for t in valid_tids) / len(valid_tids)
                    avg_alt = sum(self.target_kinematics[t]['alt'] for t in valid_tids) / len(valid_tids)
                    
                    azimuth, _, distance = geod.inv(RADAR_LON, RADAR_LAT, avg_lon, avg_lat)
                    bearing = (azimuth + 360) % 360
                    
                    fdata['lat'] = avg_lat
                    fdata['lon'] = avg_lon
                    fdata['alt'] = avg_alt
                    fdata['range_m'] = distance
                    fdata['bearing_deg'] = bearing
                    classifications = [self.active_targets[t] for t in valid_tids]
                    if "UAV (Multi-Rotor)" in classifications:
                        fdata['classification'] = "UAV (Multi-Rotor)"
                    elif "UAV (Fixed-Wing)" in classifications:
                        fdata['classification'] = "UAV (Fixed-Wing)"
                    else:
                        fdata['classification'] = classifications[0]
            
    def track_listener_thread(self, radar_ip):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        print(f" Connecting to Track Stream {radar_ip}:{TRACK_PORT}...")
        try:
            sock.connect((radar_ip, TRACK_PORT))
        except Exception as e:
            print(f"[Error] Could not connect to {radar_ip}:{TRACK_PORT} - {e}")
            return

        track_parser = Parser("tracks", RadarProduct.EchoShield)
        track_blueprint = track_parser.fixed
        expected_size = track_blueprint.itemsize
        buffer = b""

        sock.settimeout(0.5)
        print(f"[*] Track Stream Connected on {radar_ip}. Awaiting Targets...\n")

        while self.running:
            # clean up stale targets for this radar (not seen in > 1.5 seconds)
            current_time = time.time()
            with self.lock: # threading lock to prevent crashing
                stale_keys = [k for k, last_seen in list(self.active_targets_last_seen.items()) if current_time - last_seen > 1.5 and k[0] == radar_ip]
                for k in stale_keys:
                    self.active_targets.pop(k, None)
                    self.active_targets_last_seen.pop(k, None)
                    self.target_kinematics.pop(k, None)
                    
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Warning] Socket read error: {e}")
                time.sleep(1)
                continue
                
            if not data:
                print("[Warning] No data received. Check radar connection.")
                time.sleep(1)
                continue

            buffer += data

            while True:
                start_idx = buffer.find(b'<tracks>')

                if start_idx == -1:
                    buffer = buffer[-7:]
                    break

                if len(buffer) < start_idx + 12:
                    break

                packet_size = struct.unpack_from('<I', buffer[start_idx + 8:start_idx + 12])[0]
                
                if packet_size > 1000000:
                    print(f"[Warning] Invalid packet size {packet_size}. Discarding buffer.")
                    buffer = b""
                    break

                if len(buffer) < start_idx + packet_size:
                    break

                raw_packet = buffer[start_idx:start_idx + packet_size]
                buffer = buffer[start_idx + packet_size:]
                safe_packet = raw_packet[:expected_size].ljust(expected_size, b'\x00') 
                parsed_data = np.frombuffer(safe_packet, dtype=track_blueprint)[0]

                track_id = int(parsed_data['id'])

                if track_id == 0:
                    continue 

                probs = {
                    "UAV (Multi-Rotor)": float(parsed_data['prob_uav_multirotor']),
                    "UAV (Fixed-Wing)": float(parsed_data['prob_uav_fixedwing']),
                    "Walker": float(parsed_data['prob_human']),
                    "Plane": float(parsed_data['prob_aircraft']),
                    "Bird": float(parsed_data['prob_bird']),
                    "Vehicle": float(parsed_data['prob_vehicle']),
                    "Clutter": float(parsed_data['prob_clutter']),
                }

                best_match = max(probs, key=probs.get)
                if probs[best_match] == 0:
                    best_match = "Undeclared"

                target_key = (radar_ip, track_id)
                with self.lock:
                    self.active_targets[target_key] = best_match
                    self.active_targets_last_seen[target_key] = time.time()

                try:
                    ecef_x, ecef_y, ecef_z = parsed_data['ecef_pos_est']
                    lon, lat, _ = ecef_to_wgs84.transform(ecef_x, ecef_y, ecef_z)
                    
                    # extract altitude relative to radar directly from ENU (Up) vector
                    _, _, up = parsed_data['enu_pos_est']
                    rel_alt = float(up) 
                    
                    azimuth, _, distance = geod.inv(RADAR_LON, RADAR_LAT, lon, lat)
                    bearing = (azimuth + 360) % 360
                    
                    with self.lock:
                        self.target_kinematics[target_key] = {
                            "tid": track_id,
                            "ip": radar_ip,
                            "lat": lat,
                            "lon": lon,
                            "alt": rel_alt,
                            "range_m": distance,
                            "bearing_deg": bearing
                        }
                        
                        if target_key == self.selected_target_id:
                            self.selected_target_details = self.target_kinematics[target_key]

                    is_locked = False
                    if target_key == self.authorized_target_id:
                        is_locked = True
                    elif isinstance(self.authorized_target_id, int):
                        fused_group = self.fused_targets.get(self.authorized_target_id)
                        if fused_group and target_key in fused_group.get('tids', set()):
                            is_locked = True
                            
                    if is_locked:
                        print(f"[TARGET LOCKED] {radar_ip} : {hex(track_id)} | Lat: {lat:.5f}, Lon: {lon:.5f}, Alt: {rel_alt:.1f}m")
                        self.target_locked = True
                        
                        # log the coordinate output
                        try:
                            import os
                            log_path = "radar_output_log.csv"
                            write_header = not os.path.exists(log_path)
                            with open(log_path, "a") as log_file:
                                if write_header:
                                    log_file.write("Timestamp,Radar_IP,Track_ID,Latitude,Longitude,Altitude\n")
                                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                                log_file.write(f"{timestamp},{radar_ip},{hex(track_id)},{lat:.7f},{lon:.7f},{rel_alt:.1f}\n")
                        except Exception as e:
                            pass # suppress log errors

                except Exception as e:
                    print(f"[Warning] MATH FAILED: {e}")
                        
        sock.close()

    def send_coordinate_once(self, target_key):
        import json
        with self.lock:
            if isinstance(target_key, int):
                details = self.fused_targets.get(target_key)
            else:
                details = self.target_kinematics.get(target_key)
                
            if not details:
                print(f"[Warning] Cannot send one-shot: Target {target_key} not found.")
                return False
                
            lat = details.get('lat')
            lon = details.get('lon')
            alt = details.get('alt')
            tid_hex = hex(details.get('tid', target_key if isinstance(target_key, int) else target_key[1]))
            
        try: 
            if self.serial_conn is None:
                self.serial_conn = serial.Serial(RADIO_COM_PORT, RADIO_BAUD_RATE, timeout=1)
            
            payload = {
                "tid": tid_hex,
                "latitude": round(lat, 7),
                "longitude": round(lon, 7),
                "altitude_m": round(alt, 1)
            }

            json_string=json.dumps(payload) + "\n"
            self.serial_conn.write(json_string.encode('utf-8'))
            print(f"[ONE-SHOT] Sent target coordinates over serial: {json_string.strip()}")  
            
            try:
                import time
                with open("serial_output_log.txt", "a") as serial_log:
                    serial_log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] ONE-SHOT: {json_string}")
            except Exception:
                pass
                
            return True

        except Exception as e:
            print(f"[Error] Failed to send one-shot coordinate: {e}")
            return False

    def continuous_radio_worker(self):
        import json
        while self.running:
            time.sleep(0.2) # ~5Hz
            
            with self.lock:
                if not self.target_locked or not self.authorized_target_id:
                    continue
                    
                target_key = self.authorized_target_id
                if isinstance(target_key, int):
                    details = self.fused_targets.get(target_key)
                else:
                    details = self.target_kinematics.get(target_key)
                    
                if not details:
                    continue
                    
                lat = details.get('lat')
                lon = details.get('lon')
                alt = details.get('alt')
                tid_hex = hex(details.get('tid', target_key if isinstance(target_key, int) else target_key[1]))
                
            try:
                if self.serial_conn is None:
                    self.serial_conn = serial.Serial(RADIO_COM_PORT, RADIO_BAUD_RATE, timeout=1)
                    
                payload = {
                    "tid": tid_hex,
                    "latitude": round(lat, 7),
                    "longitude": round(lon, 7),
                    "altitude_m": round(alt, 1)
                }
                
                json_string = json.dumps(payload) + "\n"
                self.serial_conn.write(json_string.encode('utf-8'))
                
                try:
                    with open("serial_output_log.txt", "a") as serial_log:
                        serial_log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] CONTINUOUS: {json_string}")
                except Exception:
                    pass
            except Exception as e:
                pass # suppress serial errors
