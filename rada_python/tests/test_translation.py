"""Comprehensive unit and integration test suite for RADA Python translation."""

import json
import math
import struct
import time
import unittest

from rada_python.utils.geo import (
    to_rad,
    haversine_meters,
    bearing_degrees,
    distance_from_radar,
)
from rada_python.utils.direction import get_direction_from_bearing
from rada_python.formatters.build_ldrs_detection import (
    build_ldrs_detection,
    classification_from_family,
    format_active_time,
    threat_from_distance_m,
)
from rada_python.ws.ldrs_binary_protocol import (
    create_tracks_frame,
    decode_tracks_frame,
    HEADER_FLAGS,
    HEADER_LENGTH,
    MAGIC,
    VERSION,
)
from rada_python.workers.ldrs_processing_worker import (
    lla_to_ecef,
    ecef_delta_to_enu,
    enu_vector_to_ecef,
    enu_to_ecef_position,
    decode_status,
    decode_extended_status,
    decode_tracks_time_tag_ms,
    process_message,
)
from rada_python.workers.ldrs_worker_client import LdrsWorkerClient
from rada_python.services.vector_nav_service import VectorNavService
from rada_python.services.vector_nav_ws_service import VectorNavWsService


class TestGeoUtils(unittest.TestCase):
    def test_to_rad(self):
        self.assertAlmostEqual(to_rad(180.0), math.pi)
        self.assertAlmostEqual(to_rad(90.0), math.pi / 2.0)

    def test_haversine(self):
        # Distance between two known points in meters
        p1 = {"latitude": 41.8615, "longitude": -72.6124, "altitude": 10.0}
        p2 = {"latitude": 41.8700, "longitude": -72.6124, "altitude": 10.0}
        dist = haversine_meters(p1, p2)
        # 0.0085 deg lat is approx 945 meters
        self.assertTrue(900.0 < dist < 1000.0)

    def test_bearing(self):
        p1 = {"latitude": 40.0, "longitude": -70.0}
        p2 = {"latitude": 41.0, "longitude": -70.0}  # Due north
        brg = bearing_degrees(p1, p2)
        self.assertAlmostEqual(brg, 0.0, places=1)

        p3 = {"latitude": 40.0, "longitude": -69.0}  # Due east approx
        brg_east = bearing_degrees(p1, p3)
        self.assertTrue(88.0 < brg_east < 92.0)

    def test_distance_from_radar(self):
        radar = {"latitude": 40.0, "longitude": -70.0, "altitude": 100.0}
        target = {"latitude": 40.0, "longitude": -70.0, "altitude": 200.0}
        dfr = distance_from_radar(radar, target)
        self.assertAlmostEqual(dfr["groundDistance_m"], 0.0, places=1)
        self.assertAlmostEqual(dfr["slantDistance_m"], 100.0, places=1)


class TestDirectionUtils(unittest.TestCase):
    def test_cardinals(self):
        self.assertEqual(get_direction_from_bearing(0.0), "N")
        self.assertEqual(get_direction_from_bearing(45.0), "NE")
        self.assertEqual(get_direction_from_bearing(90.0), "E")
        self.assertEqual(get_direction_from_bearing(135.0), "SE")
        self.assertEqual(get_direction_from_bearing(180.0), "S")
        self.assertEqual(get_direction_from_bearing(225.0), "SW")
        self.assertEqual(get_direction_from_bearing(270.0), "W")
        self.assertEqual(get_direction_from_bearing(315.0), "NW")
        self.assertEqual(get_direction_from_bearing(359.0), "N")


class TestFormatter(unittest.TestCase):
    def test_build_ldrs_detection(self):
        radar = {"latitude": 41.0, "longitude": -73.0, "altitude": 50.0}
        track = {
            "trackId": 42,
            "latitude": 41.01,
            "longitude": -73.01,
            "altitude": 150.0,
            "velocity": 25.5,
            "targetFamily": 2,
            "timestamp": "2026-09-06T12:00:00.000Z",
            "messageType": 10,
            "statusFlags": 0x2001,
        }
        det = build_ldrs_detection(track, radar, started_at_ms=time.time() * 1000.0 - 65000)

        self.assertEqual(det["radarType"], "LDRS")
        self.assertEqual(det["id"], 42)
        self.assertEqual(det["classification"], "ABT/UAV")
        self.assertEqual(det["altitude"]["value"], 150)
        self.assertEqual(det["velocity"]["value"], 25.5)
        self.assertEqual(det["status"], "tracked")
        self.assertIn("latituade", det["radarPosition"])
        self.assertEqual(det["radarPosition"]["latitude"], 41.0)
        self.assertEqual(det["system"]["activeTime"], "1min 5s")


class TestBinaryProtocol(unittest.TestCase):
    def test_roundtrip_with_enu(self):
        timestamp_ms = 1700000000123.0
        records = [
            {
                "generation": 100,
                "trackId": 101,
                "state": 0x2001,
                "lifetime": 12.34,
                "informedTrackUpdateCount": 45,
                "ecef": {"x": 1234567.89, "y": -4567890.12, "z": 3456789.01},
                "ecefVelMps": {"x": 10.5, "y": -20.25, "z": 5.0},
                "enuM": {"e": 100.0, "n": 200.0, "u": 50.0},
                "rangeM": 229.13,
                "lastUpdateTimeNs": 1700000000123000000,
                "lastAssocTimeNs": 1700000000120000000,
                "acquiredTimeNs": 1699999987000000000,
                "trackUuid": "ldrs-101",
                "positionId": "ldrs-100-101",
            }
        ]

        frame = create_tracks_frame(
            timestamp_ms=timestamp_ms,
            radar_position_valid=True,
            records=records,
            include_enu=True,
        )

        self.assertTrue(frame.startswith(MAGIC))
        decoded = decode_tracks_frame(frame)

        self.assertEqual(decoded["version"], VERSION)
        self.assertEqual(decoded["messageType"], 1)
        self.assertEqual(decoded["timestampMs"], int(timestamp_ms))
        self.assertTrue(decoded["radarPositionValid"])
        self.assertTrue(decoded["includeEnu"])
        self.assertEqual(decoded["count"], 1)

        t0 = decoded["tracks"][0]
        self.assertEqual(t0["trackId"], 101)
        self.assertEqual(t0["generation"], 100)
        self.assertEqual(t0["state"], 0x2001)
        self.assertAlmostEqual(t0["lifetime"], 12.34, places=2)
        self.assertEqual(t0["informedTrackUpdateCount"], 45)
        self.assertAlmostEqual(t0["ecef"]["x"], 1234567.89, places=2)
        self.assertAlmostEqual(t0["enuM"]["e"], 100.0, places=2)
        self.assertEqual(t0["trackUuid"], "ldrs-101")
        self.assertEqual(t0["positionId"], "ldrs-100-101")

    def test_roundtrip_without_enu(self):
        timestamp_ms = 1700000000456.0
        records = [
            {
                "generation": 200,
                "trackId": 202,
                "state": 0x1000,
                "lifetime": 5.0,
                "informedTrackUpdateCount": 10,
                "ecef": {"x": 1000.0, "y": 2000.0, "z": 3000.0},
                "ecefVelMps": {"x": 1.0, "y": 2.0, "z": 3.0},
                "rangeM": 500.0,
                "lastUpdateTimeNs": 1000000,
                "lastAssocTimeNs": 1000000,
                "acquiredTimeNs": 500000,
                "trackUuid": "uuid-202",
                "positionId": "pos-202",
            }
        ]

        frame = create_tracks_frame(
            timestamp_ms=timestamp_ms,
            radar_position_valid=False,
            records=records,
            include_enu=False,
        )

        decoded = decode_tracks_frame(frame)
        self.assertFalse(decoded["includeEnu"])
        self.assertFalse(decoded["radarPositionValid"])
        self.assertIsNone(decoded["tracks"][0]["enuM"])
        self.assertEqual(decoded["tracks"][0]["trackId"], 202)


class TestLdrsProcessingWorker(unittest.TestCase):
    def test_lla_ecef_enu_roundtrip(self):
        radar_lla = {"latitude": 41.8615, "longitude": -72.6124, "altitude": 50.0}
        target_lla = {"latitude": 41.8650, "longitude": -72.6100, "altitude": 150.0}

        target_ecef = lla_to_ecef(target_lla)
        radar_ecef = lla_to_ecef(radar_lla)

        delta = {
            "x": target_ecef["x"] - radar_ecef["x"],
            "y": target_ecef["y"] - radar_ecef["y"],
            "z": target_ecef["z"] - radar_ecef["z"],
        }
        enu = ecef_delta_to_enu(delta, radar_lla)
        recon_ecef = enu_to_ecef_position(enu, radar_lla)

        self.assertAlmostEqual(recon_ecef["x"], target_ecef["x"], places=3)
        self.assertAlmostEqual(recon_ecef["y"], target_ecef["y"], places=3)
        self.assertAlmostEqual(recon_ecef["z"], target_ecef["z"], places=3)

    def test_decode_status_message(self):
        # Create a synthetic STATUS packet (type 7, size 68)
        pkt = bytearray(68)
        struct.pack_into(">4I", pkt, 0, 1001, 7, 1, 68)
        struct.pack_into(">I", pkt, 16, 0x1234)  # software version
        struct.pack_into(">I", pkt, 24, 6)       # workingMode = 6 (STANDBY)
        struct.pack_into(">H", pkt, 28, 0x0000)  # statusFlags
        struct.pack_into(">I", pkt, 32, 0)       # bitStatus

        res = decode_status(bytes(pkt))
        self.assertEqual(res["workingMode"], 6)
        self.assertEqual(res["radarSoftwareRaw"], 0x1234)

    def test_process_message_tracks(self):
        # Build synthetic TRACKS packet
        # Header (48 bytes): counter=500, type=10, version=1, size=280
        # 1 track slot of 232 bytes (total 48 + 232 = 280)
        pkt = bytearray(280)
        struct.pack_into(">4I", pkt, 0, 500, 10, 1, 280)
        struct.pack_into(">Q", pkt, 16, 12345678)  # updateTimeTagUsec
        struct.pack_into(">H", pkt, 24, 1)         # chunkNumber
        struct.pack_into(">H", pkt, 26, 1)         # totalTracksInBurst
        struct.pack_into(">H", pkt, 30, 1)         # numTracksInMessage
        # Time tag
        struct.pack_into(">7H", pkt, 32, 2026, 9, 6, 14, 30, 0, 500)

        # Track 0 at offset 48:
        base = 48
        struct.pack_into(">I", pkt, base + 0, 999) # trackId
        lat_rad = to_rad(41.8615)
        lon_rad = to_rad(-72.6124)
        struct.pack_into(">d", pkt, base + 48, lat_rad) # lat
        struct.pack_into(">d", pkt, base + 56, lon_rad) # lon
        struct.pack_into(">f", pkt, base + 64, 120.0)   # alt
        struct.pack_into(">f", pkt, base + 68, 15.0)    # doppler
        struct.pack_into(">H", pkt, base + 120, (1 << 13)) # isValidTarget flag
        struct.pack_into(">3f", pkt, base + 156, 10.0, 5.0, 1.0) # nwu velocity

        radar_pos = {"latitude": 41.8600, "longitude": -72.6100, "altitude": 50.0}
        res = process_message(bytes(pkt), {"radarPosition": radar_pos})

        self.assertEqual(res["kind"], "tracks")
        self.assertEqual(res["count"], 1)
        self.assertIn("buffer", res)
        # Decode the binary buffer produced
        frame_decoded = decode_tracks_frame(res["buffer"])
        self.assertEqual(frame_decoded["count"], 1)
        self.assertEqual(frame_decoded["tracks"][0]["trackId"], 999)


class TestWorkerClient(unittest.TestCase):
    def test_worker_submission(self):
        worker = LdrsWorkerClient(max_pending=16)
        received_messages = []

        def on_msg(m):
            received_messages.append(m)

        worker.on("message", on_msg)

        # Send a synthetic status packet
        pkt = bytearray(68)
        struct.pack_into(">4I", pkt, 0, 1, 7, 1, 68)
        struct.pack_into(">I", pkt, 24, 3)  # SEARCH

        submitted = worker.submit(bytes(pkt), {})
        self.assertTrue(submitted)

        # Wait briefly for worker thread
        for _ in range(20):
            if received_messages:
                break
            time.sleep(0.05)

        self.assertEqual(len(received_messages), 1)
        self.assertEqual(received_messages[0]["kind"], "status")
        self.assertEqual(received_messages[0]["status"]["workingMode"], 3)
        worker.terminate()


class TestVectorNavService(unittest.TestCase):
    def test_nmea_parsing(self):
        vn = VectorNavService(port_path="dummy", baud_rate=115200)
        # Test GPGGA sentence: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        line = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        pos = vn.parse_position(line)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos["latitude"], 48.0 + 7.038 / 60.0, places=4)
        self.assertAlmostEqual(pos["longitude"], 11.0 + 31.000 / 60.0, places=4)
        self.assertAlmostEqual(pos["altitude"], 545.4, places=1)
        self.assertEqual(pos["quality"], 1)

    def test_vngps_parsing(self):
        vn = VectorNavService(port_path="dummy", baud_rate=115200)
        # $VNGPS,time,fix,lat,lon,alt,...
        line = "$VNGPS,12345.67,3,37.12345,-121.54321,120.5*22"
        pos = vn.parse_position(line)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos["latitude"], 37.12345, places=5)
        self.assertAlmostEqual(pos["longitude"], -121.54321, places=5)
        self.assertAlmostEqual(pos["altitude"], 120.5, places=1)


class TestVectorNavWsService(unittest.TestCase):
    def test_ws_message_handling(self):
        vn_ws = VectorNavWsService(url="dummy")
        msg = {
            "event": "airwarden.position",
            "payload": {
                "latitude": 42.123456,
                "longitude": -71.654321,
                "altitude": 85.0,
                "heading": 185.2,
                "pitch": 2.1,
                "roll": -0.5,
                "source": "remote_sim",
            },
        }
        vn_ws.handle_message(json.dumps(msg))
        pos = vn_ws.get_radar_position()
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos["latitude"], 42.123456, places=6)
        self.assertAlmostEqual(pos["longitude"], -71.654321, places=6)
        self.assertAlmostEqual(pos["altitude"], 85.0, places=1)
        self.assertAlmostEqual(pos["headingDeg"], 185.2, places=1)


if __name__ == "__main__":
    unittest.main()
