# RADA Radar Python Client (`rada_python`)

Direct 1:1 translation of the RADA LDRS radar software from Node.js/JavaScript into Python. All packet offsets, geodetic math formulas, binary frame layouts, and control commands are strictly preserved.

---

## File Mapping

| Original JS File | Converted Python Module | Description |
| :--- | :--- | :--- |
| `v1.ldrClient.js` | [`v1_ldr_client.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/v1_ldr_client.py) | Modern client: TCP framing, VectorNav state, auto-search, binary WebSocket fanout. |
| `ldrsClient.js` | [`ldrs_client.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/ldrs_client.py) | Standalone client: TCP radar connection, VectorNav serial NMEA hook, JSON detections. |
| `ws/ldrsBinaryProtocol.js` | [`ws/ldrs_binary_protocol.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/ws/ldrs_binary_protocol.py) | `LDR1` binary frame packer and unpacker (LE header + track records + variable strings). |
| `workers/ldrsProcessingWorker.js` | [`workers/ldrs_processing_worker.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/workers/ldrs_processing_worker.py) | WGS-84 / ECEF / ENU / NWU coordinate conversions, track decoding, ICD message handlers. |
| `workers/ldrsWorkerClient.js` | [`workers/ldrs_worker_client.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/workers/ldrs_worker_client.py) | Asynchronous worker queue and thread to avoid blocking TCP socket threads. |
| `formatters/buildLdrsDetection.js` | [`formatters/build_ldrs_detection.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/formatters/build_ldrs_detection.py) | Builds standard detection dictionaries for operator interfaces. |
| `services/vectorNav.service.js` | [`services/vector_nav_service.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/services/vector_nav_service.py) | Serial port communication for VectorNav NMEA/INS sentences. |
| `services/vectorNavWs.service.js` | [`services/vector_nav_ws_service.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/services/vector_nav_ws_service.py) | WebSocket client for VectorNav JSON telemetry feeds. |
| `utils/geo.js` | [`utils/geo.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/utils/geo.py) | Haversine distance, initial bearing, slant distance. |
| `utils/direction.js` | [`utils/direction.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/utils/direction.py) | 8-point compass cardinal direction calculation. |
| `scripts/ldrsTrackSimulator.js` | [`scripts/ldrs_track_simulator.py`](file:///c:/Users/david/OneDrive/Documentos/RADA%20JAVA/rada_python/scripts/ldrs_track_simulator.py) | Dynamic track simulator broadcasting `LDR1` binary frames. |

---

## How to Run

### 1. Run the Main Client
Equivalent to `npm start`:
```bash
python -m rada_python.v1_ldr_client
```
or
```bash
python rada_python/main.py
```

### 2. Run the Standalone Client
```bash
python -m rada_python.ldrs_client
```

### 3. Run the Track Simulator
Equivalent to `npm run simulate`:
```bash
python -m rada_python.scripts.ldrs_track_simulator
```

### 4. Run the Verification Tests
```bash
python -m unittest rada_python.tests.test_translation
```
