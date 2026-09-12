"""RADA Radar Python Client and Protocol Suite."""

from .formatters.build_ldrs_detection import build_ldrs_detection
from .services.vector_nav_service import VectorNavService
from .services.vector_nav_ws_service import VectorNavWsService
from .utils.direction import get_direction_from_bearing
from .utils.geo import bearing_degrees, distance_from_radar, haversine_meters
from .workers.ldrs_processing_worker import process_message
from .workers.ldrs_worker_client import LdrsWorkerClient
from .ws.ldrs_binary_protocol import create_tracks_frame, decode_tracks_frame

__version__ = "1.0.0"

__all__ = [
    "build_ldrs_detection",
    "VectorNavService",
    "VectorNavWsService",
    "get_direction_from_bearing",
    "bearing_degrees",
    "distance_from_radar",
    "haversine_meters",
    "process_message",
    "LdrsWorkerClient",
    "create_tracks_frame",
    "decode_tracks_frame",
]
