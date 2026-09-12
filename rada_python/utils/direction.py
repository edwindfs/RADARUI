"""Direction utility for converting bearing to cardinal direction."""

import math


def get_direction_from_bearing(bearing: float) -> str:
    """Convert bearing in degrees to 8-point cardinal direction string."""
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    # Match JavaScript Math.round(((bearing % 360) / 45)) % 8
    val = (bearing % 360.0) / 45.0
    idx = int(math.floor(val + 0.5)) % 8
    return dirs[idx]
