"""HTTP endpoint registration grouped by operational responsibility."""

from .camera_api import register_camera_api
from .control_api import register_control_api
from .route_api import register_route_api
from .status_api import register_status_api

__all__ = [
    "register_camera_api",
    "register_control_api",
    "register_route_api",
    "register_status_api",
]
