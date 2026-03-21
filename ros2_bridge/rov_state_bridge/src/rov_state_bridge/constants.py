"""Stage1 bridge constants.

The bridge mirrors current shared-memory truth sources into outer-plane topics.
These names intentionally mirror the existing contracts and are not a new source
of runtime authority.
"""

TELEMETRY_SHM_NAME = "/rovctrl_telemetry_v2"
NAV_VIEW_SHM_NAME = "/rovctrl_nav_view_v1"
NAV_STATE_SHM_NAME = "/rov_nav_state_v1"

TELEMETRY_TOPIC = "/rov/telemetry"
NAV_VIEW_TOPIC = "/rov/nav_view"
NAV_STATE_TOPIC = "/rov/nav_state_raw"
HEALTH_TOPIC = "/rov/health"

DEFAULT_POLL_HZ = 10.0
DEFAULT_MAX_SEQLOCK_RETRIES = 8
