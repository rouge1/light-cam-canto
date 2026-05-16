"""Tests for webui.server CamPoller + non-blocking snapshot path.

No network, no threads started — exercises the poll cycle, circuit
breaker, cache write-through, and the request-side helpers directly.
"""
from unittest.mock import MagicMock, patch

import pytest

from webui import server as S


@pytest.fixture(autouse=True)
def _isolate_state():
    """Each test gets a clean cache + poller registry."""
    S._cache.clear()
    S._probe_cache.clear()
    saved = dict(S.POLLERS)
    S.POLLERS.clear()
    yield
    S.POLLERS.clear()
    S.POLLERS.update(saved)


def _client(ip="192.0.2.7"):
    c = MagicMock()
    c.ip = ip
    return c


_PAYLOAD = {
    "status": {"state": "listening", "mode": "rx"},
    "leds":   {"ir850": 0, "ir940": 1},
    "ae":     1,
    "proc":   {"irlink": 1, "daynightd": 0, "prudynt": 1},
}


# --- cache write-through ---------------------------------------------------

def test_poll_once_success_populates_all_demuxed_keys():
    c = _client()
    c.get_all_status.return_value = dict(_PAYLOAD)
    p = S.CamPoller("cam1", c)

    p._poll_once()

    assert p.reachable is True
    assert p.consecutive_failures == 0
    assert p.last_ok_monotonic is not None
    # /api/{cam}/all consumers + the per-key handlers all read these:
    allv = S._cam_snapshot("cam1:all")
    assert allv["status"] == _PAYLOAD["status"]
    assert allv["ae"] == {"value": 1}        # normalized once, in _commit_all
    assert S._cam_snapshot("cam1:status") == _PAYLOAD["status"]
    assert S._cam_snapshot("cam1:leds") == _PAYLOAD["leds"]
    assert S._cam_snapshot("cam1:ae") == 1
    assert S._cam_snapshot("cam1:proc") == _PAYLOAD["proc"]


def test_cam_snapshot_never_calls_the_client():
    c = _client()
    p = S.CamPoller("cam2", c)
    S.POLLERS["cam2"] = p
    # Cold: nothing cached yet.
    assert S._cam_snapshot("cam2:all") is None
    c.get_all_status.assert_not_called()
    c.get.assert_not_called()


# --- circuit breaker -------------------------------------------------------

def test_breaker_opens_after_trip_and_stops_fetching():
    c = _client()
    c.get_all_status.return_value = None  # cam down
    p = S.CamPoller("cam1", c)

    with patch.object(S, "_probe_alive", return_value=False):
        for _ in range(S.BREAKER_TRIP):
            p._poll_once()
        assert p._breaker_open() is True
        assert c.get_all_status.call_count == S.BREAKER_TRIP

        # Breaker open + probe still failing: cheap path only, no curl.
        p._poll_once()
        assert c.get_all_status.call_count == S.BREAKER_TRIP
        assert p.reachable is False


def test_breaker_half_open_recovers_on_success():
    c = _client()
    c.get_all_status.return_value = None
    p = S.CamPoller("cam1", c)

    with patch.object(S, "_probe_alive", return_value=False):
        for _ in range(S.BREAKER_TRIP):
            p._poll_once()
    assert p._breaker_open() is True

    # Probe comes back; the cam answers again → breaker closes.
    c.get_all_status.return_value = dict(_PAYLOAD)
    with patch.object(S, "_probe_alive", return_value=True):
        p._poll_once()

    assert p.consecutive_failures == 0
    assert p._breaker_open() is False
    assert p.reachable is True
    assert S._cam_snapshot("cam1:all")["status"] == _PAYLOAD["status"]


# --- adaptive interval -----------------------------------------------------

def test_compute_interval_idle_active_breaker():
    c = _client()
    p = S.CamPoller("cam1", c)

    # No demand → idle cadence.
    assert p._compute_interval() == S.POLL_IDLE_S

    # A client watching → fast cadence.
    p.note_demand()
    assert p._is_active() is True
    assert p._compute_interval() == S.POLL_ACTIVE_S

    # Breaker open trumps demand → throttled probe cadence.
    p.consecutive_failures = S.BREAKER_TRIP
    assert p._compute_interval() == S.POLL_BREAKER_S


# --- request-side gating ---------------------------------------------------

def test_cam_get_or_fetch_gated_short_circuits_when_breaker_open():
    c = _client()
    p = S.CamPoller("cam1", c)
    p.consecutive_failures = S.BREAKER_TRIP   # breaker open
    S.POLLERS["cam1"] = p

    fetch = MagicMock(return_value="should-not-run")
    out = S._cam_get_or_fetch_gated("cam1", "cam1:grid", 1.0, fetch)

    assert out is None              # no last-good cached → None
    fetch.assert_not_called()       # never issued a blocking curl


def test_cam_get_or_fetch_gated_returns_last_good_when_breaker_open():
    c = _client()
    p = S.CamPoller("cam1", c)
    p.consecutive_failures = S.BREAKER_TRIP
    S.POLLERS["cam1"] = p
    S._cache["cam1:grid"] = (S.time.monotonic(), "last-good")

    fetch = MagicMock()
    out = S._cam_get_or_fetch_gated("cam1", "cam1:grid", 1.0, fetch)

    assert out == "last-good"
    fetch.assert_not_called()


# --- SSE frame -------------------------------------------------------------

def test_build_stream_frame_shape_matches_all_endpoint():
    c1, c2 = _client("192.0.2.1"), _client("192.0.2.2")
    c1.get_all_status.return_value = dict(_PAYLOAD)
    p1, p2 = S.CamPoller("cam1", c1), S.CamPoller("cam2", c2)
    S.POLLERS["cam1"] = p1
    S.POLLERS["cam2"] = p2
    p1._poll_once()                 # cam1 has data; cam2 stays cold

    frame = S._build_stream_frame()

    assert set(frame) == {"_ts", "cam1", "cam2"}
    assert frame["cam1"]["status"] == _PAYLOAD["status"]
    assert frame["cam1"]["ae"] == {"value": 1}
    assert frame["cam1"]["_online"] is True
    assert frame["cam1"]["_stale"] is False
    # Cold cam → null-shaped default + stale, never missing keys.
    assert frame["cam2"]["status"] == {}
    assert frame["cam2"]["_stale"] is True
    assert frame["cam2"]["_age_s"] is None
