"""Tests for host/cam_status.py — parser correctness, no network."""
from unittest.mock import patch

from host.cam_status import CamStatusClient


def _make_grid_body(ts: int, blocks: list[int]) -> str:
    return " ".join(str(x) for x in [ts, *blocks])


def test_get_brightness_grid_happy_path():
    blocks = list(range(240))
    body = _make_grid_body(1234567890, blocks)
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=body):
        result = c.get_brightness_grid()
    assert result is not None
    ts, got = result
    assert ts == 1234567890
    assert got == blocks
    assert len(got) == 240


def test_get_brightness_grid_handles_extra_trailing_data():
    """CGI may emit a trailing newline or extra fields — ignore beyond 240."""
    body = _make_grid_body(99, [50] * 240) + " junk extra"
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=body):
        result = c.get_brightness_grid()
    assert result == (99, [50] * 240)


def test_get_brightness_grid_returns_none_on_short_payload():
    body = "1 2 3"
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=body):
        assert c.get_brightness_grid() is None


def test_get_brightness_grid_returns_none_on_error_marker():
    """The CGI returns 'error: no grid file' when prudynt-patched isn't running."""
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="error: no grid file"):
        assert c.get_brightness_grid() is None


def test_get_brightness_grid_returns_none_on_non_int():
    body = " ".join(["abc"] + [str(x) for x in range(240)])
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=body):
        assert c.get_brightness_grid() is None


def test_get_brightness_grid_returns_none_when_fetch_fails():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=None):
        assert c.get_brightness_grid() is None


def test_get_returns_none_on_invalid_json():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="not json"):
        assert c.get() is None


def test_get_parses_status_json():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value='{"state":"connected","rate_ms":160}'):
        s = c.get()
    assert s == {"state": "connected", "rate_ms": 160}


def test_get_ae_freeze_zero():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="0\n"):
        assert c.get_ae_freeze() == 0


def test_get_ae_freeze_one():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="1\n"):
        assert c.get_ae_freeze() == 1


def test_get_ae_freeze_handles_no_trailing_newline():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="1"):
        assert c.get_ae_freeze() == 1


def test_get_ae_freeze_returns_none_on_error_marker():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="error: no ae_freeze file"):
        assert c.get_ae_freeze() is None


def test_get_ae_freeze_returns_none_on_garbage():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="hello"):
        assert c.get_ae_freeze() is None


def test_get_ae_freeze_returns_none_on_out_of_range():
    """ae_freeze is binary; reject anything not in {0, 1} as corrupt."""
    c = CamStatusClient("192.0.2.1")
    for v in ("2", "-1", "100"):
        with patch.object(c, "_fetch", return_value=v):
            assert c.get_ae_freeze() is None, f"value {v!r} should map to None"


def test_get_ae_freeze_returns_none_when_fetch_fails():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=None):
        assert c.get_ae_freeze() is None


def test_set_ae_freeze_true_returns_true_on_200():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(200, '{"ok":true,"value":1}')) as p:
        assert c.set_ae_freeze(True) is True
    p.assert_called_once_with("/x/ae-freeze.cgi", "value=1")


def test_set_ae_freeze_false_sends_value_zero():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(200, '{"ok":true,"value":0}')) as p:
        assert c.set_ae_freeze(False) is True
    p.assert_called_once_with("/x/ae-freeze.cgi", "value=0")


def test_set_ae_freeze_returns_false_on_400():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(400, '{"ok":false,"error":"bad"}')):
        assert c.set_ae_freeze(True) is False


def test_set_ae_freeze_returns_false_on_500():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(500, '{"ok":false}')):
        assert c.set_ae_freeze(True) is False


def test_set_ae_freeze_returns_false_on_transport_failure():
    """_post returns None on timeout/network — set_ae_freeze must fail loud."""
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=None):
        assert c.set_ae_freeze(True) is False


def test_set_ae_freeze_requires_ok_true_in_body():
    """200 OK with the wrong body is still a failure — guards against
    a CGI that returns 200 from a misconfigured upstream."""
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(200, "ok")):
        assert c.set_ae_freeze(True) is False


def test_get_proc_status_happy_path():
    c = CamStatusClient("192.0.2.1")
    body = '{"irlink":1,"daynightd":0,"prudynt":1,"ts_s":12345}'
    with patch.object(c, "_fetch", return_value=body):
        s = c.get_proc_status()
    assert s == {"irlink": 1, "daynightd": 0, "prudynt": 1, "ts_s": 12345}


def test_get_proc_status_returns_none_when_fetch_fails():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value=None):
        assert c.get_proc_status() is None


def test_get_proc_status_returns_none_on_invalid_json():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_fetch", return_value="not json"):
        assert c.get_proc_status() is None


def test_get_proc_status_rejects_missing_field():
    c = CamStatusClient("192.0.2.1")
    body = '{"irlink":1,"prudynt":1}'  # daynightd missing
    with patch.object(c, "_fetch", return_value=body):
        assert c.get_proc_status() is None


def test_get_proc_status_rejects_non_int_field():
    c = CamStatusClient("192.0.2.1")
    body = '{"irlink":"oops","daynightd":0,"prudynt":1}'
    with patch.object(c, "_fetch", return_value=body):
        assert c.get_proc_status() is None


def test_get_proc_status_rejects_negative():
    c = CamStatusClient("192.0.2.1")
    body = '{"irlink":-1,"daynightd":0,"prudynt":1}'
    with patch.object(c, "_fetch", return_value=body):
        assert c.get_proc_status() is None


def test_imp_cmd_int_value():
    c = CamStatusClient("192.0.2.1")
    body = '{"code":200,"result":"success","message":""}'
    with patch.object(c, "_post", return_value=(200, body)) as p:
        assert c.imp_cmd("ir850", 1) is True
    p.assert_called_once_with("/x/json-imp.cgi",
                              '{"cmd": "ir850", "val": 1}',
                              content_type="application/json")


def test_imp_cmd_string_value():
    c = CamStatusClient("192.0.2.1")
    body = '{"code":200,"result":"success","message":""}'
    with patch.object(c, "_post", return_value=(200, body)) as p:
        assert c.imp_cmd("daynight", "night") is True
    p.assert_called_once_with("/x/json-imp.cgi",
                              '{"cmd": "daynight", "val": "night"}',
                              content_type="application/json")


def test_imp_cmd_returns_false_on_400():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(400, '{"result":"error"}')):
        assert c.imp_cmd("ir850", 1) is False


def test_imp_cmd_returns_false_on_transport_failure():
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=None):
        assert c.imp_cmd("ir850", 1) is False


def test_imp_cmd_requires_success_substring():
    """200 OK without `"success"` substring is failure (mirrors the
    cam_setup.sh / cal_procedure.py check)."""
    c = CamStatusClient("192.0.2.1")
    with patch.object(c, "_post", return_value=(200, '{"result":"error","message":"bad"}')):
        assert c.imp_cmd("ir850", 1) is False
