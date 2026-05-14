"""Thin proxy + 2 fire-and-forget pushes for the LiWiFi webui.

The server's only job is to forward per-cam HTTP reads through
host.cam_status.CamStatusClient (cookie-cached, fail-soft) and expose two
push endpoints:

  POST /api/cal/start    detached `python -m host.cal_procedure --no-interactive`
  POST /api/cal/monocal  one POST to cam2's /x/monocal-trigger.cgi

No long-running worker threads; no SSH calls in any request handler. The
frontend reads each cam's state independently via /api/cam{1,2}/* and shows
two per-camera columns of state-machine transitions. cal_procedure is
self-contained for its own cam1 daemon kill/restart, so the webui can fire
and forget.

Endpoints:
  GET  /api/{cam}/grid          → CamStatusClient.get_brightness_grid()
  GET  /api/{cam}/ae            → CamStatusClient.get_ae_freeze()
  POST /api/{cam}/ae            body `value=0|1`
  GET  /api/{cam}/proc          → CamStatusClient.get_proc_status()
  POST /api/{cam}/led           body `lamp=ir850|ir940&value=0|1`
  GET  /api/{cam}/status        → CamStatusClient.get()  (irlink-status.json)
  GET  /api/{cam}/leds          → CamStatusClient.get_led_state()
  GET  /api/{cam}/snapshot      jpeg passthrough (?ch=0|1)
  GET  /api/{cam}/cal-info      status + saved_cal + snapshot_url merged
  GET  /api/{cam}/grid-deltas   → CamStatusClient.get_grid_deltas()
  GET  /api/{cam}/events        → /tmp/irlink-events.log (?tail=N)
  POST /api/{cam}/setup         → CamStatusClient.run_setup()
  POST /api/{cam}/time-sync     → CamStatusClient.set_time()

  POST /api/cal/start           detached cal_procedure subprocess
  GET  /api/cal/status          {running, pid, started_at} from PID file
  POST /api/cal/monocal         one-shot cam2 monocal-trigger
  GET  /api/cal/monocal-status  cam2 monocal-status + cam1 cal-status merged

cam ∈ {cam1, cam2}; resolved to dacam1/dacam2 then to IPs at startup.

Usage:
  conda activate light
  python -m webui.server                 # default port 8765
  python -m webui.server --port 9000
"""
from __future__ import annotations

import argparse
import io
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from host.cam_status import CamStatusClient, resolve_ip  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBUI_DIR = os.path.dirname(os.path.abspath(__file__))

CAL_PID_FILE = "/tmp/liwifi-cal.pid"


# ---------------------------------------------------------------------------
# Coalescing cache for slow per-cam endpoints. uhttpd on the cams is a
# single-process server (`-t 0`) — every CGI hit forks+execs a shell on
# slow MIPS hardware (~2s per call). When several browser tabs poll the
# same endpoint simultaneously, requests serialize at uhttpd and pile up.
# A short-TTL coalescing cache here means concurrent identical requests
# share a single in-flight fetch, and back-to-back polls within the TTL
# return the cached response instantly without touching the cam.
# Per-key lock prevents thundering-herd refresh from many tabs at once.
# ---------------------------------------------------------------------------

_CACHE_TTL = {              # seconds — tuned per-endpoint freshness needs
    "status":   2.0,        # was 1.0; bumped to halve cam1's CGI hit rate.
                            # State transitions still get caught on next poll;
                            # the human-perceptible latency is fine at 2s.
    "ae":       2.0,
    "proc":     5.0,
    "leds":     2.0,        # was 1.0; same rationale — LED state changes
                            # are observed on next tick which is fine.
    "grid":     0.4,
    "snapshot": 1.0,
    "cal-info": 2.0,
    "events":   2.0,
    "monocal-status": 1.0,
    "all":      2.0,        # single consolidated read; same freshness as status
    "health":   10.0,       # uptime/load/mem/disk drift slowly; 10s is plenty
                            # and cheap on cam1's single-process uhttpd.
}

_cache: dict = {}
_cache_locks: dict = {}
_cache_lock = threading.Lock()


def _cache_get_or_fetch(key: str, ttl: float, fetch_fn):
    now = time.monotonic()
    cached = _cache.get(key)
    if cached and now - cached[0] < ttl:
        return cached[1]
    with _cache_lock:
        lock = _cache_locks.setdefault(key, threading.Lock())
    with lock:
        cached = _cache.get(key)
        if cached and time.monotonic() - cached[0] < ttl:
            return cached[1]
        value = fetch_fn()
        _cache[key] = (time.monotonic(), value)
        return value


def _cache_invalidate(*keys: str):
    """Drop cached entries so the next read fetches fresh from the cam.
    Call after any POST that mutates state the cam exposes via a cached GET."""
    for k in keys:
        _cache.pop(k, None)


def _cache_peek(key: str, max_age: float | None = None):
    """Return the cached value if present (no lock, no fetch). If max_age
    is given, returns None when the entry is older. Designed for piggyback
    reads — e.g. /api/{cam}/health can serve vitals from the live page
    poller's `:all` cache without triggering its own CGI hit on the cam."""
    cached = _cache.get(key)
    if not cached:
        return None
    if max_age is not None and time.monotonic() - cached[0] > max_age:
        return None
    return cached[1]


# ---------------------------------------------------------------------------
# Per-cam alive probe (TCP connect to port 80, 0.5s budget, 3s cached).
# When a cam is rebooting or unplugged, every per-cam endpoint would otherwise
# fire its own 3s curl in parallel. Chrome's 6-slot per-origin connection
# limit means the page can lock up. Probe fast-fails downstream curls so the
# connection slots churn quickly and the LEDs poll keeps the UI responsive.
# ---------------------------------------------------------------------------

_PROBE_TTL = 3.0
_PROBE_CONNECT_S = 2.0   # bumped 1.0 → 2.0 after observing cam1 TCP handshake
                          # spikes to ~1.6s when its uhttpd is mid-CGI-fork.
                          # Without it, the probe false-negative caches cam1
                          # as "dead" for 3s, the chips flip to OFFLINE, and
                          # the user sees a 3s-on/3s-off flicker on the page.
                          # 2.0s catches the spikes while still fast-failing
                          # a genuinely-unplugged cam.
_probe_cache: dict = {}
_probe_lock = threading.Lock()


def _probe_alive(ip: str) -> bool:
    now = time.monotonic()
    cached = _probe_cache.get(ip)
    if cached and now - cached[0] < _PROBE_TTL:
        return cached[1]
    with _probe_lock:
        cached = _probe_cache.get(ip)
        if cached and time.monotonic() - cached[0] < _PROBE_TTL:
            return cached[1]
        alive = False
        s = None
        try:
            s = socket.create_connection((ip, 80), timeout=_PROBE_CONNECT_S)
            alive = True
        except (OSError, socket.timeout):
            alive = False
        finally:
            if s is not None:
                try: s.close()
                except OSError: pass
        _probe_cache[ip] = (time.monotonic(), alive)
        return alive


def _cam_get_or_fetch(cam: str, key: str, ttl: float, fetch_fn):
    """Like _cache_get_or_fetch, but short-circuits to None when the cam is
    known unreachable. Falls back to last-good cached value if any."""
    client = CLIENTS.get(cam)
    if client is None or not _probe_alive(client.ip):
        cached = _cache.get(key)
        return cached[1] if cached else None
    return _cache_get_or_fetch(key, ttl, fetch_fn)


CALIBRATION_JSON_PATH = os.path.join(REPO_ROOT, "host", "calibration.json")


def _read_cam2_sees_cam1_pixel() -> tuple[int, int] | None:
    """Resolve cam2's pixel for cam1's TX (cam2's --pixel arg in monocal).
    Prefer cam2's live /opt/etc/calibration.json (every successful cal flow
    writes it). Fall back to laptop's host/calibration.json if cam2 is
    unreachable or has never been cal'd."""
    cam2 = CLIENTS.get("cam2")
    if cam2 is not None:
        live = cam2.get_saved_cal()
        if live is not None:
            pixel = live.get("tx_pixel")
            if isinstance(pixel, list) and len(pixel) == 2:
                try:
                    return int(pixel[0]), int(pixel[1])
                except (TypeError, ValueError):
                    pass
    try:
        with open(CALIBRATION_JSON_PATH) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None
    section = data.get("cam2_sees_cam1") or {}
    pixel = section.get("tx_pixel")
    if not (isinstance(pixel, list) and len(pixel) == 2):
        return None
    try:
        return int(pixel[0]), int(pixel[1])
    except (TypeError, ValueError):
        return None


CAM_HOSTS = {"cam1": "dacam1", "cam2": "dacam2"}
CLIENTS: dict[str, CamStatusClient] = {}
SERVER_STARTED_AT = time.time()

ANNOTATED_CACHE_DIR = "/tmp/liwifi-annotated"

# Cam snapshot CGI sometimes serves a JPEG that's missing the trailing
# EOI marker (race between prudynt's ongoing write and uhttpd's read).
# Without this flag, PIL raises OSError("image file is truncated") and
# both _annotate_snapshot and _tiny_wireframe fail. With it, PIL keeps
# the bytes it has and renders normally — visually indistinguishable
# from a full JPEG for our purposes (we re-encode anyway).
try:
    from PIL import ImageFile as _PILImageFile
    _PILImageFile.LOAD_TRUNCATED_IMAGES = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Annotated snapshot rendering — composite the saved cal pixel as a bullseye
# crosshair + coordinate label + info bar onto the cam's substream JPEG. The
# substream is 640×360 which matches calibration.json frame_size, so cal
# pixel coords map 1:1 onto the image. Returns a single JPEG curl-able at
# /api/{cam}/snapshot-annotated, also persisted to /tmp/liwifi-annotated/
# so it can be opened/saved/shared as a static artifact.
#
# All annotation work happens on the laptop (PIL is in the `light` conda
# env). The cams have no PIL/ImageMagick/GD — pushing the work cam-side
# isn't practical and would require cross-compiling something like libgd
# into the firmware.
# ---------------------------------------------------------------------------

_FONT_CACHE: dict = {}


def _resolve_cal_delta(saved_cal: dict | None, status: dict | None):
    """Best-available grid_delta for the bottom info bar of an annotated
    snapshot. Mirrors the precedence used to pick `sp` (saved_cal first,
    then runtime status). Returns int or None."""
    if isinstance(saved_cal, dict) and isinstance(saved_cal.get("grid_delta"), int):
        return saved_cal["grid_delta"]
    if isinstance(status, dict):
        cal = status.get("cal")
        if isinstance(cal, dict) and isinstance(cal.get("peak_delta"), int):
            return cal["peak_delta"]
    return None


def _get_font(size: int):
    """Try to load a small monospace TTF for the info bar. Falls back to
    PIL's bitmap default if no system font is available — the default is
    fine but tiny, hence the TTF preference."""
    from PIL import ImageFont
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    ]
    for path in candidates:
        try:
            font = ImageFont.truetype(path, size)
            _FONT_CACHE[size] = font
            return font
        except (OSError, IOError):
            continue
    font = ImageFont.load_default()
    _FONT_CACHE[size] = font
    return font


TINY_W = 64                # 16:9-ish; recognizable as a room shape
TINY_H = 36                #
# Wire format is now bare raster — the dimensions (TINY_W × TINY_H) and
# format (1-bit, MSB-first, row-major) are agreed out-of-band by both
# sides, so we don't burn 4 bytes per packet on magic+ver+w+h. The
# decoder synthesizes the header from constants. If dimensions ever
# change, bump both sides in lockstep (or move the version into the
# enclosing protocol envelope, not the payload).
TINY_RASTER_BYTES = (TINY_W * TINY_H + 7) // 8     # = 288 for 64×36
TINY_TOTAL_BYTES = TINY_RASTER_BYTES   # = 288 (was 292 with header)


def _tiny_wireframe(jpeg_bytes: bytes,
                    cam_label: str,
                    saved_cal: dict | None = None,
                    status: dict | None = None) -> tuple[bytes, bytes]:
    """Generate a tiny 1-bit wireframe of the cam's substream JPEG.

    Pipeline: grayscale → Gaussian blur (denoise JPEG block artifacts)
    → FIND_EDGES (Sobel) → downscale to TINY_W × TINY_H → threshold to
    1-bit → MSB-first pack 8 px/byte. Output bytes target a future
    over-IR transmission (~292 bytes uncompressed, fragments to ~25
    chunks at 12 data bytes/frame, ~5 min wall at 160ms/sym).

    Returns (raw_bytes, render_png) where:
      - raw_bytes is the wire format: 4-byte header + packed raster.
        Intentionally NO bullseye in the bytes — the receiver overlays
        it locally from the cal coords (which already round-trip via
        CAL_DONE's compressed 6-byte payload). Keeps the wire payload
        scene-pure for the would-be IR transmit path.
      - render_png is the same raster upscaled 8× nearest-neighbor
        with the same affordances as _annotate_snapshot baked on top:
        top-left cyan "CAMx SEES CAMy" label, red+yellow bullseye at
        the cal pixel with `[x, y]` coord pill, bottom-yellow info bar
        showing `pixel:` and `cal'd:` wall-clock. Mirrors the hi-res
        layout exactly so the lo-res reads as the same artifact, just
        at a lower fidelity.
    """
    from PIL import Image, ImageDraw, ImageFilter

    src = Image.open(io.BytesIO(jpeg_bytes)).convert("L")
    # Downscale FIRST, then edge-detect on the small image. Doing
    # edges-on-full-res then averaging-down via LANCZOS dilutes edges
    # below threshold; doing it the other way means each edge pixel
    # in the 64×36 grid is a real (downsampled) gradient and survives.
    # Pre-blur radius 2.0 (was 1.0) smears out fine JPEG/sensor texture
    # so the edge filter sees only meaningful geometry — drops ~10% of
    # speckle on-pixels without losing room outlines.
    blurred = src.filter(ImageFilter.GaussianBlur(radius=2.0))
    small = blurred.resize((TINY_W, TINY_H), Image.LANCZOS)
    # Detect edges on the tiny image — far stronger relative response
    # than the full-frame approach.
    edges = small.filter(ImageFilter.FIND_EDGES)
    # Threshold 50 (was 18) keeps only strong gradients (door frames,
    # window outlines, furniture silhouettes); cuts on-pixel density
    # ~31% → ~21% so the image reads as a wireframe scene instead of
    # a noisy gradient map.
    bw = edges.point(lambda p: 255 if p > 50 else 0, mode="L")

    # OSD mask — channel-1 substream carries the per-second timestamp
    # in the top ~30 px and a `thingino` watermark in the bottom ~30 px
    # of the source frame. After downscale to TINY_H=36 those bands
    # collapse to rows 0..2 (top) and 33..35 (bottom). Zero them so
    # the wireframe shows the actual scene, not OSD jitter.
    md = ImageDraw.Draw(bw)
    md.rectangle([0, 0, TINY_W - 1, 2], fill=0)
    md.rectangle([0, TINY_H - 3, TINY_W - 1, TINY_H - 1], fill=0)

    # Pack 8 px per byte, MSB-first, row-major. PIL's `convert("1")`
    # produces a 1-bit image; tobytes() gives the right packed bytes.
    # No header — the dimensions live in the wire-format contract, not
    # in the bytes (saves 4 B per packet, ~1.5% at 288 B raster).
    one_bit = bw.convert("1")
    raster_bytes = one_bit.tobytes()
    raw_bytes = raster_bytes

    # Render PNG: upscale 8× nearest-neighbor (chunky pixels = the
    # "intentional" lo-res look). Dark background, off-white "ink".
    UPSCALE = 8
    out_w, out_h = TINY_W * UPSCALE, TINY_H * UPSCALE
    big = one_bit.convert("L").resize((out_w, out_h), Image.NEAREST)
    bg = Image.new("RGB", (out_w, out_h), (8, 12, 20))
    fg = Image.new("RGB", (out_w, out_h), (220, 230, 240))
    out = Image.composite(fg, bg, big)

    # === Affordances baked into the PNG only (NOT the wire bytes) ===
    # The same bullseye + labels + info bar as the hi-res annotated.
    # Mirrors the hi-res palette: red crosshair + yellow ring + yellow
    # label, cyan top-left badge, yellow bottom info bar.
    if saved_cal and isinstance(saved_cal.get("tx_pixel"), list) \
            and len(saved_cal["tx_pixel"]) == 2:
        try:
            src_x = int(saved_cal["tx_pixel"][0])
            src_y = int(saved_cal["tx_pixel"][1])
        except (TypeError, ValueError):
            src_x = src_y = None
        if src_x is not None and 0 <= src_x < 640 and 0 <= src_y < 360:
            # Source frame is 640×360; upscaled output preserves 16:9
            # so the same scale factor applies in both axes.
            sx = out_w / 640.0
            sy = out_h / 360.0
            px = int(round(src_x * sx))
            py = int(round(src_y * sy))
            draw = ImageDraw.Draw(out, "RGBA")
            red = (255, 48, 48, 255)
            yellow = (255, 210, 74, 255)
            arm = 18
            gap = 5
            draw.line([(px - arm, py), (px - gap, py)], fill=red, width=2)
            draw.line([(px + gap, py), (px + arm, py)], fill=red, width=2)
            draw.line([(px, py - arm), (px, py - gap)], fill=red, width=2)
            draw.line([(px, py + gap), (px, py + arm)], fill=red, width=2)
            draw.ellipse([px - 7, py - 7, px + 7, py + 7],
                         outline=yellow, width=2)
            draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=yellow)
            label = f"[{src_x}, {src_y}]"
            font = _get_font(13)
            try:
                tbox = draw.textbbox((0, 0), label, font=font)
                tw, th = tbox[2] - tbox[0], tbox[3] - tbox[1]
            except AttributeError:
                tw, th = font.getsize(label)
            ly = py - 22 - th if py - 22 - th >= 0 else py + 14
            lx = px + 12 if px + 12 + tw + 6 <= out_w else px - 12 - tw - 6
            draw.rectangle([lx - 3, ly - 2, lx + tw + 3, ly + th + 2],
                           fill=(0, 0, 0, 230))
            draw.text((lx, ly), label, fill=yellow, font=font)

    draw2 = ImageDraw.Draw(out, "RGBA")
    if cam_label:
        font_top = _get_font(13)
        try:
            tbox = draw2.textbbox((0, 0), cam_label, font=font_top)
            tw, th = tbox[2] - tbox[0], tbox[3] - tbox[1]
        except AttributeError:
            tw, th = font_top.getsize(cam_label)
        draw2.rectangle([0, 0, tw + 12, th + 8], fill=(0, 0, 0, 220))
        draw2.text((6, 4), cam_label, fill=(108, 242, 255, 255), font=font_top)

    # Bottom info bar — semi-transparent black, yellow text.
    info_h = 44
    bar = Image.new("RGBA", (out_w, info_h), (0, 0, 0, 200))
    out.paste(bar, (0, out_h - info_h), bar)
    draw3 = ImageDraw.Draw(out, "RGBA")

    if saved_cal:
        sp = saved_cal.get("tx_pixel") or [-1, -1]
    else:
        sp = [-1, -1]

    cal_time_str = "—"
    if status and status.get("ts_ms") and status.get("last_event_ms"):
        age_ms = status["ts_ms"] - status["last_event_ms"]
        if age_ms >= 0:
            event_epoch = time.time() - age_ms / 1000.0
            cal_time_str = time.strftime("%Y-%m-%d  %H:%M:%S",
                                          time.localtime(event_epoch))

    yellow = (255, 210, 74, 255)
    pixel_str = f"pixel: [{sp[0]}, {sp[1]}]"
    cal_str   = f"cal'd: {cal_time_str}"
    delta = _resolve_cal_delta(saved_cal, status)
    delta_str = f"Δ{delta}" if delta is not None else ""
    font = _get_font(15)
    y0 = out_h - info_h + 6
    line_h = 18
    draw3.text((8, y0),            pixel_str, fill=yellow, font=font)
    draw3.text((8, y0 + line_h),   cal_str,   fill=yellow, font=font)
    if delta_str:
        try:
            tb = draw3.textbbox((0, 0), pixel_str, font=font)
            dx = 8 + (tb[2] - tb[0]) + 14
        except AttributeError:
            dx = 8 + 9 * len(pixel_str) + 14
        draw3.text((dx, y0), delta_str, fill=yellow, font=font)

    out_buf = io.BytesIO()
    out.save(out_buf, format="PNG", optimize=True)
    png_bytes = out_buf.getvalue()

    # Persist artifacts for inspection / static fetch.
    try:
        os.makedirs(ANNOTATED_CACHE_DIR, exist_ok=True)
        cam = cam_label.split()[0].lower()
        with open(os.path.join(ANNOTATED_CACHE_DIR, f"{cam}-tiny.bin"), "wb") as f:
            f.write(raw_bytes)
        with open(os.path.join(ANNOTATED_CACHE_DIR, f"{cam}-tiny.png"), "wb") as f:
            f.write(png_bytes)
    except OSError:
        pass

    return raw_bytes, png_bytes


def _faxify_snapshot(jpeg_bytes: bytes,
                     status: dict | None,
                     saved_cal: dict | None,
                     cam_label: str) -> bytes:
    """Fax/wireframe rendering of the cal snapshot.

    Concept: a tiny, high-contrast representation of "what cam{N} sees"
    in the spirit of how the cal artifact would look if it were small
    enough to send over the IR link itself (the link is ~3-5 bps text
    today, so this is an aesthetic exercise — but it telegraphs the
    intent: fax-machine, scanner-line, edge-traced).

    Pipeline:
      1. Grayscale + slight Gaussian blur (denoise JPEG artifacts so
         edge detection doesn't latch onto compression blocks).
      2. PIL FIND_EDGES filter (Sobel) → bright edges on black.
      3. Hard threshold to pure 1-bit (the "fax" step).
      4. Compose onto dark-bluish background with off-white edges.
      5. 50%-alpha scanline overlay every 2 px for CRT/scanner feel.
      6. Bullseye + coord label + bottom info bar in stark white.

    Output is PNG (sharp 1-bit-ish edges), saved alongside the regular
    annotated to /tmp/liwifi-annotated/{cam}-fax.png."""
    from PIL import Image, ImageDraw, ImageFilter

    src = Image.open(io.BytesIO(jpeg_bytes)).convert("L")
    w, h = src.size

    blurred = src.filter(ImageFilter.GaussianBlur(radius=1.2))
    edges = blurred.filter(ImageFilter.FIND_EDGES)

    # Hard threshold — anything above is "edge ink", below is background.
    # 30 is empirically right for the substream's brightness range; lower
    # adds JPEG-block noise, higher loses fine detail (door frames, etc).
    bw = edges.point(lambda p: 255 if p > 30 else 0, mode="L")

    bg_color = (8, 12, 20)
    ink_color = (220, 230, 240)
    bg = Image.new("RGB", (w, h), bg_color)
    fg = Image.new("RGB", (w, h), ink_color)
    out = Image.composite(fg, bg, bw)

    # Scanlines: 2-px periodic dark bands, 24% alpha. Adds the "this came
    # off a scanner" texture without hiding the edge outlines.
    scan = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    sd = ImageDraw.Draw(scan)
    for y in range(0, h, 2):
        sd.line([(0, y), (w, y)], fill=(0, 0, 0, 60))
    out = Image.alpha_composite(out.convert("RGBA"), scan).convert("RGB")

    draw = ImageDraw.Draw(out, "RGBA")

    # Bullseye in stark white — the cal pixel is the whole point of the
    # artifact. Using white not yellow because the palette here is B&W
    # (yellow on the regular annotated reads as a colored UI element;
    # here we want it to look like a pen mark on a fax).
    px = py = None
    if saved_cal and isinstance(saved_cal.get("tx_pixel"), list) \
            and len(saved_cal["tx_pixel"]) == 2:
        px, py = int(saved_cal["tx_pixel"][0]), int(saved_cal["tx_pixel"][1])
    elif status and isinstance(status.get("pixel"), list) \
            and len(status["pixel"]) == 2:
        px, py = int(status["pixel"][0]), int(status["pixel"][1])

    if px is not None and 0 <= px < w and 0 <= py < h:
        white = (255, 255, 255, 255)
        arm = 24
        draw.line([(px - arm, py), (px - 4, py)], fill=white, width=2)
        draw.line([(px + 4, py), (px + arm, py)], fill=white, width=2)
        draw.line([(px, py - arm), (px, py - 4)], fill=white, width=2)
        draw.line([(px, py + 4), (px, py + arm)], fill=white, width=2)
        draw.ellipse([px - 8, py - 8, px + 8, py + 8], outline=white, width=2)
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=white)

        label = f"[{px}, {py}]"
        font = _get_font(13)
        try:
            tbox = draw.textbbox((0, 0), label, font=font)
            tw, th = tbox[2] - tbox[0], tbox[3] - tbox[1]
        except AttributeError:
            tw, th = font.getsize(label)
        ly = py - 22 - th if py - 22 - th >= 0 else py + 12
        lx = px + 12 if px + 12 + tw + 6 <= w else px - 12 - tw - 6
        draw.rectangle([lx - 3, ly - 2, lx + tw + 3, ly + th + 2],
                       fill=(0, 0, 0, 230))
        draw.text((lx, ly), label, fill=white, font=font)

    # Bottom info bar (mirrors annotated for consistency).
    info_h = 44
    bar = Image.new("RGBA", (w, info_h), (0, 0, 0, 220))
    out.paste(bar, (0, h - info_h), bar)
    draw2 = ImageDraw.Draw(out, "RGBA")

    if saved_cal:
        sp = saved_cal.get("tx_pixel") or [-1, -1]
    else:
        sp = [-1, -1]

    cal_time_str = "—"
    if status and status.get("ts_ms") and status.get("last_event_ms"):
        age_ms = status["ts_ms"] - status["last_event_ms"]
        if age_ms >= 0:
            event_epoch = time.time() - age_ms / 1000.0
            cal_time_str = time.strftime("%Y-%m-%d  %H:%M:%S",
                                          time.localtime(event_epoch))

    white = (240, 240, 240, 255)
    pixel_str = f"pixel: [{sp[0]}, {sp[1]}]"
    cal_str   = f"cal'd: {cal_time_str}"
    delta = _resolve_cal_delta(saved_cal, status)
    delta_str = f"Δ{delta}" if delta is not None else ""
    font = _get_font(15)
    y0 = h - info_h + 6
    line_h = 18
    draw2.text((8, y0),            pixel_str, fill=white, font=font)
    draw2.text((8, y0 + line_h),   cal_str,   fill=white, font=font)
    if delta_str:
        try:
            tb = draw2.textbbox((0, 0), pixel_str, font=font)
            dx = 8 + (tb[2] - tb[0]) + 14
        except AttributeError:
            dx = 8 + 9 * len(pixel_str) + 14
        draw2.text((dx, y0), delta_str, fill=white, font=font)

    # Top-left label — fax-style "FAX" prefix to make the artifact's
    # purpose obvious when downloaded standalone.
    if cam_label:
        font_top = _get_font(13)
        text = f"FAX · {cam_label}"
        try:
            tbox = draw2.textbbox((0, 0), text, font=font_top)
            tw = tbox[2] - tbox[0]; th = tbox[3] - tbox[1]
        except AttributeError:
            tw, th = font_top.getsize(text)
        draw2.rectangle([0, 0, tw + 12, th + 8], fill=(0, 0, 0, 220))
        draw2.text((6, 4), text, fill=white, font=font_top)

    out_buf = io.BytesIO()
    out.save(out_buf, format="PNG", optimize=True)
    data = out_buf.getvalue()

    try:
        os.makedirs(ANNOTATED_CACHE_DIR, exist_ok=True)
        with open(os.path.join(ANNOTATED_CACHE_DIR,
                  f"{cam_label.split()[0].lower()}-fax.png"), "wb") as f:
            f.write(data)
    except OSError:
        pass
    return data


def _annotate_snapshot(jpeg_bytes: bytes,
                       status: dict | None,
                       saved_cal: dict | None,
                       cam_label: str) -> bytes:
    """Bullseye + coordinate label at the cal pixel + bottom info bar.
    Mirrors the visual style of the old webui's CAM{N} SEES CAM{M} canvas
    overlay, baked into the JPEG so the artifact is curl-friendly."""
    from PIL import Image, ImageDraw

    img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    w, h = img.size
    draw = ImageDraw.Draw(img, "RGBA")

    # Resolve which pixel to mark. saved_cal.tx_pixel is authoritative
    # ("where I see my peer", written by every successful cal flow); fall
    # back to runtime status.pixel (the daemon's tracking ROI) which equals
    # tx_pixel after a successful cal anyway.
    px = py = None
    if saved_cal and isinstance(saved_cal.get("tx_pixel"), list) \
            and len(saved_cal["tx_pixel"]) == 2:
        px, py = int(saved_cal["tx_pixel"][0]), int(saved_cal["tx_pixel"][1])
    elif status and isinstance(status.get("pixel"), list) \
            and len(status["pixel"]) == 2:
        px, py = int(status["pixel"][0]), int(status["pixel"][1])

    # Draw bullseye (red crosshair + yellow ring + center dot).
    if px is not None and 0 <= px < w and 0 <= py < h:
        arm = 24
        red = (255, 48, 48, 255)
        yellow = (255, 210, 74, 255)
        draw.line([(px - arm, py), (px - 4, py)], fill=red, width=2)
        draw.line([(px + 4, py), (px + arm, py)], fill=red, width=2)
        draw.line([(px, py - arm), (px, py - 4)], fill=red, width=2)
        draw.line([(px, py + 4), (px, py + arm)], fill=red, width=2)
        draw.ellipse([px - 8, py - 8, px + 8, py + 8], outline=yellow, width=2)
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=yellow)

        # Coord label — black bg, yellow text, anchored upper-right of the mark
        label = f"[{px}, {py}]"
        font = _get_font(13)
        try:
            tbox = draw.textbbox((0, 0), label, font=font)
            tw, th = tbox[2] - tbox[0], tbox[3] - tbox[1]
        except AttributeError:
            tw, th = font.getsize(label)
        # Flip the label below the mark if it would clip top edge
        ly = py - 22 - th if py - 22 - th >= 0 else py + 12
        lx = px + 12 if px + 12 + tw + 6 <= w else px - 12 - tw - 6
        draw.rectangle([lx - 3, ly - 2, lx + tw + 3, ly + th + 2],
                       fill=(0, 0, 0, 230))
        draw.text((lx, ly), label, fill=yellow, font=font)

    # Bottom info bar — minimal: just the cal'd pixel + the wall-clock
    # date/time the cal was last written. State/rate/ae/peak/delta/b are
    # available on the webui chip area; baking them into the artifact added
    # noise without value. The image is a *calibration* snapshot, not a
    # runtime telemetry frame.
    info_h = 44
    bar = Image.new("RGBA", (w, info_h), (0, 0, 0, 180))
    img.paste(bar, (0, h - info_h), bar)
    draw2 = ImageDraw.Draw(img, "RGBA")

    if saved_cal:
        sp = saved_cal.get("tx_pixel") or [-1, -1]
    elif status and (status.get("cal") or {}).get("peak"):
        sp = status["cal"]["peak"]
    else:
        sp = [-1, -1]

    # Compute wall-clock event time. Cam timestamps are monotonic
    # (clock_gettime(CLOCK_MONOTONIC) since boot), so we can't read the
    # event time directly. The age (status.ts_ms - status.last_event_ms)
    # IS reliable monotonic delta; subtract it from the laptop's wall
    # clock to land on the event's local time. Cams are time-synced via
    # /x/time-sync.cgi so cam-vs-laptop drift stays under a couple
    # seconds, well within human-readable resolution.
    cal_time_str = "—"
    if status and status.get("ts_ms") and status.get("last_event_ms"):
        age_ms = status["ts_ms"] - status["last_event_ms"]
        if age_ms >= 0:
            event_epoch = time.time() - age_ms / 1000.0
            cal_time_str = time.strftime("%Y-%m-%d  %H:%M:%S",
                                          time.localtime(event_epoch))

    yellow = (255, 210, 74, 255)
    pixel_str = f"pixel: [{sp[0]}, {sp[1]}]"
    cal_str   = f"cal'd: {cal_time_str}"
    delta = _resolve_cal_delta(saved_cal, status)
    delta_str = f"Δ{delta}" if delta is not None else ""

    font = _get_font(15)
    y0 = h - info_h + 6
    line_h = 18
    draw2.text((8, y0),            pixel_str, fill=yellow, font=font)
    draw2.text((8, y0 + line_h),   cal_str,   fill=yellow, font=font)
    if delta_str:
        # Anchor delta to the right of pixel_str on the same line, with a
        # 14px gap. textbbox gives accurate width even for proportional
        # fonts; fall back to a fixed offset on older PIL builds.
        try:
            tb = draw2.textbbox((0, 0), pixel_str, font=font)
            dx = 8 + (tb[2] - tb[0]) + 14
        except AttributeError:
            dx = 8 + 9 * len(pixel_str) + 14
        draw2.text((dx, y0), delta_str, fill=yellow, font=font)

    # Top-left small label (CAM1 SEES CAM2 etc) — burned into the JPEG so
    # the artifact is self-describing when downloaded.
    if cam_label:
        font_top = _get_font(13)
        try:
            tbox = draw2.textbbox((0, 0), cam_label, font=font_top)
            tw = tbox[2] - tbox[0]
            th = tbox[3] - tbox[1]
        except AttributeError:
            tw, th = font_top.getsize(cam_label)
        draw2.rectangle([0, 0, tw + 12, th + 8], fill=(0, 0, 0, 200))
        draw2.text((6, 4), cam_label, fill=(108, 242, 255, 255), font=font_top)

    out = io.BytesIO()
    img.save(out, format="JPEG", quality=85)
    data = out.getvalue()

    # Persist to disk so the artifact is openable/shareable as a static
    # file (`xdg-open /tmp/liwifi-annotated/cam1.jpg`, etc).
    try:
        os.makedirs(ANNOTATED_CACHE_DIR, exist_ok=True)
        with open(os.path.join(ANNOTATED_CACHE_DIR, f"{cam_label.split()[0].lower()}.jpg"), "wb") as f:
            f.write(data)
    except OSError:
        pass
    return data


def init_clients():
    for cam, host in CAM_HOSTS.items():
        try:
            ip = resolve_ip(host)
            CLIENTS[cam] = CamStatusClient(ip)
            print(f"  {cam} → {host} → {ip}")
        except RuntimeError as e:
            print(f"  {cam} → {host} → UNRESOLVED ({e})")


# ============================================================
# RUN CAL: detached subprocess + PID file
# ============================================================
#
# Old design ran cal_procedure under a daemon thread that streamed stdout
# into a deque and SSH-restarted cam1 after `proc.wait()`. Worker thread
# could sit blocked for 5+ minutes on flaky cams and feel like a server
# hang.
#
# New design: spawn cal_procedure with start_new_session=True, redirect
# stdout/stderr to DEVNULL, write its PID to /tmp/liwifi-cal.pid, return
# immediately. cal_procedure self-handles cam1 daemon kill (line 696-704
# in host/cal_procedure.py) and restart between directions, so we don't
# need the SSH steps in the request handler. Status endpoint reports
# liveness via os.kill(pid, 0).
# ============================================================


def _read_cal_pid() -> tuple[int | None, float | None]:
    """Return (pid, started_at_epoch) from the PID file, or (None, None)
    if the file is missing or the process is no longer alive."""
    try:
        with open(CAL_PID_FILE) as f:
            line = f.readline().strip()
    except OSError:
        return None, None
    parts = line.split(",", 1)
    try:
        pid = int(parts[0])
    except (ValueError, IndexError):
        return None, None
    try:
        started = float(parts[1]) if len(parts) > 1 else None
    except ValueError:
        started = None
    try:
        os.kill(pid, 0)
    except (OSError, ProcessLookupError):
        return None, None
    return pid, started


def _start_cal_subprocess() -> tuple[int, float] | None:
    """Spawn cal_procedure detached; write PID file; return (pid, t0).
    Returns None if a previous run is still alive."""
    pid, started = _read_cal_pid()
    if pid is not None:
        return None
    t0 = time.time()
    proc = subprocess.Popen(
        [sys.executable, "-u", "-m", "host.cal_procedure", "--no-interactive"],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        with open(CAL_PID_FILE, "w") as f:
            f.write(f"{proc.pid},{t0:.3f}\n")
    except OSError:
        pass
    return proc.pid, t0


# ============================================================
# HTTP handler
# ============================================================


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEBUI_DIR, **kwargs)

    def log_message(self, format, *args): pass

    def handle_one_request(self):
        try:
            super().handle_one_request()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, code: int, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, code: int, body: str):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, code: int, content_type: str, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> str:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0: return ""
        return self.rfile.read(n).decode("utf-8", errors="replace")

    def _client_for(self, cam: str) -> CamStatusClient | None:
        return CLIENTS.get(cam)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path.startswith("/api/"):
            self._handle_api_get(path, parsed.query)
            return
        super().do_GET()

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path.startswith("/api/"):
            self._handle_api_post(path)
            return
        self._send_json(405, {"ok": False, "error": "method not allowed"})

    # --- /api routing ---

    def _handle_api_get(self, path: str, query: str = ""):
        if path == "/api/info":
            # Cam IPs + link type ("usb" / "wifi" / "unknown") + server
            # start epoch for the status-bar footer and the hero-panel
            # connection icons. Link type comes from the IP prefix:
            # 172.16.x.x is the USB NCM subnet, 192.168.x.x is the LAN.
            def _link_for(ip: str | None) -> str:
                if not ip: return "unknown"
                if ip.startswith("172.16."):   return "usb"
                if ip.startswith("192.168."):  return "wifi"
                return "unknown"
            cam1_ip = CLIENTS["cam1"].ip if "cam1" in CLIENTS else None
            cam2_ip = CLIENTS["cam2"].ip if "cam2" in CLIENTS else None
            self._send_json(200, {
                "started_at": SERVER_STARTED_AT,
                "cam1_ip": cam1_ip,
                "cam2_ip": cam2_ip,
                "cam1_link": _link_for(cam1_ip),
                "cam2_link": _link_for(cam2_ip),
            })
            return

        if path == "/api/cal/status":
            pid, started = _read_cal_pid()
            self._send_json(200, {
                "running": pid is not None,
                "pid": pid,
                "started_at": started,
            })
            return

        if path == "/api/cal/monocal-status":
            cam2 = self._client_for("cam2")
            cam1 = self._client_for("cam1")
            cam2_status = _cam_get_or_fetch(
                "cam2", "cam2:monocal-status", _CACHE_TTL["monocal-status"],
                cam2.get_monocal_status) if cam2 else None
            cam1_status = _cam_get_or_fetch(
                "cam1", "cam1:status", _CACHE_TTL["status"],
                cam1.get) if cam1 else None
            self._send_json(200, {
                "cam2": cam2_status or {"state": "transient"},
                "cam1": cam1_status or {},
            })
            return

        parts = path.strip("/").split("/")
        if len(parts) != 3:
            self._send_json(404, {"ok": False, "error": "bad path"})
            return
        _, cam, op = parts
        client = self._client_for(cam)
        if client is None:
            self._send_json(503, {"ok": False, "error": f"cam {cam!r} not initialised"})
            return

        if op == "grid":
            r = _cam_get_or_fetch(cam, f"{cam}:grid", _CACHE_TTL["grid"],
                                  client.get_brightness_grid)
            if r is None:
                self._send_json(200, {"ts": None, "blocks": None})
                return
            ts, blocks = r
            self._send_json(200, {"ts": ts, "blocks": blocks})
            return

        if op == "ae":
            v = _cam_get_or_fetch(cam, f"{cam}:ae", _CACHE_TTL["ae"],
                                  client.get_ae_freeze)
            self._send_json(200, {"value": v})
            return

        if op == "proc":
            r = _cam_get_or_fetch(cam, f"{cam}:proc", _CACHE_TTL["proc"],
                                  client.get_proc_status)
            if r is None:
                self._send_json(200, {"irlink": None, "daynightd": None, "prudynt": None})
                return
            self._send_json(200, r)
            return

        if op == "status":
            r = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                  client.get)
            if r is None:
                self._send_json(200, {})
                return
            self._send_json(200, r)
            return

        if op == "health":
            # Bundled health check. Cam-load optimization (Phase 1+2 of the
            # congestion fix): all-status.cgi now ALSO carries the vitals
            # fields (uptime/load/mem/disk/dmesg) so a single CGI gives us
            # everything. We piggyback on the live page poller's `:all`
            # cache (it polls every 2s anyway) — when warm, this endpoint
            # adds ZERO load to the cam. Cold path falls back to the
            # standalone health.cgi, mainly for direct CLI probing on a
            # quiescent webui.
            t0 = time.monotonic()
            allst = _cache_peek(f"{cam}:all", _CACHE_TTL["all"])
            if allst is None:
                # Cache miss → fetch via the normal single-flight path.
                allst = _cam_get_or_fetch(cam, f"{cam}:all", _CACHE_TTL["all"],
                                          client.get_all_status)
            # Extract vitals: prefer the all-status payload (one round trip
            # already paid for above); fall back to standalone health.cgi
            # if the cam is running an older build of all-status.cgi that
            # doesn't include them yet.
            vitals = None
            if isinstance(allst, dict):
                vitals = allst.get("vitals")
            if vitals is None:
                vitals = _cam_get_or_fetch(cam, f"{cam}:health",
                                           _CACHE_TTL["health"], client.get_health)
            uhttpd_ms = int((time.monotonic() - t0) * 1000)
            if isinstance(allst, dict) and isinstance(allst.get("ae"), int):
                allst["ae"] = {"value": allst["ae"]}
            self._send_json(200, {
                "ok": vitals is not None and allst is not None,
                "vitals": vitals or {},
                "all":    allst or {
                    "status": {},
                    "leds":   {"ir850": None, "ir940": None},
                    "ae":     None,
                    "proc":   {"irlink": None, "daynightd": None, "prudynt": None},
                },
                "uhttpd_rt_ms": uhttpd_ms,
            })
            return

        if op == "all":
            # Single consolidated read of status+leds+ae+proc, one CGI fork
            # on the cam. The webui poll cycle uses this to avoid 4-way
            # serialization on cam1's single-process uhttpd. Demux happens
            # client-side. Shape always present so the frontend can rely
            # on `r.proc.irlink` etc. without optional-chaining everywhere.
            r = _cam_get_or_fetch(cam, f"{cam}:all", _CACHE_TTL["all"],
                                  client.get_all_status)
            if r is None:
                self._send_json(200, {
                    "status": {},
                    "leds":   {"ir850": None, "ir940": None},
                    "ae":     None,
                    "proc":   {"irlink": None, "daynightd": None, "prudynt": None},
                })
                return
            # Normalize ae from int to {value: int} so it matches /api/{cam}/ae.
            if isinstance(r.get("ae"), int):
                r["ae"] = {"value": r["ae"]}
            self._send_json(200, r)
            return

        if op == "leds":
            r = _cam_get_or_fetch(cam, f"{cam}:leds", _CACHE_TTL["leds"],
                                  client.get_led_state)
            if r is None:
                self._send_json(200, {"ir850": None, "ir940": None, "ircut": None})
                return
            self._send_json(200, r)
            return

        if op == "snapshot":
            ch = 1
            if query:
                params = dict(urllib.parse.parse_qsl(query))
                if params.get("ch") == "0":
                    ch = 0
            data = _cam_get_or_fetch(
                cam, f"{cam}:snapshot:{ch}", _CACHE_TTL["snapshot"],
                lambda: client.get_snapshot(channel=ch))
            if data is None:
                self._send_text(503, "(snapshot fetch failed)\n")
                return
            self._send_bytes(200, "image/jpeg", data)
            return

        if op in ("snapshot-tiny", "snapshot-tiny.png"):
            # Tiny 64×36 1-bit wireframe encoded for eventual IR transfer.
            # Default returns raw bytes (288 B — bare raster, no header;
            # dimensions are part of the wire-format contract) for the
            # would-be transmit path. ?fmt=png OR the .png suffix returns
            # an 8×-upscaled PNG for browser display in the lo-res webui card.
            want_png = op.endswith(".png")
            if not want_png and query:
                qparams = dict(urllib.parse.parse_qsl(query))
                if qparams.get("fmt") == "png":
                    want_png = True
            raw = _cam_get_or_fetch(
                cam, f"{cam}:snapshot:1", _CACHE_TTL["snapshot"],
                lambda: client.get_snapshot(channel=1))
            if raw is None:
                self._send_text(503, "(snapshot fetch failed)\n")
                return
            # Pull saved_cal so the rendered PNG can show the bullseye at
            # the cal pixel. The wire bytes (return value [0]) never carry
            # the bullseye — receiver overlays it from coords it already
            # has from CAL_DONE.
            saved = _cam_get_or_fetch(cam, f"{cam}:saved-cal", 10.0,
                                      client.get_saved_cal)
            status = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                       client.get) or {}
            label = f"{cam.upper()} SEES {('CAM2' if cam == 'cam1' else 'CAM1')}"
            try:
                bin_bytes, png_bytes = _tiny_wireframe(raw, label, saved, status)
            except Exception as e:
                self._send_text(500, f"(tiny encode failed: {type(e).__name__}: {e})\n")
                return
            if want_png:
                self._send_bytes(200, "image/png", png_bytes)
            else:
                self._send_bytes(200, "application/octet-stream", bin_bytes)
            return

        if op == "snapshot-fax":
            # Fax/wireframe variant of snapshot-annotated — designed to
            # look like the cal artifact would if it were small enough to
            # transmit over the IR link itself. Returns PNG.
            raw = _cam_get_or_fetch(
                cam, f"{cam}:snapshot:1", _CACHE_TTL["snapshot"],
                lambda: client.get_snapshot(channel=1))
            if raw is None:
                self._send_text(503, "(snapshot fetch failed)\n")
                return
            status = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                       client.get) or {}
            saved = _cam_get_or_fetch(cam, f"{cam}:saved-cal", 10.0,
                                      client.get_saved_cal)
            label = f"{cam.upper()} SEES {('CAM2' if cam == 'cam1' else 'CAM1')}"
            try:
                img_bytes = _faxify_snapshot(raw, status, saved, label)
            except Exception as e:
                self._send_text(500, f"(faxify failed: {type(e).__name__}: {e})\n")
                return
            self._send_bytes(200, "image/png", img_bytes)
            return

        if op == "snapshot-annotated":
            # Composite the saved cal pixel as a bullseye + coord label +
            # info bar onto the substream JPEG. Single curl returns a
            # self-describing image that's also persisted to
            # /tmp/liwifi-annotated/{cam}.jpg.
            raw = _cam_get_or_fetch(
                cam, f"{cam}:snapshot:1", _CACHE_TTL["snapshot"],
                lambda: client.get_snapshot(channel=1))
            if raw is None:
                self._send_text(503, "(snapshot fetch failed)\n")
                return
            status = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                       client.get) or {}
            saved = _cam_get_or_fetch(cam, f"{cam}:saved-cal", 10.0,
                                      client.get_saved_cal)
            label = f"{cam.upper()} SEES {('CAM2' if cam == 'cam1' else 'CAM1')}"
            try:
                img_bytes = _annotate_snapshot(raw, status, saved, label)
            except Exception as e:
                self._send_text(500, f"(annotate failed: {type(e).__name__}: {e})\n")
                return
            self._send_bytes(200, "image/jpeg", img_bytes)
            return

        if op == "cal-info":
            r = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                  client.get)
            if r is None:
                r = {}
            else:
                r = dict(r)
            saved = _cam_get_or_fetch(cam, f"{cam}:saved-cal", 10.0,
                                      client.get_saved_cal)
            if saved is not None:
                r["saved_cal"] = saved
            r["snapshot_url"] = f"/api/{cam}/snapshot"
            self._send_json(200, r)
            return

        if op == "grid-deltas":
            r = _cam_get_or_fetch(cam, f"{cam}:grid-deltas", 1.0,
                                  client.get_grid_deltas)
            if r is None:
                self._send_json(404, {"state": "never_run"})
                return
            self._send_json(200, r)
            return

        if op == "events":
            tail = None
            if query:
                params = dict(urllib.parse.parse_qsl(query))
                t = params.get("tail")
                if t and t.isdigit():
                    tail = int(t)
            if not _probe_alive(client.ip):
                self._send_text(503, "(cam unreachable)\n")
                return
            body = client.get_events_log(tail=tail)
            if body is None:
                self._send_text(503, "(could not reach events-log.cgi)\n")
                return
            self._send_text(200, body)
            return

        if op == "syslog":
            # Filtered busybox syslog ring buffer — captures rc.local boot
            # trail, prudynt debug, etc. ?tag=<sub>&tail=N forwarded to CGI.
            tag = None
            tail = None
            if query:
                params = dict(urllib.parse.parse_qsl(query))
                tag = params.get("tag") or None
                t = params.get("tail")
                if t and t.isdigit():
                    tail = int(t)
            if not _probe_alive(client.ip):
                self._send_text(503, "(cam unreachable)\n")
                return
            body = client.get_syslog(tag=tag, tail=tail)
            if body is None:
                self._send_text(503, "(could not reach logread.cgi)\n")
                return
            self._send_text(200, body)
            return

        self._send_json(404, {"ok": False, "error": f"unknown op {op!r}"})

    def _handle_api_post(self, path: str):
        if path == "/api/cal/start":
            result = _start_cal_subprocess()
            if result is None:
                pid, started = _read_cal_pid()
                self._send_json(409, {"ok": False,
                    "error": "cal already running",
                    "pid": pid, "started_at": started})
                return
            pid, started = result
            self._send_json(200, {"ok": True, "pid": pid, "started_at": started})
            return

        if path == "/api/cal/monocal":
            cam2 = self._client_for("cam2")
            if cam2 is None:
                self._send_json(503, {"ok": False, "error": "cam2 not initialised"})
                return
            body = self._read_body()
            params = dict(urllib.parse.parse_qsl(body))
            coords_str = params.get("coords")
            if coords_str:
                try:
                    cx, cy = (int(p) for p in coords_str.split(",", 1))
                except (ValueError, TypeError):
                    self._send_json(400, {"ok": False,
                                          "error": "coords must be X,Y"})
                    return
                coords = (cx, cy)
            else:
                coords = _read_cam2_sees_cam1_pixel()
                if coords is None:
                    self._send_json(400, {"ok": False,
                        "error": "no coords; run cal_procedure first or pass coords=X,Y"})
                    return
            try:
                speed_ms = int(params.get("speed_ms", "160"))
            except (ValueError, TypeError):
                speed_ms = 160

            # Pre-flight: setup.cgi on cam2 puts it in a known IR-comm-ready
            # state (ircut open, daynightd dead, LEDs off, ae_freeze=0,
            # mono/night). Without this, a prior `kill -9 irlink` can leave
            # ae_freeze=1 or LEDs stuck on, and monocal silently produces a
            # weak/failed cal. setup is skipped only if `setup=0` is passed.
            # cam1 is left alone — its daemon-listen has its own state mgmt
            # and rc.local already ran setup at boot.
            if params.get("setup", "1") != "0":
                setup_res = cam2.run_setup(timeout=20.0)
                if setup_res is None or not setup_res.get("ok"):
                    self._send_json(503, {
                        "ok": False,
                        "error": "cam2 setup pre-flight failed",
                        "setup": setup_res,
                    })
                    return

            result = cam2.start_monocal(coords, speed_ms=speed_ms)
            if result is None:
                self._send_json(409, {"ok": False,
                    "error": "monocal trigger refused (irlink already running on cam2?)"})
                return
            self._send_json(200, {"ok": True, "coords": list(coords),
                                  "speed_ms": speed_ms, "trigger": result})
            return

        if path == "/api/cal/monocal-both":
            # Fused MONOCAL BOTH: single trigger that runs Phase 1 + Phase 2
            # in one irlink session (single handshake). Spawns
            # `irlink monocal --both` via the new monocal-both-trigger CGI.
            # Setup pre-flight runs by default (first trigger of the session,
            # mirrors /api/cal/monocal). The webui polls /api/cal/monocal-status
            # — Phase 1 walks monocal_* states, Phase 2 walks monocal_rev_*
            # states. Final state: monocal_rev_done.
            cam2 = self._client_for("cam2")
            if cam2 is None:
                self._send_json(503, {"ok": False, "error": "cam2 not initialised"})
                return
            body = self._read_body()
            params = dict(urllib.parse.parse_qsl(body))
            coords_str = params.get("coords")
            if coords_str:
                try:
                    cx, cy = (int(p) for p in coords_str.split(",", 1))
                except (ValueError, TypeError):
                    self._send_json(400, {"ok": False,
                                          "error": "coords must be X,Y"})
                    return
                coords = (cx, cy)
            else:
                coords = _read_cam2_sees_cam1_pixel()
                if coords is None:
                    self._send_json(400, {"ok": False,
                        "error": "no coords; run cal_procedure first or pass coords=X,Y"})
                    return
            try:
                speed_ms = int(params.get("speed_ms", "160"))
            except (ValueError, TypeError):
                speed_ms = 160

            # Pre-flight setup on cam2 (mirrors /api/cal/monocal). cam1 is
            # untouched — its daemon-listen has its own state mgmt and
            # rc.local already ran setup at boot.
            if params.get("setup", "1") != "0":
                setup_res = cam2.run_setup(timeout=20.0)
                if setup_res is None or not setup_res.get("ok"):
                    self._send_json(503, {
                        "ok": False,
                        "error": "cam2 setup pre-flight failed",
                        "setup": setup_res,
                    })
                    return

            result = cam2.start_monocal_both(coords, speed_ms=speed_ms)
            if result is None:
                self._send_json(409, {"ok": False,
                    "error": "monocal-both trigger refused (irlink already running on cam2?)"})
                return
            self._send_json(200, {"ok": True, "coords": list(coords),
                                  "speed_ms": speed_ms, "mode": "both",
                                  "trigger": result})
            return

        if path == "/api/cal/monocal-rev":
            # Phase 2 of bidirectional cal: cam2 SCANS, cam1 HOLDS.
            # Mirror of /api/cal/monocal but spawns `irlink monocal --reverse`.
            # Result: cam2's /opt/etc/calibration.json refreshes.
            # Setup pre-flight is skipped by default — this endpoint is
            # intended to chain immediately after /api/cal/monocal which
            # just ran setup on cam2. Force a re-setup with setup=1.
            cam2 = self._client_for("cam2")
            if cam2 is None:
                self._send_json(503, {"ok": False, "error": "cam2 not initialised"})
                return
            body = self._read_body()
            params = dict(urllib.parse.parse_qsl(body))
            coords_str = params.get("coords")
            if coords_str:
                try:
                    cx, cy = (int(p) for p in coords_str.split(",", 1))
                except (ValueError, TypeError):
                    self._send_json(400, {"ok": False,
                                          "error": "coords must be X,Y"})
                    return
                coords = (cx, cy)
            else:
                coords = _read_cam2_sees_cam1_pixel()
                if coords is None:
                    self._send_json(400, {"ok": False,
                        "error": "no coords; run cal_procedure first or pass coords=X,Y"})
                    return
            try:
                speed_ms = int(params.get("speed_ms", "160"))
            except (ValueError, TypeError):
                speed_ms = 160

            # Setup defaults OFF here — assume the caller (MONOCAL BOTH
            # orchestration) already ran setup before Phase 1. Explicit
            # setup=1 forces a re-run for standalone reverse triggers.
            if params.get("setup", "0") == "1":
                setup_res = cam2.run_setup(timeout=20.0)
                if setup_res is None or not setup_res.get("ok"):
                    self._send_json(503, {
                        "ok": False,
                        "error": "cam2 setup pre-flight failed",
                        "setup": setup_res,
                    })
                    return

            result = cam2.start_monocal_rev(coords, speed_ms=speed_ms)
            if result is None:
                self._send_json(409, {"ok": False,
                    "error": "monocal-rev trigger refused (irlink already running on cam2?)"})
                return
            self._send_json(200, {"ok": True, "coords": list(coords),
                                  "speed_ms": speed_ms, "mode": "reverse",
                                  "trigger": result})
            return

        parts = path.strip("/").split("/")
        if len(parts) != 3:
            self._send_json(404, {"ok": False, "error": "bad path"})
            return
        _, cam, op = parts
        client = self._client_for(cam)
        if client is None:
            self._send_json(503, {"ok": False, "error": f"cam {cam!r} not initialised"})
            return
        body = self._read_body()
        params = dict(urllib.parse.parse_qsl(body))

        if op == "ae":
            value = params.get("value")
            if value not in ("0", "1"):
                self._send_json(400, {"ok": False, "error": "value must be 0 or 1"})
                return
            ok = client.set_ae_freeze(value == "1")
            _cache_invalidate(f"{cam}:ae", f"{cam}:status", f"{cam}:cal-info")
            self._send_json(200 if ok else 502, {"ok": ok, "value": int(value) if ok else None})
            return

        if op == "time-sync":
            report = client.set_time()
            if report is None:
                self._send_json(502, {"ok": False,
                                      "error": "time-sync CGI unreachable"})
                return
            self._send_json(200, report)
            return

        if op == "setup":
            report = client.run_setup()
            if report is None:
                self._send_json(502, {"ok": False, "error": "setup CGI unreachable"})
                return
            self._send_json(200 if report.get("ok") else 207, report)
            return

        if op == "led":
            lamp = params.get("lamp")
            value = params.get("value")
            if lamp not in ("ir850", "ir940") or value not in ("0", "1"):
                self._send_json(400, {"ok": False, "error": "lamp ∈ {ir850,ir940}; value ∈ {0,1}"})
                return
            if value == "1":
                client.imp_cmd("daynight", "night")
            ok = client.imp_cmd(lamp, int(value))
            _cache_invalidate(f"{cam}:leds", f"{cam}:status",
                              f"{cam}:cal-info", f"{cam}:proc")
            self._send_json(200 if ok else 502, {"ok": ok, "lamp": lamp, "value": int(value) if ok else None})
            return

        self._send_json(404, {"ok": False, "error": f"unknown op {op!r}"})


_TIME_SYNC_INTERVAL_S = 3600


def _periodic_time_sync_loop():
    """Background daemon: re-push laptop time to every cam every hour.
    Cams have no RTC and outbound NTP is blocked, so without this they drift
    apart over long-running sessions. Best-effort."""
    while True:
        time.sleep(_TIME_SYNC_INTERVAL_S)
        for name, cl in list(CLIENTS.items()):
            try:
                r = cl.set_time()
                if r and r.get("drift_s") is not None:
                    drift = r["drift_s"]
                    if abs(drift) >= 2:
                        print(f"[time-sync] {name}: corrected drift {drift:+}s")
            except Exception as e:
                print(f"[time-sync] {name}: {type(e).__name__}: {e}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--bind", default="127.0.0.1")
    args = p.parse_args()

    print("liwifi thin webui server")
    print(f"  webui dir: {WEBUI_DIR}")
    print(f"  resolving cams:")
    init_clients()
    print(f"  listening: http://{args.bind}:{args.port}/")
    threading.Thread(target=_periodic_time_sync_loop, daemon=True).start()

    httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
