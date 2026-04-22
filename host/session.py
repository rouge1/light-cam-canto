"""Laptop-side orchestrator for the IR-link application layer.

Runs two SSH subprocesses (one per camera), each driving `irlink listen` or
`irlink connect` in interactive mode. Parses `MSG-HEX:` lines from stdout
and dispatches app-layer messages; writes `send-hex ...` to stdin.

See /home/user/.claude/plans/steady-finding-pearl.md for design details.

Usage:
    python -m host.session --dry-run
    python -m host.session --stats-interval 10
    python -m host.session --symbol-ms 120 --text "HELLO"
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from protocol import app
from protocol.app import (
    APP_HELLO, APP_META, APP_META_ACK, APP_CAL_RESULT, APP_STATS,
    APP_TEXT, APP_BYE, APP_CHUNK, APP_CHUNK_ACK, APP_NACK,
    AppMessage, pack_hello, pack_meta, pack_meta_ack, pack_cal_result,
    pack_text, pack_bye, pack_chunk_ack,
    fragment, reassemble, missing_chunks,
    STATUS_IDLE, STATUS_READY, STATUS_BUSY, STATUS_ERROR,
    MAX_SINGLE_FRAME, MAX_CHUNK_DATA,
)

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")


def _ts() -> str:
    return time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"


class CamLink:
    """One SSH-driven camera running `irlink` in interactive mode."""

    def __init__(self, host: str, role: str, pixel: tuple[int, int],
                 symbol_ms: int, label: str, log_fn: Callable[[str], None]):
        assert role in ("listen", "connect")
        self.host = host
        self.role = role
        self.pixel = pixel
        self.symbol_ms = symbol_ms
        self.label = label
        self.log = log_fn
        self.proc: Optional[subprocess.Popen] = None
        self.connected = threading.Event()
        self._stdout_thr: Optional[threading.Thread] = None
        self._stderr_thr: Optional[threading.Thread] = None
        self._app_queue: queue.Queue[AppMessage] = queue.Queue()
        self._stats_queue: queue.Queue[dict] = queue.Queue()
        self._stop = threading.Event()
        # Send-complete signalling: irlink prints "PROTO: ACK received for seq="
        # after our DATA was ACKed by the peer, or "PROTO: send failed" after
        # all retries are exhausted. We use this to serialize back-to-back
        # sends — without it, the orchestrator's next send_app() can fire while
        # our irlink is still mid-TX, causing both sides to TX simultaneously.
        self._send_complete = threading.Event()
        self._send_ok = False

    def start(self):
        x, y = self.pixel
        # Thingino busybox has no stdbuf — irlink calls fflush(stdout)
        # after each MSG-HEX/MSG/STATS line, and stderr is line-buffered
        # by default, so we don't need external buffering tools.
        remote_cmd = (
            f"/opt/bin/irlink {self.role} "
            f"--pixel {x},{y} --speed {self.symbol_ms}"
        )
        ssh_cmd = ["ssh", "-T", f"root@{self.host}", remote_cmd]
        self.log(f"[{self.label}] launching: {' '.join(ssh_cmd)}")
        # Read pipes as raw bytes — irlink's legacy "MSG: <text>" line dumps
        # the binary app payload directly, which can contain non-UTF-8 bytes
        # (e.g. META's timestamp/version fields). text=True would crash the
        # reader thread on UnicodeDecodeError. We decode with errors="replace"
        # per-line below.
        self.proc = subprocess.Popen(
            ssh_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._stdout_thr = threading.Thread(target=self._read_stdout, daemon=True)
        self._stderr_thr = threading.Thread(target=self._read_stderr, daemon=True)
        self._stdout_thr.start()
        self._stderr_thr.start()

    def _read_stdout(self):
        assert self.proc and self.proc.stdout
        for raw in self.proc.stdout:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if self._stop.is_set():
                break
            if line.startswith("MSG-HEX:"):
                hex_part = line[len("MSG-HEX:"):].strip()
                try:
                    payload = bytes.fromhex(hex_part)
                    msg = app.unpack(payload)
                    self.log(f"[{self.label}] RX app={_type_name(msg.type)} fields={msg.fields}")
                    self._app_queue.put(msg)
                except Exception as e:
                    self.log(f"[{self.label}] parse error: {e} (line={line!r})")
            elif line.startswith("STATS:"):
                stats = _parse_stats_line(line)
                self.log(f"[{self.label}] stats: {stats}")
                self._stats_queue.put(stats)
            elif line.startswith("PONG:"):
                self.log(f"[{self.label}] {line}")
            elif line.startswith("MSG:"):
                # Legacy text form — we use MSG-HEX; ignore.
                pass
            else:
                self.log(f"[{self.label}] stdout: {line}")

    def _read_stderr(self):
        assert self.proc and self.proc.stderr
        for raw in self.proc.stderr:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if self._stop.is_set():
                break
            if "connected!" in line:
                self.connected.set()
                self.log(f"[{self.label}] CONNECTED")
            # Send-outcome lines from irlink's do_send_bytes()
            if "ACK received for seq=" in line:
                self._send_ok = True
                self._send_complete.set()
            elif "PROTO: send failed" in line:
                self._send_ok = False
                self._send_complete.set()
            self.log(f"[{self.label}] stderr: {line}")

    def send_app(self, payload: bytes, wait: bool = True,
                 timeout: float = 800.0) -> Optional[bool]:
        """Write a send-hex command to irlink's stdin. If wait=True, block
        until irlink prints 'ACK received' or 'send failed' on stderr; returns
        True on success, False on send-failed, raises TimeoutError if neither
        appears within `timeout` seconds. With wait=False, returns None
        immediately (fire-and-forget). Default wait=True serializes back-to-
        back sends so the orchestrator never queues a new DATA while the
        previous one is still in flight."""
        assert self.proc and self.proc.stdin
        hex_str = payload.hex()
        self.log(f"[{self.label}] TX hex({len(payload)}B) type={_type_name(payload[0])}"
                 f" wait={wait}")
        # Clear BEFORE writing so we don't observe a previous send's result.
        self._send_complete.clear()
        self._send_ok = False
        self.proc.stdin.write(f"send-hex {hex_str}\n".encode("ascii"))
        self.proc.stdin.flush()
        if not wait:
            return None
        if not self._send_complete.wait(timeout):
            raise TimeoutError(
                f"[{self.label}] send did not complete within {timeout}s "
                f"(no 'ACK received' or 'send failed' on stderr)"
            )
        if not self._send_ok:
            self.log(f"[{self.label}] send FAILED")
        else:
            self.log(f"[{self.label}] send confirmed (peer ACKed)")
        return self._send_ok

    def wait_send_complete(self, timeout: float = 800.0) -> bool:
        """Block until the most recent send_app(wait=False) completes."""
        if not self._send_complete.wait(timeout):
            raise TimeoutError(
                f"[{self.label}] send did not complete within {timeout}s"
            )
        return self._send_ok

    def send_cmd(self, cmd: str):
        assert self.proc and self.proc.stdin
        self.log(f"[{self.label}] TX cmd: {cmd}")
        self.proc.stdin.write(f"{cmd}\n".encode("ascii"))
        self.proc.stdin.flush()

    def wait_for_app(self, type_filter: Optional[int] = None,
                     timeout: float = 120.0) -> AppMessage:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"[{self.label}] timeout waiting for app type={type_filter}"
                )
            try:
                msg = self._app_queue.get(timeout=min(remaining, 1.0))
            except queue.Empty:
                continue
            if type_filter is None or msg.type == type_filter:
                return msg

    def request_stats(self, timeout: float = 10.0) -> dict:
        while not self._stats_queue.empty():
            try:
                self._stats_queue.get_nowait()
            except queue.Empty:
                break
        self.send_cmd("stats")
        return self._stats_queue.get(timeout=timeout)

    def stop(self):
        self._stop.set()
        if self.proc:
            try:
                if self.proc.stdin and not self.proc.stdin.closed:
                    self.proc.stdin.write(b"quit\n")
                    self.proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.proc.kill()


def _type_name(t: int) -> str:
    names = {
        APP_HELLO: "HELLO", APP_META: "META", APP_META_ACK: "META_ACK",
        APP_CAL_RESULT: "CAL_RESULT", APP_STATS: "STATS",
        APP_TEXT: "TEXT", APP_BYE: "BYE",
        APP_CHUNK: "CHUNK", APP_CHUNK_ACK: "CHUNK_ACK",
        APP_NACK: "NACK",
    }
    return names.get(t, f"0x{t:02x}")


def _parse_stats_line(line: str) -> dict:
    # "STATS: tx=12 rx=8 crc=1 rtx=2 dll=3"
    out = {}
    body = line[len("STATS:"):].strip()
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out


def _load_calibration() -> dict:
    with open(CALIBRATION_FILE) as f:
        return json.load(f)


@dataclass
class SessionConfig:
    cam1_host: str = "da-camera1"
    cam2_host: str = "da-camera2"
    cam1_name: str = "cam1-window"
    cam2_name: str = "cam2-laptop"
    symbol_ms: int = 120
    handshake_timeout: float = 180.0
    msg_timeout: float = 180.0


class IRSession:
    """Drives a full HELLO → META → CAL_RESULT → TEXT → BYE flow."""

    def __init__(self, cfg: SessionConfig, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn
        cal = _load_calibration()
        self._cam1_sees_cam2 = tuple(cal["cam1_sees_cam2"]["tx_pixel"])
        self._cam2_sees_cam1 = tuple(cal["cam2_sees_cam1"]["tx_pixel"])
        self.cam1 = CamLink(
            cfg.cam1_host, "listen", self._cam1_sees_cam2,
            cfg.symbol_ms, "cam1", log_fn,
        )
        self.cam2 = CamLink(
            cfg.cam2_host, "connect", self._cam2_sees_cam1,
            cfg.symbol_ms, "cam2", log_fn,
        )
        self._next_msg_id = 0

    def start(self):
        self.cam1.start()
        time.sleep(2.0)  # let the listener bind before the initiator dials
        self.cam2.start()
        if not self.cam1.connected.wait(self.cfg.handshake_timeout):
            raise TimeoutError("cam1 never connected")
        if not self.cam2.connected.wait(self.cfg.handshake_timeout):
            raise TimeoutError("cam2 never connected")
        self.log(f"[session] handshake complete at {_ts()}")

    def hello_exchange(self) -> dict:
        """cam2→cam1 HELLO (name), then bilateral META (state) + META_ACK."""
        self.log(f"[session] HELLO exchange start at {_ts()}")
        # cam2 announces itself
        self.cam2.send_app(pack_hello(self.cfg.cam2_name))
        hello_cam2 = self.cam1.wait_for_app(APP_HELLO, timeout=self.cfg.msg_timeout)
        # cam1 replies with its name
        self.cam1.send_app(pack_hello(self.cfg.cam1_name))
        hello_cam1 = self.cam2.wait_for_app(APP_HELLO, timeout=self.cfg.msg_timeout)

        # cam1 → cam2: META (state)
        self.cam1.send_app(pack_meta(int(time.time()), STATUS_READY))
        cam1_meta_on_cam2 = self.cam2.wait_for_app(APP_META, timeout=self.cfg.msg_timeout)
        self.cam2.send_app(pack_meta_ack())
        self.cam1.wait_for_app(APP_META_ACK, timeout=self.cfg.msg_timeout)

        # cam2 → cam1: META (state)
        self.cam2.send_app(pack_meta(int(time.time()), STATUS_READY))
        cam2_meta_on_cam1 = self.cam1.wait_for_app(APP_META, timeout=self.cfg.msg_timeout)
        self.cam1.send_app(pack_meta_ack())
        self.cam2.wait_for_app(APP_META_ACK, timeout=self.cfg.msg_timeout)

        self.log(f"[session] HELLO exchange complete at {_ts()}")
        return {
            "cam1_name": hello_cam1.fields["name"],
            "cam2_name": hello_cam2.fields["name"],
            "cam1_meta": cam1_meta_on_cam2.fields,
            "cam2_meta": cam2_meta_on_cam1.fields,
        }

    def cal_result_exchange(self) -> dict:
        """Both sides report the pixel they're watching; compare to calibration.json."""
        self.log(f"[session] CAL_RESULT exchange at {_ts()}")
        # cam1 tells cam2 "I see you at cam1_sees_cam2"
        x1, y1 = self._cam1_sees_cam2
        self.cam1.send_app(pack_cal_result(x1, y1))
        r1 = self.cam2.wait_for_app(APP_CAL_RESULT, timeout=self.cfg.msg_timeout)
        # cam2 tells cam1 "I see you at cam2_sees_cam1"
        x2, y2 = self._cam2_sees_cam1
        self.cam2.send_app(pack_cal_result(x2, y2))
        r2 = self.cam1.wait_for_app(APP_CAL_RESULT, timeout=self.cfg.msg_timeout)

        match_1 = (r1.fields["x"], r1.fields["y"]) == (x1, y1)
        match_2 = (r2.fields["x"], r2.fields["y"]) == (x2, y2)
        self.log(
            f"[session] CAL_RESULT: cam1→cam2 {r1.fields} match={match_1}, "
            f"cam2→cam1 {r2.fields} match={match_2}"
        )
        return {
            "cam1_reports": r1.fields,
            "cam2_reports": r2.fields,
            "cam1_matches_calibration": match_1,
            "cam2_matches_calibration": match_2,
        }

    def send_text(self, text: str, direction: str = "cam2_to_cam1"):
        sender, receiver = (self.cam2, self.cam1) if direction == "cam2_to_cam1" else (self.cam1, self.cam2)
        data = text.encode("utf-8")
        if len(data) + 1 <= MAX_SINGLE_FRAME:
            sender.send_app(pack_text(text))
            msg = receiver.wait_for_app(APP_TEXT, timeout=self.cfg.msg_timeout)
            assert msg.fields["text"] == text, f"text mismatch: got {msg.fields['text']!r}"
            return msg.fields["text"]
        # Fragmented path
        return self._send_fragmented(sender, receiver, APP_TEXT, data)

    def _send_fragmented(self, sender: CamLink, receiver: CamLink,
                         app_type: int, data: bytes,
                         max_attempts: int = 3) -> str:
        msg_id = self._next_msg_id & 0xFF
        self._next_msg_id += 1
        chunks = fragment(app_type, data, msg_id=msg_id)
        total = len(chunks)
        self.log(f"[session] fragmented send: msg_id={msg_id} total={total}")

        to_send = list(range(total))
        received: dict[int, bytes] = {}

        for attempt in range(max_attempts):
            for seq in to_send:
                sender.send_app(chunks[seq])

            # Collect chunks from receiver until we have all or timeout
            deadline = time.monotonic() + self.cfg.msg_timeout
            while set(received.keys()) != set(range(total)):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    msg = receiver.wait_for_app(APP_CHUNK, timeout=min(remaining, 2.0))
                except TimeoutError:
                    break
                if msg.fields["msg_id"] != msg_id:
                    continue
                received[msg.fields["seq"]] = msg.fields["data"]

            got = sorted(received.keys())
            receiver.send_app(pack_chunk_ack(msg_id, got, total))
            # Wait for the ACK bitmap on sender's side
            try:
                ack = sender.wait_for_app(APP_CHUNK_ACK, timeout=self.cfg.msg_timeout)
                missing = missing_chunks(ack.fields["bitmap"], total)
            except TimeoutError:
                missing = [s for s in range(total) if s not in received]

            if not missing:
                raw_chunks = [
                    app.pack_chunk(msg_id, s, total, received[s])
                    for s in range(total)
                ]
                app_type_out, body = reassemble(raw_chunks)
                assert app_type_out == app_type
                return body.decode("utf-8", errors="replace")
            self.log(f"[session] retransmit missing chunks: {missing}")
            to_send = missing

        raise RuntimeError(f"fragmented send failed after {max_attempts} attempts")

    def stats(self) -> dict:
        s1 = self.cam1.request_stats()
        s2 = self.cam2.request_stats()
        self.log(f"[session] stats cam1={s1} cam2={s2}")
        return {"cam1": s1, "cam2": s2}

    def bye(self, reason: str = ""):
        self.log(f"[session] BYE at {_ts()}")
        try:
            self.cam2.send_app(pack_bye(reason))
            self.cam1.wait_for_app(APP_BYE, timeout=self.cfg.msg_timeout)
        except TimeoutError:
            self.log("[session] BYE not confirmed, tearing down anyway")
        self.cam1.stop()
        self.cam2.stop()


def dry_run():
    """Exercise pack/unpack/fragment logic without hitting any cameras."""
    print("[dry-run] verifying app-layer pack/unpack round-trips...")
    for name, builder in [
        ("HELLO", lambda: pack_hello("cam2-laptop")),
        ("META", lambda: pack_meta(int(time.time()), STATUS_READY)),
        ("META_ACK", lambda: pack_meta_ack()),
        ("CAL_RESULT", lambda: pack_cal_result(351, 165)),
        ("TEXT-short", lambda: pack_text("HELLO")),
        ("BYE", lambda: pack_bye("done")),
    ]:
        b = builder()
        m = app.unpack(b)
        print(f"  {name}: {len(b)}B  → type=0x{m.type:02x} fields={m.fields}")

    print("[dry-run] fragmentation...")
    pangram = "The quick brown fox jumps over the lazy dog — twice over!"
    data = pangram.encode()
    chunks = fragment(APP_TEXT, data)
    print(f"  pangram={len(data)}B → {len(chunks)} chunks ×≤16B")
    t, body = reassemble(chunks)
    assert t == APP_TEXT and body == data
    print("  reassembled OK")

    print("[dry-run] done.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--symbol-ms", type=int, default=120)
    p.add_argument("--text", default="HELLO")
    p.add_argument("--cam1-host", default="da-camera1")
    p.add_argument("--cam2-host", default="da-camera2")
    p.add_argument("--cam1-name", default="cam1-window")
    p.add_argument("--cam2-name", default="cam2-laptop")
    p.add_argument("--handshake-only", action="store_true",
                   help="Verify SSH orchestration + protocol handshake, then exit")
    p.add_argument("--skip-hello", action="store_true")
    p.add_argument("--skip-cal", action="store_true")
    p.add_argument("--skip-text", action="store_true")
    p.add_argument("--skip-stats", action="store_true")
    args = p.parse_args()

    if args.dry_run:
        dry_run()
        return

    def log(msg: str):
        print(f"{_ts()} {msg}", flush=True)

    cfg = SessionConfig(
        cam1_host=args.cam1_host, cam2_host=args.cam2_host,
        cam1_name=args.cam1_name, cam2_name=args.cam2_name,
        symbol_ms=args.symbol_ms,
    )
    sess = IRSession(cfg, log_fn=log)
    try:
        sess.start()
        if args.handshake_only:
            log("[session] handshake-only mode: stopping after connect")
            sess.cam1.stop()
            sess.cam2.stop()
            return
        if not args.skip_hello:
            sess.hello_exchange()
        if not args.skip_cal:
            sess.cal_result_exchange()
        if not args.skip_text:
            echoed = sess.send_text(args.text, direction="cam2_to_cam1")
            log(f"[session] TEXT echo: {echoed!r}")
        if not args.skip_stats:
            sess.stats()
        sess.bye("session complete")
    except Exception as e:
        log(f"[session] ERROR: {type(e).__name__}: {e}")
        sess.cam1.stop()
        sess.cam2.stop()
        raise


if __name__ == "__main__":
    main()
