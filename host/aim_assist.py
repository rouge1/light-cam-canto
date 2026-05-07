#!/usr/bin/env python3
"""aim_assist — laptop tool to point cam2 at cam1 with live feedback.

Workflow:
  1. SSH cam1 → hold IR LEDs ON for ~120s
  2. Poll cam2's /run/prudynt/brightness_grid every 250ms via SSH
  3. Show live max-block delta vs baseline (★ GOOD / ◆ OK / · weak)
  4. User moves cam2 until aim is good, presses Enter
  5. Tool kills cam1 LEDs, restarts cam1's daemon-listen, runs bicall
     from cam2 → fresh /opt/etc/calibration.json on both sides.

Currently uses SSH to drive cam1 (dev mode — cam1 still on WiFi). Replace
the SSH LED-hold with an AIM_REQ message over light when cam1 goes truly
autonomous.

Usage:
  python -m host.aim_assist                           # aim → bicall
  python -m host.aim_assist --no-cal                  # aim only
  python -m host.aim_assist --cam2-pixel 271,91       # cam2's coord for bicall
  python -m host.aim_assist --hold 180                # longer LED hold window
"""
import argparse
import select
import signal
import subprocess
import sys
import termios
import time
import tty

CAM1 = "da-camera1"
CAM2 = "da-camera2"

RED = "\033[0;31m"
YEL = "\033[0;33m"
GRN = "\033[0;32m"
CYA = "\033[0;36m"
RST = "\033[0m"

THRESHOLD_GOOD = 80
THRESHOLD_OK = 40


def ssh(host, cmd, timeout=5):
    r = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=3", "-o", "BatchMode=yes", host, cmd],
        capture_output=True, text=True, timeout=timeout,
    )
    return r.stdout if r.returncode == 0 else None


def read_grid(host):
    out = ssh(host, "cat /run/prudynt/brightness_grid", timeout=3)
    if not out:
        return None
    parts = out.split()
    if len(parts) < 241:
        return None
    try:
        ts = int(parts[0])
        blocks = [int(x) for x in parts[1:241]]
    except ValueError:
        return None
    return ts, blocks


def find_max(blocks):
    idx = max(range(240), key=lambda i: blocks[i])
    return idx, blocks[idx], idx % 20, idx // 20


def measure_baseline(host, n=8):
    print(f"Measuring baseline (cam1 LEDs OFF) on {host}...")
    samples = []
    for _ in range(n):
        g = read_grid(host)
        if g is not None:
            samples.append(find_max(g[1])[1])
        time.sleep(0.25)
    if not samples:
        raise RuntimeError(f"Could not read brightness_grid from {host}")
    baseline = sum(samples) // len(samples)
    print(f"  baseline max-block brightness: {baseline}")
    return baseline


def kill_cam1_daemon(host):
    """Stop any irlink daemon on cam1 so we can hold LEDs manually."""
    ssh(host, "killall irlink 2>/dev/null", timeout=3)
    time.sleep(1)
    ssh(host, "echo 0 > /run/prudynt/ae_freeze 2>/dev/null", timeout=3)


def hold_leds(host, hold_seconds):
    """Background SSH that holds cam1 LEDs ON, then turns them off."""
    cmd = (
        'echo \'{"daynight":{"enabled":false}}\' | prudyntctl json - >/dev/null 2>&1; '
        'light ir850 on; light ir940 on; '
        f'sleep {hold_seconds}; '
        'light ir850 off; light ir940 off'
    )
    return subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", host, cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def force_leds_off(host):
    ssh(host, "light ir850 off; light ir940 off", timeout=3)


def stdin_has_data():
    return select.select([sys.stdin], [], [], 0)[0]


def aim_loop(baseline):
    print()
    print(f"{CYA}Move cam2 until you see GOOD aim. [Enter]=cal  [q]=quit{RST}")
    print()
    last_len = 0
    while True:
        if stdin_has_data():
            ch = sys.stdin.read(1)
            if ch in ("\r", "\n"):
                print()
                return "cal"
            if ch.lower() == "q":
                print()
                return "quit"
        g = read_grid(CAM2)
        if g is None:
            line, col = "ERR: cam2 not responding", RED
        else:
            _, bright, bx, by = find_max(g[1])
            delta = bright - baseline
            if delta >= THRESHOLD_GOOD:
                tag, col = "★ GOOD AIM", GRN
            elif delta >= THRESHOLD_OK:
                tag, col = "◆ OK", YEL
            else:
                tag, col = "· weak", RED
            line = (f"max-block ({bx:2d},{by:2d}) bright={bright:3d} "
                    f"baseline={baseline:3d} delta=+{delta:3d}  {tag}")
        sys.stdout.write("\r" + " " * last_len + "\r" + col + line + RST)
        sys.stdout.flush()
        last_len = len(line)
        time.sleep(0.25)


def restart_cam1_daemon(host):
    """Read /opt/etc/calibration.json on cam1 and restart daemon-listen."""
    coords = ssh(host,
        'grep \'"tx_pixel"\' /opt/etc/calibration.json '
        '| grep -oE "[0-9]+" | head -2 | tr "\\n" "," | sed "s/,$//"',
        timeout=5)
    if not coords or not coords.strip():
        raise RuntimeError(f"{host}: /opt/etc/calibration.json missing or unparseable")
    coords = coords.strip()
    print(f"  starting cam1 daemon-listen --pixel {coords} --speed 160")
    subprocess.run(
        ["ssh", "-o", "BatchMode=yes", host,
         f"nohup /opt/bin/irlink daemon-listen --pixel {coords} "
         "--speed 160 >/var/log/irlink-boot.log 2>&1 &"],
        timeout=5,
    )
    time.sleep(3)
    out = ssh(host, "pgrep -af 'irlink daemon-listen'")
    if not out or "daemon-listen" not in out:
        raise RuntimeError(f"{host}: daemon-listen failed to start")


def run_bicall(cam2_pixel):
    """Spawn cam2 irlink connect, send bicall, stream output until done."""
    print(f"\n{CYA}=== bicall via cam2 (--pixel {cam2_pixel}) ==={RST}")
    cmd = f"/opt/bin/irlink connect --pixel {cam2_pixel} --speed 160"
    proc = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", CAM2, cmd],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    sent_bicall = False
    sent_quit = False
    try:
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if not sent_bicall and "PROTO: connected!" in line:
                proc.stdin.write("bicall\n")
                proc.stdin.flush()
                sent_bicall = True
            if "BICAL: bidirectional cal complete" in line or "BICAL: phase 1" in line and "failed" in line:
                if not sent_quit:
                    proc.stdin.write("quit\n")
                    proc.stdin.flush()
                    sent_quit = True
        proc.wait(timeout=10)
    except (KeyboardInterrupt, subprocess.TimeoutExpired):
        proc.kill()


def main():
    p = argparse.ArgumentParser(
        description="Aim cam2 at cam1 with live feedback, then trigger bicall")
    p.add_argument("--cam1", default=CAM1)
    p.add_argument("--cam2", default=CAM2)
    p.add_argument("--hold", type=int, default=120,
                   help="seconds to hold cam1 LEDs ON for aim phase")
    p.add_argument("--cam2-pixel", default="258,92",
                   help="cam2's pixel coord for cam1's TX (passed to bicall)")
    p.add_argument("--no-cal", action="store_true",
                   help="skip bicall after aim — just print final aim quality")
    args = p.parse_args()
    global CAM1, CAM2
    CAM1, CAM2 = args.cam1, args.cam2

    if not sys.stdin.isatty():
        print("aim_assist needs a real TTY for keypress input", file=sys.stderr)
        return 1

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    led_proc = None

    def cleanup(*_):
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        except Exception:
            pass
        if led_proc and led_proc.poll() is None:
            led_proc.terminate()
        force_leds_off(CAM1)

    signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(130)))

    try:
        print(f"{CYA}Stopping cam1 daemon (frees LED control)...{RST}")
        kill_cam1_daemon(CAM1)
        force_leds_off(CAM1)
        time.sleep(1)

        baseline = measure_baseline(CAM2)

        print(f"{CYA}Holding cam1 IR LEDs ON for {args.hold}s...{RST}")
        led_proc = hold_leds(CAM1, args.hold)
        time.sleep(2)  # let LEDs settle

        tty.setcbreak(fd)
        action = aim_loop(baseline)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)

        if led_proc and led_proc.poll() is None:
            led_proc.terminate()
        force_leds_off(CAM1)

        if action == "quit":
            print("Quit without cal.")
            return 0

        if args.no_cal:
            print("--no-cal: skipping bicall.")
            return 0

        print(f"\n{CYA}=== restarting cam1 daemon-listen ==={RST}")
        restart_cam1_daemon(CAM1)
        run_bicall(args.cam2_pixel)
        return 0
    finally:
        cleanup()


if __name__ == "__main__":
    sys.exit(main())
