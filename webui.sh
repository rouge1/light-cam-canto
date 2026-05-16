#!/usr/bin/env bash
# webui.sh — orchestrator for the LiWiFi live-status webpage.
#
#   ./webui.sh start [--no-preflight] [--no-browser]
#   ./webui.sh stop
#   ./webui.sh restart
#   ./webui.sh status
#   ./webui.sh logs [webui|photos|both]
#
# Spawns two background servers:
#   - photos HTTP server (port 8889)  → CAL RESULTS iframe target
#   - webui server       (port 8765)  → live dashboard
# Plus runs host/cam_setup.sh as preflight (opens IR-cut, kills daynightd).
# PIDs + logs live in /tmp/lightcam-webui/.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="/tmp/lightcam-webui"
PHOTOS_PORT=8889
WEBUI_PORT=8765
WEBUI_HOST="127.0.0.1"
CONDA_ENV="light"

mkdir -p "$RUN_DIR"

PHOTOS_PID="$RUN_DIR/photos.pid"
WEBUI_PID="$RUN_DIR/webui.pid"
PHOTOS_LOG="$RUN_DIR/photos.log"
WEBUI_LOG="$RUN_DIR/webui.log"
BOOT_LOG="$RUN_DIR/boot.log"

# Reboot persistence. `webui.sh install` adds an idempotent @reboot line to
# the user crontab — no root/sudo, and it survives reboot regardless of login
# (unlike a systemd --user unit, which needs `loginctl enable-linger`). cron's
# env is minimal, but find_python locates the conda python via $HOME and the
# script is invoked by absolute path; --no-preflight skips the camera-
# dependent (and currently routeless) /api/setup-all at boot; --no-browser
# since there's no display. The marker is a trailing sh-comment (cron runs the
# line via `sh -c`, which ignores `#…`) so a single grep adds/removes it.
SELF="$ROOT/webui.sh"
CRON_MARK="# lightcam-webui-autostart"
CRON_LINE="@reboot $SELF start --no-preflight --no-browser >> $BOOT_LOG 2>&1  $CRON_MARK"

if [ -t 1 ]; then
  C_GREEN=$'\033[32m'; C_RED=$'\033[31m'; C_AMBER=$'\033[33m'
  C_DIM=$'\033[2m';    C_BOLD=$'\033[1m'; C_RESET=$'\033[0m'
else
  C_GREEN=""; C_RED=""; C_AMBER=""; C_DIM=""; C_BOLD=""; C_RESET=""
fi

# Resolve the python from conda env 'light'. Prefer an already-activated env;
# otherwise look in standard conda install locations.
find_python() {
  if [ "${CONDA_DEFAULT_ENV:-}" = "$CONDA_ENV" ] && command -v python >/dev/null 2>&1; then
    command -v python; return
  fi
  for p in \
    "${CONDA_PREFIX:-}/envs/$CONDA_ENV/bin/python" \
    "${CONDA_PREFIX_1:-}/envs/$CONDA_ENV/bin/python" \
    "$HOME/miniconda3/envs/$CONDA_ENV/bin/python" \
    "$HOME/anaconda3/envs/$CONDA_ENV/bin/python" \
    "/opt/conda/envs/$CONDA_ENV/bin/python" \
    "/opt/miniconda3/envs/$CONDA_ENV/bin/python"; do
    [ -n "$p" ] && [ -x "$p" ] && { echo "$p"; return; }
  done
  echo "ERROR: could not find python in conda env '$CONDA_ENV'." >&2
  echo "  Either activate it first ('conda activate $CONDA_ENV') or" >&2
  echo "  edit CONDA_ENV at the top of this script." >&2
  return 1
}

is_running() {
  local pid_file="$1" pid
  [ -f "$pid_file" ] || return 1
  pid=$(cat "$pid_file" 2>/dev/null) || return 1
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

port_owner() {
  # PID owning a listening TCP port, if any (only sees own user's procs without root).
  ss -ltnp "sport = :$1" 2>/dev/null \
    | awk -F'pid=' 'NR>1{split($2,a,","); print a[1]; exit}'
}

print_one() {
  local label="$1" pid_file="$2" port="$3" url="$4"
  if is_running "$pid_file"; then
    local pid; pid=$(cat "$pid_file")
    printf "  ${C_GREEN}●${C_RESET} ${C_BOLD}%-15s${C_RESET}  pid=%-7s port=%s\n" "$label" "$pid" "$port"
    [ -n "$url" ] && printf "      ${C_DIM}%s${C_RESET}\n" "$url"
  else
    local owner; owner=$(port_owner "$port" || true)
    if [ -n "$owner" ]; then
      printf "  ${C_AMBER}!${C_RESET} ${C_BOLD}%-15s${C_RESET}  ${C_AMBER}foreign pid=%s on port %s${C_RESET}\n" \
        "$label" "$owner" "$port"
    else
      printf "  ${C_RED}○${C_RESET} ${C_BOLD}%-15s${C_RESET}  ${C_DIM}stopped${C_RESET}\n" "$label"
    fi
  fi
}

autostart_status() {
  if command -v crontab >/dev/null 2>&1 && crontab -l 2>/dev/null | grep -qF "$CRON_MARK"; then
    printf "  ${C_GREEN}●${C_RESET} ${C_BOLD}%-15s${C_RESET}  ${C_DIM}@reboot via crontab${C_RESET}\n" "autostart"
  else
    printf "  ${C_RED}○${C_RESET} ${C_BOLD}%-15s${C_RESET}  ${C_DIM}not installed — ./webui.sh install${C_RESET}\n" "autostart"
  fi
}

cmd_status() {
  echo -e "${C_BOLD}LiWiFi webui${C_RESET}  ${C_DIM}($RUN_DIR)${C_RESET}"
  print_one "photos server" "$PHOTOS_PID" "$PHOTOS_PORT" "http://127.0.0.1:$PHOTOS_PORT/calibration.html"
  print_one "webui server"  "$WEBUI_PID"  "$WEBUI_PORT"  "http://$WEBUI_HOST:$WEBUI_PORT/"
  autostart_status
}

cmd_install() {
  command -v crontab >/dev/null 2>&1 || {
    echo -e "${C_RED}✗${C_RESET} crontab not found — cannot install reboot autostart" >&2
    exit 1
  }
  local cur filtered
  cur=$(crontab -l 2>/dev/null || true)
  filtered=$(printf '%s\n' "$cur" | grep -vF "$CRON_MARK" || true)
  # Re-emit existing entries minus ours, then append ours; drop blank lines.
  printf '%s\n%s\n' "$filtered" "$CRON_LINE" | grep -v '^[[:space:]]*$' | crontab -
  echo -e "${C_GREEN}✓${C_RESET} reboot autostart installed (user crontab @reboot)"
  echo -e "  ${C_DIM}${CRON_LINE}${C_RESET}"
  echo -e "  ${C_DIM}boot output → $BOOT_LOG${C_RESET}"
  echo -e "  ${C_DIM}note: starts once at boot; no auto-restart if it later crashes${C_RESET}"
}

cmd_uninstall() {
  command -v crontab >/dev/null 2>&1 || { echo -e "${C_DIM}crontab not found${C_RESET}"; return 0; }
  local cur filtered
  cur=$(crontab -l 2>/dev/null || true)
  if ! printf '%s\n' "$cur" | grep -qF "$CRON_MARK"; then
    echo -e "${C_DIM}reboot autostart not installed${C_RESET}"; return 0
  fi
  filtered=$(printf '%s\n' "$cur" | grep -vF "$CRON_MARK" || true)
  if [ -z "$(printf '%s' "$filtered" | tr -d '[:space:]')" ]; then
    crontab -r 2>/dev/null || true
  else
    printf '%s\n' "$filtered" | grep -v '^[[:space:]]*$' | crontab -
  fi
  echo -e "${C_GREEN}✓${C_RESET} reboot autostart removed"
}

start_bg() {
  # start_bg <name> <pidfile> <logfile> <workdir> <cmd...>
  #
  # Spawns CMD detached, writes the actual python PID (not a launcher's) to
  # pidfile. We can't rely on `nohup CMD &; echo $!` because coreutils-on-Linux
  # nohup forks for tty/stderr handling, leaving $! at the nohup wrapper PID
  # (one less than the real python). Bash's `( ... ) &` adds another fork on
  # top. We saw stop fail to kill anything because of this.
  #
  # Robust pattern: have the soon-to-be-python process write its OWN $$ before
  # exec'ing the real command. exec preserves PID across image replacement, so
  # the PID written equals the python PID.
  local name="$1" pidfile="$2" logfile="$3" workdir="$4"; shift 4
  nohup bash -c '
    cd "$1" || exit 1
    pidfile="$2"
    logfile="$3"
    shift 3
    # Final redirections inside the wrapper so they apply to the exec'\''d
    # process directly. /dev/null on stdin guards against SIGTTIN/SIGTTOU.
    exec >>"$logfile" 2>&1 </dev/null
    # Write our PID (= python'\''s PID after exec) to the pidfile.
    echo $$ >"$pidfile"
    # Replace ourselves with the target command. PID is preserved.
    exec "$@"
  ' _ "$workdir" "$pidfile" "$logfile" "$@" </dev/null >/dev/null 2>&1 &
  disown
}

cmd_start() {
  local do_preflight=1 open_browser=1
  while [ $# -gt 0 ]; do
    case "$1" in
      --no-preflight) do_preflight=0 ;;
      --no-browser)   open_browser=0 ;;
      *) echo "unknown flag: $1" >&2; exit 2 ;;
    esac
    shift
  done

  if is_running "$PHOTOS_PID" && is_running "$WEBUI_PID"; then
    echo "Both servers already running."
    cmd_status
    return 0
  fi

  local py; py=$(find_python) || exit 1
  echo -e "${C_DIM}python: $py${C_RESET}"

  # NOTE on order: preflight is now POST /api/setup-all, served by the webui
  # server itself (which fans out to each cam's /x/setup.cgi). So we have to
  # start the servers first, then trigger preflight. The legacy SSH-driven
  # host/cam_setup.sh is still on disk for recovery cases that need SSH-only
  # paths (prudynt restart, /opt mount, irlink redeploy).

  # 1. photos server
  if is_running "$PHOTOS_PID"; then
    echo -e "${C_DIM}photos server already up (pid $(cat "$PHOTOS_PID"))${C_RESET}"
  else
    local owner; owner=$(port_owner "$PHOTOS_PORT" || true)
    if [ -n "$owner" ]; then
      echo -e "${C_RED}✗${C_RESET} port $PHOTOS_PORT busy (pid $owner) — refusing to start photos server."
      exit 1
    fi
    [ -d "$ROOT/photos" ] || { echo -e "${C_RED}✗${C_RESET} $ROOT/photos missing"; exit 1; }
    echo -e "${C_BOLD}▸ photos server${C_RESET}  port $PHOTOS_PORT"
    start_bg "photos" "$PHOTOS_PID" "$PHOTOS_LOG" "$ROOT/photos" \
      "$py" -m http.server "$PHOTOS_PORT" --bind 127.0.0.1
  fi

  # 2. webui server
  if is_running "$WEBUI_PID"; then
    echo -e "${C_DIM}webui server already up (pid $(cat "$WEBUI_PID"))${C_RESET}"
  else
    local owner; owner=$(port_owner "$WEBUI_PORT" || true)
    if [ -n "$owner" ]; then
      echo -e "${C_RED}✗${C_RESET} port $WEBUI_PORT busy (pid $owner) — refusing to start webui server."
      exit 1
    fi
    echo -e "${C_BOLD}▸ webui server${C_RESET}   port $WEBUI_PORT"
    start_bg "webui" "$WEBUI_PID" "$WEBUI_LOG" "$ROOT" \
      "$py" -m webui.server --bind "$WEBUI_HOST" --port "$WEBUI_PORT"
  fi

  # 3. give them a moment to bind, then verify
  sleep 0.8
  is_running "$PHOTOS_PID" || {
    echo -e "${C_RED}✗${C_RESET} photos server failed to start — see $PHOTOS_LOG"
    tail -n 20 "$PHOTOS_LOG" 2>/dev/null | sed 's/^/    /'
    exit 1
  }
  is_running "$WEBUI_PID" || {
    echo -e "${C_RED}✗${C_RESET} webui server failed to start — see $WEBUI_LOG"
    tail -n 20 "$WEBUI_LOG" 2>/dev/null | sed 's/^/    /'
    exit 1
  }

  # 4. preflight via the webui server itself (POST /api/setup-all)
  if [ "$do_preflight" -eq 1 ]; then
    echo -e "${C_BOLD}▸ preflight${C_RESET}  POST /api/setup-all"
    local out
    out=$(curl -fsS -m 30 -X POST "http://$WEBUI_HOST:$WEBUI_PORT/api/setup-all" 2>&1) || {
      echo -e "${C_AMBER}!${C_RESET} setup-all curl failed: $out"
      out=""
    }
    if [ -n "$out" ]; then
      "$py" -c '
import json, sys
r = json.loads(sys.stdin.read())
overall = "OK" if r.get("ok") else "FAIL"
print(f"  overall: {overall}")
for cam, rep in (r.get("cams") or {}).items():
    cam_ok = rep.get("ok")
    label = "OK" if cam_ok else "FAIL"
    err = rep.get("error", "")
    suffix = f"  {err}" if not cam_ok and err else ""
    print(f"  [{cam}] {label}{suffix}")
    for s in rep.get("steps", []) or []:
        mark = "OK " if s.get("ok") else "ERR"
        name = s.get("name", "")
        detail = s.get("detail", "")
        print(f"    [{mark}] {name:<22} {detail}")
    ts = rep.get("time_sync") or {}
    if ts.get("ok"):
        drift = ts.get("drift_s")
        print(f"    [OK ] time-sync             corrected drift {drift:+}s")
    elif ts:
        msg = ts.get("error", "unknown")
        print(f"    [ERR] time-sync             {msg}")
' <<< "$out" || echo "$out"
    fi
  else
    echo -e "${C_DIM}skipping preflight${C_RESET}"
  fi

  echo
  cmd_status
  echo
  echo -e "${C_GREEN}▶${C_RESET} ${C_BOLD}http://$WEBUI_HOST:$WEBUI_PORT/${C_RESET}"
  if [ "$open_browser" -eq 1 ] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://$WEBUI_HOST:$WEBUI_PORT/" >/dev/null 2>&1 &
  fi
}

stop_one() {
  local label="$1" pid_file="$2" pid
  if is_running "$pid_file"; then
    pid=$(cat "$pid_file")
    echo -e "${C_BOLD}▸ stopping $label${C_RESET} (pid $pid)"
    kill "$pid" 2>/dev/null || true
    for _ in $(seq 1 6); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.3
    done
    if kill -0 "$pid" 2>/dev/null; then
      echo -e "${C_AMBER}!${C_RESET} $label ignored SIGTERM — sending SIGKILL"
      kill -9 "$pid" 2>/dev/null || true
    fi
  else
    echo -e "${C_DIM}$label not running${C_RESET}"
  fi
  rm -f "$pid_file"
}

cmd_stop() {
  stop_one "webui server"  "$WEBUI_PID"
  stop_one "photos server" "$PHOTOS_PID"
}

cmd_logs() {
  local which="${1:-webui}"
  case "$which" in
    photos)    tail -F "$PHOTOS_LOG" ;;
    webui)     tail -F "$WEBUI_LOG" ;;
    both|all)  tail -F "$WEBUI_LOG" "$PHOTOS_LOG" ;;
    *) echo "logs takes: webui | photos | both" >&2; exit 2 ;;
  esac
}

usage() {
  cat <<EOF
LiWiFi webui control script

  ./webui.sh start [--no-preflight] [--no-browser]
                                       run cam_setup.sh, start photos + webui
  ./webui.sh stop                      stop both servers
  ./webui.sh restart                   stop + start
  ./webui.sh status                    show running state + autostart
  ./webui.sh logs [webui|photos|both]  tail server logs (default: webui)
  ./webui.sh install                   start automatically on reboot (@reboot cron)
  ./webui.sh uninstall                 remove the reboot autostart

  webui:   http://$WEBUI_HOST:$WEBUI_PORT/   (port $WEBUI_PORT)
  photos:  http://127.0.0.1:$PHOTOS_PORT/    (port $PHOTOS_PORT, iframe target)
  state:   $RUN_DIR
EOF
}

case "${1:-}" in
  start)             shift; cmd_start "$@" ;;
  stop)              cmd_stop ;;
  restart)           cmd_stop; sleep 0.4; shift || true; cmd_start "$@" ;;
  status|st)         cmd_status ;;
  logs|log|tail)     shift; cmd_logs "$@" ;;
  install)           cmd_install ;;
  uninstall)         cmd_uninstall ;;
  ""|-h|--help|help) usage ;;
  *) echo "unknown command: ${1:-}" >&2; usage; exit 2 ;;
esac
