#!/bin/sh
# Thingino CGI: trigger an `irlink monocal --reverse` run on cam2.
#
# Phase 2 of bidirectional cal — cam2 is the SCANNER, cam1 is the HOLDER.
# Mirror of monocal-trigger.cgi but adds --reverse to the spawned binary so
# the requestor (cam2) inverts its role: instead of holding LEDs, cam2 sends
# CAL_REQ with flags=0 (regular cal), then runs do_calibrate_pixel_to() during
# cam1's LED hold window. Result: cam2's /opt/etc/calibration.json updates.
#
# Same status JSON pattern as the forward trigger — irlink writes
# /run/monocal-status.json with monocal_rev_* state names that the webui
# polls via /x/monocal-status.cgi. State machine flow:
#   spawning_rev → starting → handshaking → monocal_rev_req
#   → monocal_rev_ack_wait → monocal_rev_scanning → monocal_rev_done
#
# POST coords=X,Y[&speed=N]
#   coords  required, cam2's pixel for cam1's TX (from cam2's calibration.json)
#   speed   optional, ms/symbol (default 160)
#
# Refusals:
#   409 Conflict  if irlink already running (any subcommand) — caller should
#                 stop the conflicting process first. Most likely cause when
#                 chaining after Phase 1: previous monocal hasn't fully exited
#                 yet. Wait ~1s and retry.
#   400 Bad Req   if coords missing or malformed
#
# Deploy: scp -O monocal-rev-trigger.cgi root@dacam2:/var/www/x/
#         ssh dacam2 "chmod 755 /var/www/x/monocal-rev-trigger.cgi"
# Mode MUST be 755 — uhttpd serves mode 700 as 403 Forbidden.

if [ "$REQUEST_METHOD" != "POST" ]; then
    echo "Status: 405 Method Not Allowed"
    echo "Content-Type: application/json"
    echo ""
    echo '{"ok":false,"error":"POST only"}'
    exit 0
fi

LEN="${CONTENT_LENGTH:-0}"
if [ "$LEN" -gt 256 ] 2>/dev/null; then
    LEN=256
fi
if [ "$LEN" -gt 0 ] 2>/dev/null; then
    BODY=$(head -c "$LEN")
else
    BODY=""
fi

# Parse k=v&k=v form.
COORDS=""
SPEED=160
OLD_IFS="$IFS"
IFS='&'
for kv in $BODY; do
    case "$kv" in
        coords=*) COORDS="${kv#coords=}" ;;
        speed=*)  SPEED="${kv#speed=}" ;;
    esac
done
IFS="$OLD_IFS"

# Validate coords as "<int>,<int>"
case "$COORDS" in
    [0-9]*,[0-9]*) : ;;
    *)
        echo "Status: 400 Bad Request"
        echo "Content-Type: application/json"
        echo ""
        echo '{"ok":false,"error":"coords required as X,Y"}'
        exit 0
        ;;
esac

# Validate speed
case "$SPEED" in
    ''|*[!0-9]*) SPEED=160 ;;
esac
if [ "$SPEED" -lt 40 ] 2>/dev/null || [ "$SPEED" -gt 2000 ] 2>/dev/null; then
    SPEED=160
fi

# Refuse if any irlink already running.
RUNNING=$(pgrep irlink 2>/dev/null | wc -l)
if [ "$RUNNING" -gt 0 ] 2>/dev/null; then
    echo "Status: 409 Conflict"
    echo "Content-Type: application/json"
    echo ""
    echo "{\"ok\":false,\"error\":\"irlink already running\",\"running\":$RUNNING}"
    exit 0
fi

# Pre-write status. Distinct "spawning_rev" so the webui poller can tell
# Phase 2 has been kicked off even before irlink writes its first state.
# Without this, the first poll might see leftover "monocal_done" from
# Phase 1 and mistakenly conclude Phase 2 already finished.
NOW_MS=$(date +%s)000
TMP=/run/monocal-status.json.tmp.$$
cat > "$TMP" <<EOF
{"schema":1,"state":"spawning_rev","started_ms":$NOW_MS,"updated_ms":$NOW_MS,"ended_ms":null,"ok":null,"error":"","ack_received_ms":0,"led_hold_started_ms":0,"led_hold_ended_ms":0,"peer_pixel":null,"speed_ms":$SPEED}
EOF
mv "$TMP" /run/monocal-status.json 2>/dev/null

# Spawn detached with --reverse — cam2 scans, cam1 holds.
setsid /opt/bin/irlink monocal --reverse --pixel "$COORDS" --speed "$SPEED" \
    > /run/monocal-stdout.log 2>&1 < /dev/null &
PID=$!

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
echo "{\"ok\":true,\"pid\":$PID,\"coords\":\"$COORDS\",\"speed_ms\":$SPEED,\"mode\":\"reverse\"}"
