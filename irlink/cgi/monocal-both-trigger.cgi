#!/bin/sh
# Thingino CGI: trigger a fused `irlink monocal --both` run on cam2.
#
# Runs Phase 1 (cam2 holds + cam1 scans → cam1.json) and Phase 2
# (cam2 scans + cam1 holds → cam2.json) in ONE irlink session — single
# SYN handshake, single AE-freeze epoch. Saves the ~55s second-handshake
# overhead that the two-trigger orchestration path paid for.
#
# Same status-JSON state names as the individual triggers; the webui can
# tell which phase is in progress by the monocal_* vs monocal_rev_* prefix.
# Final success state is monocal_rev_done (Phase 2 wrote last).
#
# POST coords=X,Y[&speed=N]
#   coords  required, cam2's pixel for cam1's TX
#   speed   optional, ms/symbol (default 160)
#
# Refusals: 409 if irlink already running; 400 if coords missing/bad.
#
# Deploy: scp -O monocal-both-trigger.cgi root@dacam2:/var/www/x/
#         ssh dacam2 "chmod 755 /var/www/x/monocal-both-trigger.cgi"
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

case "$SPEED" in
    ''|*[!0-9]*) SPEED=160 ;;
esac
if [ "$SPEED" -lt 40 ] 2>/dev/null || [ "$SPEED" -gt 2000 ] 2>/dev/null; then
    SPEED=160
fi

RUNNING=$(pgrep irlink 2>/dev/null | wc -l)
if [ "$RUNNING" -gt 0 ] 2>/dev/null; then
    echo "Status: 409 Conflict"
    echo "Content-Type: application/json"
    echo ""
    echo "{\"ok\":false,\"error\":\"irlink already running\",\"running\":$RUNNING}"
    exit 0
fi

# Pre-write status. "spawning_both" so the webui poller knows fused mode
# started; the irlink binary overwrites with real state within ms.
NOW_MS=$(date +%s)000
TMP=/run/monocal-status.json.tmp.$$
cat > "$TMP" <<EOF
{"schema":1,"state":"spawning_both","started_ms":$NOW_MS,"updated_ms":$NOW_MS,"ended_ms":null,"ok":null,"error":"","ack_received_ms":0,"led_hold_started_ms":0,"led_hold_ended_ms":0,"peer_pixel":null,"speed_ms":$SPEED}
EOF
mv "$TMP" /run/monocal-status.json 2>/dev/null

setsid /opt/bin/irlink monocal --both --pixel "$COORDS" --speed "$SPEED" \
    > /run/monocal-stdout.log 2>&1 < /dev/null &
PID=$!

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
echo "{\"ok\":true,\"pid\":$PID,\"coords\":\"$COORDS\",\"speed_ms\":$SPEED,\"mode\":\"both\"}"
