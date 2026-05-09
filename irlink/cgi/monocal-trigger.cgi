#!/bin/sh
# Thingino CGI: trigger an `irlink monocal` run on cam2.
#
# Spawns the monocal subcommand in the background via setsid+nohup so it
# survives the HTTP request, then returns immediately with the spawned PID.
# The exchange takes ~50s and writes /run/monocal-status.json as it
# progresses; the laptop polls /x/monocal-status.cgi for state changes.
#
# POST coords=X,Y[&speed=N]
#   coords  required, cam2's pixel for cam1's TX (from cam2's calibration.json)
#   speed   optional, ms/symbol (default 160)
#
# Refusals:
#   409 Conflict  if irlink already running (any subcommand) — caller should
#                 stop the conflicting process first
#   400 Bad Req   if coords missing or malformed
#
# Deploy: scp -O monocal-trigger.cgi root@dacam2:/var/www/x/
#         ssh dacam2 "chmod 755 /var/www/x/monocal-trigger.cgi"

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

# Parse k=v&k=v form. Body is ASCII; no URL-decoding needed for X,Y.
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

# Validate speed as positive int 40..2000
case "$SPEED" in
    ''|*[!0-9]*) SPEED=160 ;;
esac
if [ "$SPEED" -lt 40 ] 2>/dev/null || [ "$SPEED" -gt 2000 ] 2>/dev/null; then
    SPEED=160
fi

# Refuse if any irlink already running on this cam (busybox pgrep has no -c).
RUNNING=$(pgrep irlink 2>/dev/null | wc -l)
if [ "$RUNNING" -gt 0 ] 2>/dev/null; then
    echo "Status: 409 Conflict"
    echo "Content-Type: application/json"
    echo ""
    echo "{\"ok\":false,\"error\":\"irlink already running\",\"running\":$RUNNING}"
    exit 0
fi

# Pre-write status so the first poll sees "spawning" instead of stale state.
# busybox `date` on Thingino doesn't support %3N — pad seconds*1000 instead.
# The irlink monocal binary overwrites this with real ms granularity within
# a few hundred ms of spawning.
NOW_MS=$(date +%s)000
TMP=/run/monocal-status.json.tmp.$$
cat > "$TMP" <<EOF
{"schema":1,"state":"spawning","started_ms":$NOW_MS,"updated_ms":$NOW_MS,"ended_ms":null,"ok":null,"error":"","ack_received_ms":0,"led_hold_started_ms":0,"led_hold_ended_ms":0,"peer_pixel":null,"speed_ms":$SPEED}
EOF
mv "$TMP" /run/monocal-status.json 2>/dev/null

# Spawn detached. setsid so the process survives the CGI exit.
setsid /opt/bin/irlink monocal --pixel "$COORDS" --speed "$SPEED" \
    > /run/monocal-stdout.log 2>&1 < /dev/null &
PID=$!

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
echo "{\"ok\":true,\"pid\":$PID,\"coords\":\"$COORDS\",\"speed_ms\":$SPEED}"
