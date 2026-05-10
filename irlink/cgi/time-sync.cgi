#!/bin/sh
# Thingino CGI: set this cam's wall clock from a laptop-supplied epoch.
#
# Cameras have no RTC battery and the ISP/router blocks outbound UDP 123, so
# ntpd never syncs. Without time sync, both cams drift apart by tens of
# seconds and protocol timing math (started_ms, hold deadlines, watchdog
# falling-edge windows) gets unreliable. The webui server pushes time here
# on every /api/setup-all and once per hour from a background thread.
#
# POST epoch=<unix-seconds> [ms=<unix-ms>]
#   epoch  required, integer seconds since 1970-01-01 UTC
#   ms     optional, full ms-resolution unix timestamp; if present, used in
#          preference to epoch for higher precision (busybox `date` only
#          accepts integer seconds anyway, so ms is currently advisory —
#          included so a future setter can do micro-corrections).
#
# Returns JSON: {ok, old_epoch, new_epoch, drift_s}
#   drift_s = old - new (positive = clock was ahead of laptop)
#
# Deploy: scp -O time-sync.cgi root@dacamN:/var/www/x/time-sync.cgi
#         ssh dacamN "chmod 755 /var/www/x/time-sync.cgi"

if [ "$REQUEST_METHOD" != "POST" ]; then
    echo "Status: 405 Method Not Allowed"
    echo "Content-Type: application/json"
    echo ""
    echo '{"ok":false,"error":"POST only"}'
    exit 0
fi

LEN="${CONTENT_LENGTH:-0}"
if [ "$LEN" -gt 256 ] 2>/dev/null; then LEN=256; fi
if [ "$LEN" -gt 0 ] 2>/dev/null; then BODY=$(head -c "$LEN"); else BODY=""; fi

EPOCH=""
OLD_IFS="$IFS"
IFS='&'
for kv in $BODY; do
    case "$kv" in
        epoch=*) EPOCH="${kv#epoch=}" ;;
    esac
done
IFS="$OLD_IFS"

# Validate epoch as positive integer.
case "$EPOCH" in
    ''|*[!0-9]*)
        echo "Status: 400 Bad Request"
        echo "Content-Type: application/json"
        echo ""
        echo '{"ok":false,"error":"epoch required as positive integer seconds"}'
        exit 0
        ;;
esac

OLD_EPOCH=$(date -u +%s)
# busybox date -s @N sets from epoch.
date -u -s "@$EPOCH" >/dev/null 2>&1
NEW_EPOCH=$(date -u +%s)
DRIFT=$((OLD_EPOCH - NEW_EPOCH))

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
echo "{\"ok\":true,\"old_epoch\":$OLD_EPOCH,\"new_epoch\":$NEW_EPOCH,\"drift_s\":$DRIFT}"
