#!/bin/sh
# Thingino CGI: serve cam2's monocal status JSON.
#
# irlink's `monocal` subcommand writes /run/monocal-status.json atomically
# (write-then-rename) at every state transition: spawning → handshaking →
# monocal_req → monocal_ack_wait → monocal_holding_off → monocal_holding_on
# → monocal_awaiting_done → monocal_done | monocal_failed.
#
# GET → the JSON file verbatim. Two soft-fail wrappers per the planning:
#   missing file  → {"state":"never_run", ...}, HTTP 200
#   empty file    → HTTP 503 (mid-rename window — caller retries next tick)
#
# Deploy: scp -O monocal-status.cgi root@dacam2:/var/www/x/
#         ssh dacam2 "chmod 755 /var/www/x/monocal-status.cgi"

STATUS_PATH=/run/monocal-status.json

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"

if [ ! -f "$STATUS_PATH" ]; then
    echo ""
    echo '{"schema":1,"state":"never_run","ok":null}'
    exit 0
fi

# Detect mid-rename: file present but empty.
SIZE=$(wc -c < "$STATUS_PATH" 2>/dev/null)
if [ -z "$SIZE" ] || [ "$SIZE" -lt 2 ] 2>/dev/null; then
    # Use full status line for non-uhttpd shells; uhttpd reads the first line.
    echo "Status: 503 Service Unavailable"
    echo ""
    echo '{"schema":1,"state":"transient","ok":null}'
    exit 0
fi

echo ""
cat "$STATUS_PATH"
