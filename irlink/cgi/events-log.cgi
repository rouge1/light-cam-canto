#!/bin/sh
# Thingino CGI: serve /tmp/irlink-events.log as plain text.
#
# irlink appends tagged one-line records (STATE / AE / TX / RX) to this log
# from every state transition, AE freeze toggle, send_message call, and
# rx_thread decode. The file self-truncates at 64 KB. Optional ?tail=N
# returns only the last N lines (default: full file).
#
# Deploy: scp -O events-log.cgi root@dacamN:/var/www/x/events-log.cgi
#         ssh dacamN "chmod 755 /var/www/x/events-log.cgi"
# Mode 755 (NOT just +x) — uhttpd serves mode 700 as 403 Forbidden.

echo "Content-Type: text/plain"
echo "Cache-Control: no-cache, no-store"
echo ""

if [ ! -r /tmp/irlink-events.log ]; then
    echo "(no events log — irlink not started yet, or pre-logging build)"
    exit 0
fi

# Parse ?tail=N from QUERY_STRING. Default 0 = full file.
N=0
case "$QUERY_STRING" in
    *tail=*)
        N=$(echo "$QUERY_STRING" | sed -n 's/.*tail=\([0-9]*\).*/\1/p')
        ;;
esac

if [ "$N" -gt 0 ] 2>/dev/null; then
    tail -n "$N" /tmp/irlink-events.log
else
    cat /tmp/irlink-events.log
fi
