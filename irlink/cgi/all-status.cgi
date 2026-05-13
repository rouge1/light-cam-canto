#!/bin/sh
# Thingino CGI: single-fork consolidated read of everything the live
# webui polls per cycle. Replaces 4 concurrent /x/*.cgi hits with one,
# eliminating uhttpd serialization contention on cam1 (single-process
# busybox uhttpd) that caused the OFFLINE flicker.
#
# Emits one JSON object with the same field shapes the individual
# endpoints used to return, so the webui can demux without translation:
#   {
#     "status": <contents of /run/irlink-status.json>,
#     "leds":   {"ir850": 0|1, "ir940": 0|1},
#     "ae":     0|1,
#     "proc":   {"irlink": N, "daynightd": N, "prudynt": 0|1, "ts_s": N}
#   }
#
# Deploy: scp -O all-status.cgi root@dacamN:/var/www/x/all-status.cgi
#         ssh dacamN "chmod 755 /var/www/x/all-status.cgi"
# Mode MUST be 755 — uhttpd serves mode 700 as 403 Forbidden.

# 1. irlink status JSON (may be missing if irlink hasn't written yet)
if [ -r /run/irlink-status.json ]; then
    STATUS=$(cat /run/irlink-status.json)
else
    IRLINK_RUNNING=$(pgrep irlink 2>/dev/null | wc -l | tr -d ' ')
    STATUS="{\"error\":\"no status file\",\"irlink_running\":$IRLINK_RUNNING}"
fi

# 2. IR LED GPIOs — actual pin levels, not daynightd's model
GPIO47=$(gpio read 47 2>/dev/null || echo 0)
GPIO49=$(gpio read 49 2>/dev/null || echo 0)

# 3. AE freeze
if [ -r /run/prudynt/ae_freeze ]; then
    AE=$(cat /run/prudynt/ae_freeze)
else
    AE=0
fi

# 4. Process counts (busybox pgrep doesn't support -c)
count_proc() {
    pgrep "$1" 2>/dev/null | wc -l | tr -d ' '
}
IRLINK=$(count_proc irlink)
DAYNIGHTD=$(count_proc daynightd)
if pidof prudynt-patched >/dev/null 2>&1; then
    PRUDYNT=1
else
    PRUDYNT=0
fi
TS_S=$(date +%s)

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
printf '{"status":%s,"leds":{"ir850":%s,"ir940":%s},"ae":%s,"proc":{"irlink":%s,"daynightd":%s,"prudynt":%s,"ts_s":%s}}\n' \
    "$STATUS" "$GPIO47" "$GPIO49" "$AE" "$IRLINK" "$DAYNIGHTD" "$PRUDYNT" "$TS_S"
