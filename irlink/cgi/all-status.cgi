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
#     "proc":   {"irlink": N, "daynightd": N, "prudynt": 0|1, "ts_s": N},
#     "vitals": {"uptime_s": N, "load": {l1,l5,l15},
#                "mem_kb": {total,free,available},
#                "disk_pct": {overlay,opt,tmp}, "dmesg_errors": N}
#   }
#
# The vitals fields are folded in (was a separate /x/health.cgi) so the
# live webui poll cycle and /api/{cam}/health both read everything from
# one CGI fork. Cuts cam-side CGI invocations per health check from 2 → 1
# and lets the laptop's piggyback-cache pattern serve health calls at
# zero cost when the live poller has data fresh in cache.
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

# 5. Vitals — same fields the standalone /x/health.cgi used to emit.
#    Folding them in here is the "less congestion" optimization: one fork
#    serves both the live poll cycle and any health probe.
UPTIME_S=$(awk '{print int($1)}' /proc/uptime 2>/dev/null)
[ -z "$UPTIME_S" ] && UPTIME_S=0
LOAD_LINE=$(cat /proc/loadavg 2>/dev/null)
L1=$(echo "$LOAD_LINE"  | awk '{print $1}'); [ -z "$L1" ]  && L1=0
L5=$(echo "$LOAD_LINE"  | awk '{print $2}'); [ -z "$L5" ]  && L5=0
L15=$(echo "$LOAD_LINE" | awk '{print $3}'); [ -z "$L15" ] && L15=0
MEM_TOTAL=$(awk '/^MemTotal:/   {print $2}' /proc/meminfo 2>/dev/null); [ -z "$MEM_TOTAL" ] && MEM_TOTAL=0
MEM_FREE=$(awk  '/^MemFree:/    {print $2}' /proc/meminfo 2>/dev/null); [ -z "$MEM_FREE" ]  && MEM_FREE=0
MEM_AVAIL=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null); [ -z "$MEM_AVAIL" ] && MEM_AVAIL=$MEM_FREE
disk_pct() {
    df -P -k "$1" 2>/dev/null | awk 'NR==2 {gsub("%","",$5); print $5+0}'
}
DISK_OVERLAY=$(disk_pct /overlay); [ -z "$DISK_OVERLAY" ] && DISK_OVERLAY=0
DISK_OPT=$(disk_pct /opt);         [ -z "$DISK_OPT" ]     && DISK_OPT=0
DISK_TMP=$(disk_pct /tmp);         [ -z "$DISK_TMP" ]     && DISK_TMP=0
# dmesg counter — exclude noisy atbm_log lines (WiFi key rotation).
DMESG_ERR=$(dmesg 2>/dev/null | grep -v atbm_log | grep -ciE 'error|fail|panic|oops' | tr -d ' ')
[ -z "$DMESG_ERR" ] && DMESG_ERR=0

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
printf '{"status":%s,"leds":{"ir850":%s,"ir940":%s},"ae":%s,"proc":{"irlink":%s,"daynightd":%s,"prudynt":%s,"ts_s":%s},"vitals":{"uptime_s":%s,"load":{"l1":%s,"l5":%s,"l15":%s},"mem_kb":{"total":%s,"free":%s,"available":%s},"disk_pct":{"overlay":%s,"opt":%s,"tmp":%s},"dmesg_errors":%s}}\n' \
    "$STATUS" "$GPIO47" "$GPIO49" "$AE" \
    "$IRLINK" "$DAYNIGHTD" "$PRUDYNT" "$TS_S" \
    "$UPTIME_S" "$L1" "$L5" "$L15" \
    "$MEM_TOTAL" "$MEM_FREE" "$MEM_AVAIL" \
    "$DISK_OVERLAY" "$DISK_OPT" "$DISK_TMP" "$DMESG_ERR"
