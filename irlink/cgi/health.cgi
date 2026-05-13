#!/bin/sh
# Thingino CGI: cam system vitals — the stuff not covered by other CGIs.
#
# Emits JSON with uptime, load avg, memory, disk usage for overlay/opt/tmp,
# and a count of error-class lines in dmesg. Designed to be cheap (a few
# /proc reads + one `dmesg` + three `df`s) — runs in well under 100 ms on
# the slow MIPS even under load.
#
# Other CGIs cover the rest of the per-cam health picture:
#   - /x/cal-status.cgi   irlink state + counters
#   - /x/proc-status.cgi  irlink / daynightd / prudynt running?
#   - /x/ae-freeze.cgi    AE freeze flag
#   - /x/gpio-state.cgi   IR LED GPIO levels
# Or call /x/all-status.cgi for those four bundled.
#
# Deploy: scp -O health.cgi root@dacamN:/var/www/x/health.cgi
#         ssh dacamN "chmod 755 /var/www/x/health.cgi"
# Mode MUST be 755 — uhttpd serves mode 700 as 403 Forbidden.

# Uptime — integer seconds (busybox /proc/uptime emits "12345.67 5432.10").
UPTIME_S=$(awk '{print int($1)}' /proc/uptime 2>/dev/null)
[ -z "$UPTIME_S" ] && UPTIME_S=0

# Load averages — three space-separated floats. Pass through as JSON
# numbers (no quoting in the printf format below).
LOAD_LINE=$(cat /proc/loadavg 2>/dev/null)
L1=$(echo "$LOAD_LINE"  | awk '{print $1}')
L5=$(echo "$LOAD_LINE"  | awk '{print $2}')
L15=$(echo "$LOAD_LINE" | awk '{print $3}')
[ -z "$L1" ]  && L1=0
[ -z "$L5" ]  && L5=0
[ -z "$L15" ] && L15=0

# Memory in kB. Some kernels omit MemAvailable; fall back to MemFree.
MEM_TOTAL=$(awk '/^MemTotal:/   {print $2}' /proc/meminfo 2>/dev/null)
MEM_FREE=$(awk  '/^MemFree:/    {print $2}' /proc/meminfo 2>/dev/null)
MEM_AVAIL=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null)
[ -z "$MEM_TOTAL" ] && MEM_TOTAL=0
[ -z "$MEM_FREE" ]  && MEM_FREE=0
[ -z "$MEM_AVAIL" ] && MEM_AVAIL=$MEM_FREE

# Disk usage percent. `df -P -k` is the portable POSIX form busybox
# implements. `gsub("%","",$5)` strips the trailing percent sign so the
# value lands in JSON as a bare number.
disk_pct() {
    df -P -k "$1" 2>/dev/null | awk 'NR==2 {gsub("%","",$5); print $5+0}'
}
DISK_OVERLAY=$(disk_pct /overlay)
DISK_OPT=$(disk_pct /opt)
DISK_TMP=$(disk_pct /tmp)
[ -z "$DISK_OVERLAY" ] && DISK_OVERLAY=0
[ -z "$DISK_OPT" ]     && DISK_OPT=0
[ -z "$DISK_TMP" ]     && DISK_TMP=0

# Count error-class dmesg lines. WiFi key-rotate messages dominate the
# log and we don't want to flag them — exclude atbm_log (the wifi chip
# driver's heartbeat) before grepping for error|fail|panic|oops.
DMESG_ERR=$(dmesg 2>/dev/null | grep -v atbm_log | grep -ciE 'error|fail|panic|oops' | tr -d ' ')
[ -z "$DMESG_ERR" ] && DMESG_ERR=0

TS_S=$(date +%s)

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
printf '{"uptime_s":%s,"load":{"l1":%s,"l5":%s,"l15":%s},"mem_kb":{"total":%s,"free":%s,"available":%s},"disk_pct":{"overlay":%s,"opt":%s,"tmp":%s},"dmesg_errors":%s,"ts_s":%s}\n' \
    "$UPTIME_S" "$L1" "$L5" "$L15" "$MEM_TOTAL" "$MEM_FREE" "$MEM_AVAIL" "$DISK_OVERLAY" "$DISK_OPT" "$DISK_TMP" "$DMESG_ERR" "$TS_S"
