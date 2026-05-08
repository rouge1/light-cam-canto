#!/bin/sh
# Thingino CGI: process-running probe.
#
# Returns JSON with the running-instance count for the three processes
# preflight cares about. Phase 4 of the SSH→web migration: replaces the
# bundled SSH `pgrep` calls in aim_assist.preflight() and the post-start
# verify in restart_cam1_daemon().
#
# Note: busybox `pgrep -c <name>` does NOT work on Thingino (unrecognized
# option). Use `pgrep <name> | wc -l` instead. For prudynt-patched we use
# pidof since the binary name has a hyphen and pgrep's name-matching is
# basename-only.
#
# Deploy: scp -O proc-status.cgi root@dacamN:/var/www/x/proc-status.cgi
#         ssh dacamN "chmod 755 /var/www/x/proc-status.cgi"
# Mode MUST be 755 — `chmod +x` lands at 700 under root's default umask, and
# uhttpd serves mode-700 CGIs as 403 Forbidden.

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
echo "{\"irlink\":$IRLINK,\"daynightd\":$DAYNIGHTD,\"prudynt\":$PRUDYNT,\"ts_s\":$TS_S}"
