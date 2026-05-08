#!/bin/sh
# Thingino CGI: serve prudynt-patched's brightness grid.
#
# BrightnessMonitor in patched prudynt writes /run/prudynt/brightness_grid
# every frame: "<ts_ms> <b0> <b1> ... <b239>\n" (242 space-separated ints,
# 20×12 grid). The laptop's aim_assist.read_grid() polls this at 4 Hz; SSH
# polling at that rate fans out to ~12 sshd logins/sec on flaky WiFi (3×
# retries each), which has documented overload behavior on cam2 during cal.
#
# Deploy: scp -O brightness-grid.cgi root@dacamN:/var/www/x/brightness-grid.cgi
#         ssh dacamN "chmod 755 /var/www/x/brightness-grid.cgi"
# Mode MUST be 755 — `chmod +x` lands at 700 under root's default umask, and
# uhttpd serves mode-700 CGIs as 403 Forbidden.
#
# Auth follows cal-status.cgi convention — Thingino's webserver may require
# cookie auth on /x/ paths; the laptop client logs in via /x/login.cgi and
# replays the cookie.

echo "Content-Type: text/plain"
echo "Cache-Control: no-cache, no-store"
echo ""

if [ -r /run/prudynt/brightness_grid ]; then
    cat /run/prudynt/brightness_grid
else
    echo "error: no brightness_grid file (prudynt-patched not running?)"
fi
