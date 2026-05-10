#!/bin/sh
# Thingino CGI: serve /opt/etc/calibration.json (the persistent saved cal).
#
# This is the authoritative "where does this cam see its peer?" record —
# updated on every successful cal flow (host-driven cal_procedure,
# on-camera calibrate-pixel, monocal scan, bootstrap fallback search).
# Survives reboots (JFFS2-persistent).
#
# Distinct from /x/cal-status.cgi (which serves /run/irlink-status.json,
# the live runtime status that may not have peak/delta data after a
# protocol-path monocal where this cam was the holder, not the scanner).
#
# Deploy: scp -O saved-cal.cgi root@dacamN:/var/www/x/saved-cal.cgi
#         ssh dacamN "chmod 755 /var/www/x/saved-cal.cgi"

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""

if [ -r /opt/etc/calibration.json ]; then
    cat /opt/etc/calibration.json
else
    echo '{"error":"no calibration.json"}'
fi
