#!/bin/sh
# Thingino CGI: serve irlink's status JSON.
#
# irlink writes /run/irlink-status.json atomically (write-then-rename) at
# every protocol event — handshake, cal phase, rate change, error. The
# laptop polls this CGI instead of SSHing into the camera, which avoids
# the per-poll sshd login overhead that was overloading cam2 during cal.
#
# Deploy: scp -O cal-status.cgi root@dacamN:/var/www/x/cal-status.cgi
#         ssh dacamN "chmod +x /var/www/x/cal-status.cgi"
#
# Auth note: Thingino's webserver may serve /x/ paths with cookie auth.
# If unauthed access is rejected, the laptop must log in via /x/login.cgi
# and pass the cookie back (cam_login() pattern in host/cam_setup.sh).

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""

if [ -r /run/irlink-status.json ]; then
    cat /run/irlink-status.json
else
    echo '{"error":"no status file","irlink_running":'$(pgrep -c irlink 2>/dev/null || echo 0)'}'
fi
