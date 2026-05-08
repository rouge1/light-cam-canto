#!/bin/sh
# Thingino CGI: real-time IR LED GPIO state.
#
# Returns JSON with the current pin levels of the IR LED control GPIOs:
#   GPIO 47 = ir850 (visible red glow)
#   GPIO 49 = ir940 (PWM channel 0, invisible)
#
# Why this exists: /x/json-heartbeat-slow.cgi reports an internal model
# state (driven by daynightd) that doesn't sync with manual `gpio set`
# / `imp_cmd ir850/ir940` writes. This CGI reads the actual pin level via
# `gpio read`, so the laptop UI can reflect real LED state during cal.
#
# Deploy: scp -O gpio-state.cgi root@dacamN:/var/www/x/gpio-state.cgi
#         ssh dacamN "chmod 755 /var/www/x/gpio-state.cgi"

GPIO47=$(gpio read 47 2>/dev/null || echo 0)
GPIO49=$(gpio read 49 2>/dev/null || echo 0)

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
echo "{\"ir850\":${GPIO47},\"ir940\":${GPIO49}}"
