#!/bin/sh
# Thingino CGI: serve /run/grid-deltas.json — last on-camera grid scan
# result emitted by `do_grid_calibration`.
#
# Body shape:
#   {
#     "ts_ms":      <int>,            // when the scan finished
#     "best_block": <0..239>,         // argmax pick (post-OSD-mask)
#     "best_delta": <int>,            // its delta value
#     "deltas":     [<240 ints>]      // row-major, post-OSD-mask
#   }
#
# Returned 404 with `{"state":"never_run"}` if the file doesn't exist
# (cam hasn't run a grid cal yet since boot — `/run` is tmpfs).
#
# Used by host/cam_status.CamStatusClient.get_grid_deltas() and the webui's
# `/api/{cam}/grid-deltas` endpoint to debug "monocal landed at the wrong
# block" without SSHing.
#
# Deploy: scp -O grid-deltas.cgi root@dacamN:/var/www/x/grid-deltas.cgi
#         ssh dacamN "chmod 755 /var/www/x/grid-deltas.cgi"

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"

if [ -r /run/grid-deltas.json ]; then
    echo ""
    cat /run/grid-deltas.json
else
    echo "Status: 404 Not Found"
    echo ""
    echo '{"state":"never_run"}'
fi
