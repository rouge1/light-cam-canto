#!/bin/sh
# Thingino CGI: put this cam into a known IR-comm-ready state.
#
# All the writes that the laptop's host/cam_setup.sh used to do over many
# round-trips, bundled into one cam-side script. Returns a JSON report:
#   {ok: true|false, steps: [{name, ok, detail}, ...]}
#
# Bundles:
#   - kill daynightd (so it doesn't fight subsequent ISP writes)
#   - prudyntctl: disable daynight loop, force night mode, force mono image
#   - LEDs off via `light` AND raw `gpio clear` (belt + suspenders — sometimes
#     daynightd's model says off but GPIO is still high)
#   - ae_freeze = 0 (idle)
#   - ircut LAST (filter removed; other ISP writes can re-set it)
#   - read-back checks: gpio levels, ae_freeze, brightness_grid alive,
#     prudynt-patched running, irlink binary present
#
# Deploy: scp -O setup.cgi root@dacamN:/var/www/x/setup.cgi
#         ssh dacamN "chmod 755 /var/www/x/setup.cgi"
#
# POST only — even with no body, refuse GETs since this changes ISP state.

if [ "$REQUEST_METHOD" != "POST" ]; then
    echo "Status: 405 Method Not Allowed"
    echo "Content-Type: application/json"
    echo ""
    echo '{"ok":false,"error":"POST only"}'
    exit 0
fi

RESULTS=""
ALL_OK=1

# record <name> <ok 0|1> <detail>   — appends one step to the JSON output.
# Detail is plain text, no quotes — keep it short, no embedded backslashes.
record() {
    if [ -n "$RESULTS" ]; then RESULTS="$RESULTS,"; fi
    RESULTS="$RESULTS{\"name\":\"$1\",\"ok\":$2,\"detail\":\"$3\"}"
    [ "$2" = "0" ] && ALL_OK=0
}

# 1. Kill daynightd if running. It periodically rewrites ircut/LEDs based on
#    its own ambient model and will undo our writes a second later.
if pidof daynightd >/dev/null 2>&1; then
    killall daynightd 2>/dev/null
    sleep 1
    if pidof daynightd >/dev/null 2>&1; then
        record "daynightd-stopped" 0 "still running after kill"
    else
        record "daynightd-stopped" 1 "killed"
    fi
else
    record "daynightd-stopped" 1 "already stopped"
fi

# 2. Verify prudynt-patched is up. Can't restart it from inside a CGI it's
#    serving — fail loud so the operator runs the SSH recovery path.
if pidof prudynt-patched >/dev/null 2>&1; then
    record "prudynt-running" 1 "ok"
else
    record "prudynt-running" 0 "not running (manual SSH fix needed)"
fi

# 3. Disable prudynt's daynight loop, then force night mode. Mirrors the
#    json-imp.cgi sequence so behavior matches what laptop callers had.
echo '{"daynight":{"enabled":false}}' | prudyntctl json - >/dev/null 2>&1
if echo '{"daynight":{"enabled":false,"force_mode":"night"}}' \
        | prudyntctl json - >/dev/null 2>&1; then
    record "set-night" 1 "ok"
else
    record "set-night" 0 "prudyntctl failed"
fi

# 4. Force monochrome (running_mode=1). prudyntctl is the only knob; the
#    `imp-control ispmode` path would also work but stays out of sync with
#    json-imp.cgi conventions.
if echo '{"daynight":{"enabled":false},"image":{"running_mode":1}}' \
        | prudyntctl json - >/dev/null 2>&1; then
    record "set-mono" 1 "ok"
else
    record "set-mono" 0 "prudyntctl failed"
fi

# 5. LEDs off via `light` AND `gpio clear` — see header comment.
light ir850 0 >/dev/null 2>&1
light ir940 0 >/dev/null 2>&1
gpio clear 47 2>/dev/null
gpio clear 49 2>/dev/null
G47=$(gpio read 47 2>/dev/null || echo X)
G49=$(gpio read 49 2>/dev/null || echo X)
if [ "$G47" = "0" ]; then record "ir850-off" 1 "gpio47=0"
else record "ir850-off" 0 "gpio47=$G47"; fi
if [ "$G49" = "0" ]; then record "ir940-off" 1 "gpio49=0"
else record "ir940-off" 0 "gpio49=$G49"; fi

# 6. AE freeze idle (=0). Stuck-at-1 happens after `killall -9 irlink` (skips
#    SIGTERM cleanup); resetting here is the same fix cam_setup.sh applied.
echo 0 > /run/prudynt/ae_freeze 2>/dev/null
AE=$(cat /run/prudynt/ae_freeze 2>/dev/null || echo MISSING)
if [ "$AE" = "0" ]; then record "ae-freeze-0" 1 "ok"
else record "ae-freeze-0" 0 "is $AE"; fi

# 7. Brightness grid is being produced by prudynt's BrightnessMonitor.
GRID_LEN=$(wc -c < /run/prudynt/brightness_grid 2>/dev/null || echo 0)
GRID_LEN=$(echo "$GRID_LEN" | tr -d '[:space:]')
if [ "$GRID_LEN" -gt 10 ] 2>/dev/null; then
    record "grid-active" 1 "${GRID_LEN}B"
else
    record "grid-active" 0 "missing or empty"
fi

# 8. irlink binary present (sanity for the operator).
if [ -x /opt/bin/irlink ]; then
    record "irlink-present" 1 "ok"
else
    record "irlink-present" 0 "missing from /opt/bin/"
fi

# 9. ircut LAST. `ircut off` = filter REMOVED (IR passes through, night mode).
#    Other ISP writes above can re-close it, so this has to be the final step.
if ircut off >/dev/null 2>&1; then
    record "ircut-open" 1 "ok"
else
    record "ircut-open" 0 "ircut returned non-zero"
fi

echo "Content-Type: application/json"
echo "Cache-Control: no-cache, no-store"
echo ""
echo "{\"ok\":$([ "$ALL_OK" = "1" ] && echo true || echo false),\"steps\":[$RESULTS]}"
