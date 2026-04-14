#!/bin/bash
#
# cam_setup.sh — Verify and configure cameras for irlink communication.
#
# Uses the Thingino web API (curl) for ISP/LED/ircut control since CLI
# commands (imp-control, color, daynight) are unreliable.
# Uses SSH only for: mount, process management, brightness files, GPIO reads.
#
# Usage:
#   ./cam_setup.sh                  # setup both cameras
#   ./cam_setup.sh da-camera1       # setup one camera
#   ./cam_setup.sh --check          # verify only, don't fix
#   ./cam_setup.sh --reboot CAM     # reboot and re-check

set -uo pipefail

RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[0;33m'
RST='\033[0m'

CHECK_ONLY=0
REBOOT_CAM=""

# Parse flags
args=()
for arg in "$@"; do
    case "$arg" in
        --check) CHECK_ONLY=1 ;;
        --reboot) REBOOT_CAM="next" ;;
        *)
            if [ "$REBOOT_CAM" = "next" ]; then
                REBOOT_CAM="$arg"
            else
                args+=("$arg")
            fi
            ;;
    esac
done

if [ ${#args[@]} -gt 0 ]; then
    CAMERAS="${args[*]}"
elif [ "$REBOOT_CAM" != "" ] && [ "$REBOOT_CAM" != "next" ]; then
    CAMERAS="$REBOOT_CAM"
else
    CAMERAS="da-camera1 da-camera2"
fi

pass() { echo -e "  ${GRN}✓${RST} $1"; }
fail() { echo -e "  ${RED}✗${RST} $1"; }
warn() { echo -e "  ${YEL}!${RST} $1"; }
info() { echo -e "  ${YEL}→${RST} $1"; }

CAM_PASSWORD="password"

ssh_cmd() {
    ssh -o ConnectTimeout=5 -o BatchMode=yes "$1" "$2" 2>/dev/null
}

# Get camera IP from hostname
cam_ip() {
    case "$1" in
        da-camera1) echo "192.168.50.110" ;;
        da-camera2) echo "192.168.50.141" ;;
        *) ssh_cmd "$1" "hostname -I" | awk '{print $1}' ;;
    esac
}

# Login once per camera, reuse cookie
cam_login() {
    local ip="$1"
    local cookie_file="/tmp/cam_cookie_${ip}.txt"
    local result
    result=$(curl -s -c "$cookie_file" "http://${ip}/x/login.cgi" \
        -H "Content-Type: application/json" \
        -d "{\"username\":\"root\",\"password\":\"${CAM_PASSWORD}\"}" 2>/dev/null)
    echo "$result"
}

# Send command to /x/json-imp.cgi (ISP controls: color, ircut, ir850, ir940, daynight)
imp_cmd() {
    local ip="$1" cmd="$2" val="$3"
    local cookie_file="/tmp/cam_cookie_${ip}.txt"
    curl -s -b "$cookie_file" "http://${ip}/x/json-imp.cgi" \
        -H "Content-Type: application/json" \
        -d "{\"cmd\":\"${cmd}\",\"val\":${val}}" 2>/dev/null
}

# Get fast heartbeat (SSE — color_mode, daynight_mode)
cam_heartbeat_fast() {
    local ip="$1"
    local cookie_file="/tmp/cam_cookie_${ip}.txt"
    # SSE stream — grab first data line and close
    curl -s -b "$cookie_file" --max-time 3 "http://${ip}/x/json-heartbeat.cgi" \
        -H "Accept: application/json" 2>/dev/null | grep '^data:' | head -1 | sed 's/^data: //'
}

# Get slow heartbeat (regular JSON — ircut_state, ir850_state, ir940_state)
cam_heartbeat_slow() {
    local ip="$1"
    local cookie_file="/tmp/cam_cookie_${ip}.txt"
    curl -s -b "$cookie_file" --max-time 3 "http://${ip}/x/json-heartbeat-slow.cgi" \
        -H "Accept: application/json" 2>/dev/null
}

check_cam() {
    local cam="$1"
    local errors=0
    local ip
    ip=$(cam_ip "$cam")

    echo ""
    echo "=== $cam ($ip) ==="
    echo ""

    # 1. SSH connectivity
    if ! ssh_cmd "$cam" "echo ok" >/dev/null 2>&1; then
        fail "SSH connection failed"
        return 1
    fi
    pass "SSH connected"

    # 2. Web API login
    local login_result
    login_result=$(cam_login "$ip")
    if echo "$login_result" | grep -q '"success"'; then
        pass "Web API authenticated"
    else
        fail "Web API login failed: $login_result"
        errors=$((errors + 1))
    fi

    # 3. /opt mounted
    if ssh_cmd "$cam" "mount | grep -q '/opt'"; then
        pass "/opt mounted"
    else
        fail "/opt not mounted"
        if [ $CHECK_ONLY -eq 0 ]; then
            info "Mounting /opt..."
            ssh_cmd "$cam" "mount -t jffs2 /dev/mtdblock5 /opt"
            if ssh_cmd "$cam" "mount | grep -q '/opt'"; then
                pass "/opt mounted (fixed)"
            else
                fail "/opt mount failed"
                errors=$((errors + 1))
            fi
        else
            errors=$((errors + 1))
        fi
    fi

    # 4. Patched prudynt running
    if ssh_cmd "$cam" "pidof prudynt-patched >/dev/null"; then
        pass "prudynt-patched running"
    else
        fail "prudynt-patched not running"
        if [ $CHECK_ONLY -eq 0 ]; then
            info "Starting patched prudynt..."
            ssh_cmd "$cam" "/etc/init.d/S31prudynt stop 2>/dev/null; sleep 2; nohup /opt/bin/prudynt-patched > /tmp/prudynt.log 2>&1 &"
            ssh_cmd "$cam" "sleep 3"
            if ssh_cmd "$cam" "pidof prudynt-patched >/dev/null"; then
                pass "prudynt-patched started (fixed)"
                # Re-login since prudynt just restarted
                cam_login "$ip" >/dev/null
            else
                fail "prudynt-patched failed to start"
                errors=$((errors + 1))
            fi
        else
            errors=$((errors + 1))
        fi
    fi

    # 5. Kill daynightd (before setting night mode, so it doesn't fight us)
    if ssh_cmd "$cam" "! pidof daynightd >/dev/null"; then
        pass "daynightd not running"
    else
        fail "daynightd running (will fight IR settings)"
        if [ $CHECK_ONLY -eq 0 ]; then
            info "Killing daynightd..."
            ssh_cmd "$cam" "killall daynightd 2>/dev/null"
            if ssh_cmd "$cam" "sleep 1; ! pidof daynightd >/dev/null"; then
                pass "daynightd killed (fixed)"
            else
                fail "daynightd still running"
                errors=$((errors + 1))
            fi
        else
            errors=$((errors + 1))
        fi
    fi

    # 6. Night mode via API (monochrome + ircut open)
    #    Then get heartbeat to verify actual ISP state
    if [ $CHECK_ONLY -eq 0 ]; then
        info "Setting night mode via API..."
        local result
        result=$(imp_cmd "$ip" "daynight" "\"night\"")
        if echo "$result" | grep -q '"success"'; then
            pass "Night mode command sent"
        else
            warn "Night mode API returned: $result"
            errors=$((errors + 1))
        fi
    fi

    # 7. Verify ISP state from heartbeat APIs
    local hb_fast hb_slow
    hb_fast=$(cam_heartbeat_fast "$ip")
    hb_slow=$(cam_heartbeat_slow "$ip")

    # 7a. Color mode from fast heartbeat: 0=color, 1=monochrome
    if [ -n "$hb_fast" ]; then
        local color_mode
        color_mode=$(echo "$hb_fast" | grep -o '"color_mode":[0-9]*' | grep -o '[0-9]*$')
        if [ "$color_mode" = "1" ]; then
            pass "Monochrome mode (color_mode=$color_mode)"
        else
            fail "Color mode active (color_mode=$color_mode)"
            if [ $CHECK_ONLY -eq 0 ]; then
                info "Forcing monochrome via API..."
                imp_cmd "$ip" "color" 1 >/dev/null
                pass "Monochrome command sent"
            else
                errors=$((errors + 1))
            fi
        fi
    else
        warn "Could not get fast heartbeat — skipping color check"
    fi

    # 7b. LED states from slow heartbeat
    if [ -n "$hb_slow" ]; then
        local ir850_state ir940_state
        ir850_state=$(echo "$hb_slow" | grep -o '"ir850_state":[0-9]*' | grep -o '[0-9]*$')
        ir940_state=$(echo "$hb_slow" | grep -o '"ir940_state":[0-9]*' | grep -o '[0-9]*$')

        # 850nm LED: must be OFF
        if [ "$ir850_state" = "0" ]; then
            pass "850nm LED off (ir850_state=$ir850_state)"
        else
            fail "850nm LED on (ir850_state=$ir850_state)"
            if [ $CHECK_ONLY -eq 0 ]; then
                info "Turning off 850nm via API..."
                imp_cmd "$ip" "ir850" 0 >/dev/null
                pass "850nm off command sent"
            else
                errors=$((errors + 1))
            fi
        fi

        # 940nm LED: must be OFF (irlink controls it)
        if [ "$ir940_state" = "0" ]; then
            pass "940nm LED off (ir940_state=$ir940_state)"
        else
            fail "940nm LED on (ir940_state=$ir940_state)"
            if [ $CHECK_ONLY -eq 0 ]; then
                info "Turning off 940nm via API..."
                imp_cmd "$ip" "ir940" 0 >/dev/null
                pass "940nm off command sent"
            else
                errors=$((errors + 1))
            fi
        fi
    else
        warn "Could not get slow heartbeat — skipping LED checks"
        errors=$((errors + 1))
    fi

    # 8. Verify GPIO matches API state (belt and suspenders)
    local gpio47 gpio49
    gpio47=$(ssh_cmd "$cam" "gpio read 47 2>/dev/null" || echo "unknown")
    gpio49=$(ssh_cmd "$cam" "gpio read 49 2>/dev/null" || echo "unknown")
    if [ "$gpio47" = "0" ]; then
        pass "GPIO 47 (850nm) confirmed off"
    else
        warn "GPIO 47 reads $gpio47 — forcing off"
        ssh_cmd "$cam" "gpio clear 47"
    fi
    if [ "$gpio49" = "0" ]; then
        pass "GPIO 49 (940nm) confirmed off"
    else
        warn "GPIO 49 reads $gpio49 — forcing off"
        ssh_cmd "$cam" "gpio clear 49"
    fi

    # 9. Brightness grid available
    local grid_len
    grid_len=$(ssh_cmd "$cam" "wc -c < /run/prudynt/brightness_grid 2>/dev/null" || echo "0")
    grid_len=$(echo "$grid_len" | tr -d '[:space:]')
    if [ "$grid_len" -gt 10 ] 2>/dev/null; then
        pass "Brightness grid active ($grid_len bytes)"
    else
        fail "Brightness grid missing or empty"
        errors=$((errors + 1))
    fi

    # 10. AE freeze file
    local ae_val
    ae_val=$(ssh_cmd "$cam" "cat /run/prudynt/ae_freeze 2>/dev/null" || echo "MISSING")
    if [ "$ae_val" = "MISSING" ]; then
        fail "ae_freeze file missing"
        if [ $CHECK_ONLY -eq 0 ]; then
            info "Creating ae_freeze file..."
            ssh_cmd "$cam" "echo 0 > /run/prudynt/ae_freeze"
            ae_val=$(ssh_cmd "$cam" "cat /run/prudynt/ae_freeze 2>/dev/null" || echo "MISSING")
            if [ "$ae_val" != "MISSING" ]; then
                pass "ae_freeze file created (set to $ae_val)"
            else
                fail "ae_freeze file still missing"
                errors=$((errors + 1))
            fi
        else
            errors=$((errors + 1))
        fi
    elif [ "$ae_val" = "0" ]; then
        pass "AE freeze idle (ae_freeze=0)"
    else
        warn "AE freeze stuck at $ae_val (should be 0 when idle)"
        if [ $CHECK_ONLY -eq 0 ]; then
            info "Resetting ae_freeze to 0..."
            ssh_cmd "$cam" "echo 0 > /run/prudynt/ae_freeze"
            pass "AE freeze reset to 0"
        else
            errors=$((errors + 1))
        fi
    fi

    # 11. irlink binary present
    if ssh_cmd "$cam" "test -x /opt/bin/irlink"; then
        local irlink_date
        irlink_date=$(ssh_cmd "$cam" "ls -la /opt/bin/irlink" | awk '{print $6, $7, $8}')
        pass "irlink binary present ($irlink_date)"
    else
        fail "irlink binary missing from /opt/bin/"
        errors=$((errors + 1))
    fi

    # 12. IR cut filter removed — LAST step (daynight/color commands may re-close it)
    #     ircut_state: 0=removed(night/IR passes), 1=set(day/IR blocked)
    local hb_slow2
    hb_slow2=$(cam_heartbeat_slow "$ip")
    local ircut_state
    ircut_state=$(echo "$hb_slow2" | grep -o '"ircut_state":[0-9]*' | grep -o '[0-9]*$')
    if [ "$ircut_state" = "0" ]; then
        pass "IR cut filter removed (ircut_state=$ircut_state)"
    else
        fail "IR cut filter set/blocking IR (ircut_state=$ircut_state)"
        if [ $CHECK_ONLY -eq 0 ]; then
            info "Removing IR cut filter via API (final step)..."
            imp_cmd "$ip" "ircut" 0 >/dev/null
            # Verify it stuck
            hb_slow2=$(cam_heartbeat_slow "$ip")
            ircut_state=$(echo "$hb_slow2" | grep -o '"ircut_state":[0-9]*' | grep -o '[0-9]*$')
            if [ "$ircut_state" = "0" ]; then
                pass "IR cut filter removed (verified)"
            else
                fail "IR cut filter won't stay removed"
                errors=$((errors + 1))
            fi
        else
            errors=$((errors + 1))
        fi
    fi

    echo ""
    if [ $errors -eq 0 ]; then
        echo -e "  ${GRN}All checks passed for $cam${RST}"
    else
        echo -e "  ${RED}$errors issue(s) remaining on $cam${RST}"
    fi

    return $errors
}

# Handle reboot request
if [ "$REBOOT_CAM" != "" ] && [ "$REBOOT_CAM" != "next" ]; then
    echo "Rebooting $REBOOT_CAM..."
    ssh_cmd "$REBOOT_CAM" "reboot" || true
    echo "Waiting 60s for reboot..."
    sleep 60
    echo "Re-checking..."
    check_cam "$REBOOT_CAM"
    exit $?
fi

echo "==============================="
echo " Camera Setup for irlink"
echo "==============================="
if [ $CHECK_ONLY -eq 1 ]; then
    echo " (check-only mode)"
fi

total_errors=0
for cam in $CAMERAS; do
    if ! check_cam "$cam"; then
        total_errors=$((total_errors + 1))
    fi
done

echo ""
echo "==============================="
if [ $total_errors -eq 0 ]; then
    echo -e " ${GRN}All cameras ready for irlink${RST}"
else
    echo -e " ${RED}$total_errors camera(s) have issues${RST}"
    echo " Try: $0 --reboot <camera>"
fi
echo "==============================="
