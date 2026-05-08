#!/bin/sh
# Thingino CGI: read or write the AE-freeze state.
#
# patched prudynt-t writes a single character ('0' or '1') into
# /run/prudynt/ae_freeze to track whether IMP_ISP_Tuning_SetAeFreeze is
# currently engaged.
#
# GET  → plain-text "0\n" or "1\n" (Phase 2)
# POST value=0|1  → atomic write, returns {"ok":true,"value":N} JSON (Phase 3)
# POST anything else  → 400 Bad Request, file untouched
#
# Deploy: scp -O ae-freeze.cgi root@dacamN:/var/www/x/ae-freeze.cgi
#         ssh dacamN "chmod 755 /var/www/x/ae-freeze.cgi"
# Mode MUST be 755 — `chmod +x` lands at 700 under root's default umask, and
# uhttpd serves mode-700 CGIs as 403 Forbidden.

AE_PATH=/run/prudynt/ae_freeze

if [ "$REQUEST_METHOD" = "POST" ]; then
    # Read the body (cap length so a hostile client can't drown us).
    LEN="${CONTENT_LENGTH:-0}"
    if [ "$LEN" -gt 32 ] 2>/dev/null; then
        LEN=32
    fi
    if [ "$LEN" -gt 0 ] 2>/dev/null; then
        BODY=$(head -c "$LEN")
    else
        BODY=""
    fi

    case "$BODY" in
        value=0) NEW=0 ;;
        value=1) NEW=1 ;;
        *)
            echo "Status: 400 Bad Request"
            echo "Content-Type: application/json"
            echo ""
            echo '{"ok":false,"error":"value must be 0 or 1"}'
            exit 0
            ;;
    esac

    # Atomic write: temp + rename. /run/prudynt is tmpfs; rename is atomic
    # within the same filesystem. $$ scopes the temp to this CGI invocation
    # so concurrent POSTs don't collide.
    TMP="${AE_PATH}.tmp.$$"
    if printf '%s' "$NEW" > "$TMP" 2>/dev/null && mv "$TMP" "$AE_PATH" 2>/dev/null; then
        echo "Content-Type: application/json"
        echo "Cache-Control: no-cache, no-store"
        echo ""
        echo "{\"ok\":true,\"value\":$NEW}"
    else
        rm -f "$TMP" 2>/dev/null
        echo "Status: 500 Internal Server Error"
        echo "Content-Type: application/json"
        echo ""
        echo '{"ok":false,"error":"write failed"}'
    fi
    exit 0
fi

# GET — plain text (Phase 2 contract).
echo "Content-Type: text/plain"
echo "Cache-Control: no-cache, no-store"
echo ""
if [ -r "$AE_PATH" ]; then
    cat "$AE_PATH"
else
    echo "error: no ae_freeze file"
fi
