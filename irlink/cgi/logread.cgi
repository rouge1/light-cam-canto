#!/bin/sh
# Thingino CGI: serve filtered busybox syslog ring buffer as plain text.
#
# busybox `logread` dumps the kernel/syslog ring buffer that captures
# everything the system writes via syslog(3) — in our setup that's
# rc.local (Mounted /opt, ISP set, "final LED clear: ir850 1->0 ir940 1->0",
# etc), prudynt's debug output, S94rc.local boot script, and any logger -t
# call. Distinct from /tmp/irlink-events.log which only carries irlink's
# own log_event() output.
#
# Query params:
#   ?tag=<substring>    grep filter on the syslog record (e.g. tag=rc.local
#                       to see only boot diagnostics). Treated as a
#                       fixed-string substring match for safety.
#   ?tail=<N>           return only the last N matching lines (default 50)
#
# Deploy: scp -O logread.cgi root@dacamN:/var/www/x/logread.cgi
#         ssh dacamN "chmod 755 /var/www/x/logread.cgi"
# Mode 755 (NOT just +x) — uhttpd serves mode 700 as 403 Forbidden.

echo "Content-Type: text/plain"
echo "Cache-Control: no-cache, no-store"
echo ""

# Parse ?tag and ?tail from QUERY_STRING. busybox sed is fine with these
# patterns — keep it simple, single-key extraction per param.
TAG=""
N=50
case "$QUERY_STRING" in
    *tag=*)
        # Strip everything before tag= and everything after the next & (if any).
        # Replace common URL-encoded chars by hand — busybox has no decoder.
        TAG=$(echo "$QUERY_STRING" \
              | sed -n 's/.*tag=\([^&]*\).*/\1/p' \
              | sed 's/+/ /g
                     s/%20/ /g
                     s/%2[Ee]/./g
                     s/%2[Dd]/-/g
                     s/%5[Ff]/_/g
                     s/%3[Aa]/:/g
                     s/%2[Ff]/\//g
                     s/%5[Bb]/[/g
                     s/%5[Dd]/]/g')
        ;;
esac
case "$QUERY_STRING" in
    *tail=*)
        N=$(echo "$QUERY_STRING" | sed -n 's/.*tail=\([0-9]*\).*/\1/p')
        ;;
esac
[ -z "$N" ] && N=50

# Safety: refuse multi-line / shell-meta tags. busybox grep accepts most
# strings as fixed-string with -F, but we still drop anything that looks
# like injection so a malformed query can't fork extra shells.
case "$TAG" in
    *[\"\'\`\$\\]*|*$'\n'*) TAG="" ;;
esac

if [ -n "$TAG" ]; then
    logread 2>/dev/null | grep -F "$TAG" | tail -n "$N"
else
    logread 2>/dev/null | tail -n "$N"
fi
