THINGINO_OUTPUT = ../thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl
CROSS_CC = $(THINGINO_OUTPUT)/host/bin/mipsel-linux-gcc
CFLAGS = -Os -Wall -Wextra -pthread
LDFLAGS = -lrt -pthread

irlink: irlink.c
	$(CROSS_CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

rs_toggle_test: rs_toggle_test.c
	$(CROSS_CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

irlink_rs_tx: irlink_rs_tx.c
	$(CROSS_CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

irlink_rs_rx: irlink_rs_rx.c
	$(CROSS_CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

.PHONY: deploy deploy-both deploy-rs clean

deploy: irlink
	@echo "Stopping irlink on both cams (avoids 'Text file busy')..."
	-ssh -o ConnectTimeout=5 root@dacam1 "killall irlink 2>/dev/null; sleep 1" 2>/dev/null
	-ssh -o ConnectTimeout=5 root@dacam2 "killall irlink 2>/dev/null; sleep 1" 2>/dev/null
	scp -O irlink root@dacam1:/opt/bin/irlink
	scp -O irlink root@dacam2:/opt/bin/irlink
	scp -O cgi/cal-status.cgi root@dacam1:/var/www/x/cal-status.cgi
	scp -O cgi/cal-status.cgi root@dacam2:/var/www/x/cal-status.cgi
	scp -O cgi/events-log.cgi root@dacam1:/var/www/x/events-log.cgi
	scp -O cgi/events-log.cgi root@dacam2:/var/www/x/events-log.cgi
	scp -O cgi/saved-cal.cgi  root@dacam1:/var/www/x/saved-cal.cgi
	scp -O cgi/saved-cal.cgi  root@dacam2:/var/www/x/saved-cal.cgi
	scp -O cgi/setup.cgi      root@dacam1:/var/www/x/setup.cgi
	scp -O cgi/setup.cgi      root@dacam2:/var/www/x/setup.cgi
	scp -O cgi/time-sync.cgi  root@dacam1:/var/www/x/time-sync.cgi
	scp -O cgi/time-sync.cgi  root@dacam2:/var/www/x/time-sync.cgi
	scp -O cgi/grid-deltas.cgi root@dacam1:/var/www/x/grid-deltas.cgi
	scp -O cgi/grid-deltas.cgi root@dacam2:/var/www/x/grid-deltas.cgi
	scp -O cgi/logread.cgi    root@dacam1:/var/www/x/logread.cgi
	scp -O cgi/logread.cgi    root@dacam2:/var/www/x/logread.cgi
	@# Monocal CGIs ship to cam2 ONLY — cam2 is the requestor (laptop-tethered).
	@# cam1 is responder; its monocal handling is in the irlink binary itself.
	scp -O cgi/monocal-trigger.cgi root@dacam2:/var/www/x/monocal-trigger.cgi
	scp -O cgi/monocal-status.cgi  root@dacam2:/var/www/x/monocal-status.cgi
	@# Webserver runs as www-data — needs world-readable + executable.
	@# Mode MUST be 755, not just +x — uhttpd serves mode 700 as 403.
	-ssh -o ConnectTimeout=5 root@dacam1 "chmod 755 /var/www/x/cal-status.cgi /var/www/x/events-log.cgi /var/www/x/saved-cal.cgi /var/www/x/setup.cgi /var/www/x/time-sync.cgi /var/www/x/grid-deltas.cgi /var/www/x/logread.cgi"
	-ssh -o ConnectTimeout=5 root@dacam2 "chmod 755 /var/www/x/cal-status.cgi /var/www/x/events-log.cgi /var/www/x/saved-cal.cgi /var/www/x/setup.cgi /var/www/x/time-sync.cgi /var/www/x/grid-deltas.cgi /var/www/x/logread.cgi /var/www/x/monocal-trigger.cgi /var/www/x/monocal-status.cgi"

deploy-cam1: irlink
	-ssh -o ConnectTimeout=5 root@dacam1 "killall irlink 2>/dev/null; sleep 1" 2>/dev/null
	scp -O irlink root@dacam1:/opt/bin/irlink

deploy-cam2: irlink
	-ssh -o ConnectTimeout=5 root@dacam2 "killall irlink 2>/dev/null; sleep 1" 2>/dev/null
	scp -O irlink root@dacam2:/opt/bin/irlink

deploy-test: rs_toggle_test
	scp -O rs_toggle_test root@dacam1:/opt/bin/rs_toggle_test

deploy-rs: irlink_rs_tx irlink_rs_rx
	scp -O irlink_rs_tx root@dacam1:/opt/bin/irlink_rs_tx
	scp -O irlink_rs_tx root@dacam2:/opt/bin/irlink_rs_tx
	scp -O irlink_rs_rx root@dacam1:/opt/bin/irlink_rs_rx
	scp -O irlink_rs_rx root@dacam2:/opt/bin/irlink_rs_rx

clean:
	rm -f irlink rs_toggle_test irlink_rs_tx irlink_rs_rx
