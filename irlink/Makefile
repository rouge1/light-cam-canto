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
	scp -O irlink root@da-camera1:/opt/bin/irlink
	scp -O irlink root@da-camera2:/opt/bin/irlink

deploy-cam1: irlink
	scp -O irlink root@da-camera1:/opt/bin/irlink

deploy-cam2: irlink
	scp -O irlink root@da-camera2:/opt/bin/irlink

deploy-test: rs_toggle_test
	scp -O rs_toggle_test root@da-camera1:/opt/bin/rs_toggle_test

deploy-rs: irlink_rs_tx irlink_rs_rx
	scp -O irlink_rs_tx root@da-camera1:/opt/bin/irlink_rs_tx
	scp -O irlink_rs_tx root@da-camera2:/opt/bin/irlink_rs_tx
	scp -O irlink_rs_rx root@da-camera1:/opt/bin/irlink_rs_rx
	scp -O irlink_rs_rx root@da-camera2:/opt/bin/irlink_rs_rx

clean:
	rm -f irlink rs_toggle_test irlink_rs_tx irlink_rs_rx
