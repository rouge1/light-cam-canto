THINGINO_OUTPUT = ../thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl
CROSS_CC = $(THINGINO_OUTPUT)/host/bin/mipsel-linux-gcc
CFLAGS = -Os -Wall -Wextra

irlink_rx: irlink_rx.c
	$(CROSS_CC) $(CFLAGS) -o $@ $< -lrt

.PHONY: deploy deploy-both clean

deploy: irlink_rx
	scp -O irlink_rx root@da-camera2:/opt/bin/irlink_rx
	ssh root@da-camera2 "chmod +x /opt/bin/irlink_rx"

deploy-both: irlink_rx
	scp -O irlink_rx root@da-camera1:/opt/bin/irlink_rx
	scp -O irlink_rx root@da-camera2:/opt/bin/irlink_rx
	ssh root@da-camera1 "chmod +x /opt/bin/irlink_rx"
	ssh root@da-camera2 "chmod +x /opt/bin/irlink_rx"

clean:
	rm -f irlink_rx
