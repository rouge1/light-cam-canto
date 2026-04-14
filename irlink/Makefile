THINGINO_OUTPUT = ../thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl
CROSS_CC = $(THINGINO_OUTPUT)/host/bin/mipsel-linux-gcc
CFLAGS = -Os -Wall -Wextra -pthread
LDFLAGS = -lrt -pthread

irlink: irlink.c
	$(CROSS_CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

.PHONY: deploy deploy-both clean

deploy: irlink
	scp -O irlink root@da-camera1:/opt/bin/irlink
	scp -O irlink root@da-camera2:/opt/bin/irlink

deploy-cam1: irlink
	scp -O irlink root@da-camera1:/opt/bin/irlink

deploy-cam2: irlink
	scp -O irlink root@da-camera2:/opt/bin/irlink

clean:
	rm -f irlink
