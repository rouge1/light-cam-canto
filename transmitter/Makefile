THINGINO_OUTPUT = ../thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl
CROSS_CC = $(THINGINO_OUTPUT)/host/bin/mipsel-linux-gcc
CFLAGS = -Os -Wall -Wextra

tx_pwm: tx_pwm.c
	$(CROSS_CC) $(CFLAGS) -o $@ $<

.PHONY: deploy clean

deploy: tx_pwm
	scp -O tx_pwm root@da-camera1:/opt/bin/tx_pwm
	ssh root@da-camera1 "chmod +x /opt/bin/tx_pwm"

clean:
	rm -f tx_pwm
