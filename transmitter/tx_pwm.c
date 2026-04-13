/*
 * tx_pwm — Fast IR transmitter using GPIO on Ingenic T31.
 *
 * Reads Manchester-encoded symbols (0s and 1s) from a file and
 * toggles the 940nm IR LED GPIO with precise timing.
 *
 * Uses direct sysfs GPIO for reliable operation. Precise timing
 * via usleep() is sufficient for the ~5-15 bps range where the
 * RTSP receiver frame rate is the bottleneck.
 *
 * Usage: tx_pwm <symbol_duration_us> <symbol_file>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

/* GPIO 49 = 940nm IR LEDs */
#define IR_GPIO  49
#define MAX_SYMBOLS  4096

static int gpio_fd = -1;

static int gpio_init(int pin)
{
    char path[64];
    FILE *fp;

    /* Export GPIO */
    fp = fopen("/sys/class/gpio/export", "w");
    if (fp) {
        fprintf(fp, "%d", pin);
        fclose(fp);
    }
    /* Ignore error — may already be exported */

    /* Set direction to output */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    fp = fopen(path, "w");
    if (!fp) {
        perror("open direction");
        return -1;
    }
    fprintf(fp, "out");
    fclose(fp);

    /* Open value file for fast writes */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    gpio_fd = open(path, O_WRONLY);
    if (gpio_fd < 0) {
        perror("open gpio value");
        return -1;
    }

    return 0;
}

static inline void gpio_set(int val)
{
    if (val)
        write(gpio_fd, "1", 1);
    else
        write(gpio_fd, "0", 1);
}

static void gpio_cleanup(void)
{
    if (gpio_fd >= 0) {
        gpio_set(0);
        close(gpio_fd);
    }
}

int main(int argc, char *argv[])
{
    int symbol_duration_us;
    char symbols[MAX_SYMBOLS];
    int n_symbols = 0;
    FILE *fp;
    int ch;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <symbol_duration_us> <symbol_file>\n", argv[0]);
        return 1;
    }

    symbol_duration_us = atoi(argv[1]);
    if (symbol_duration_us < 100 || symbol_duration_us > 2000000) {
        fprintf(stderr, "symbol_duration_us must be 100-2000000\n");
        return 1;
    }

    /* Read symbols from file */
    fp = fopen(argv[2], "r");
    if (!fp) {
        perror("fopen");
        return 1;
    }

    while ((ch = fgetc(fp)) != EOF && n_symbols < MAX_SYMBOLS) {
        if (ch == '0')
            symbols[n_symbols++] = 0;
        else if (ch == '1')
            symbols[n_symbols++] = 1;
    }
    fclose(fp);

    if (n_symbols == 0) {
        fprintf(stderr, "No symbols read\n");
        return 1;
    }

    fprintf(stderr, "TX: %d symbols at %d us/symbol (%d ms total)\n",
            n_symbols, symbol_duration_us,
            (n_symbols * symbol_duration_us) / 1000);

    /* Initialize GPIO */
    if (gpio_init(IR_GPIO) < 0)
        return 1;

    /* Quiet period before transmission */
    gpio_set(0);
    usleep(symbol_duration_us * 2);

    /* Transmit symbols */
    for (int i = 0; i < n_symbols; i++) {
        gpio_set(symbols[i]);
        usleep(symbol_duration_us);
    }

    /* Trailer: toggle a few times to ensure the last edge is captured */
    gpio_set(0);
    usleep(symbol_duration_us);
    gpio_set(1);
    usleep(symbol_duration_us);
    gpio_set(0);
    usleep(symbol_duration_us);

    /* Clean up */
    gpio_cleanup();

    fprintf(stderr, "TX: done\n");
    return 0;
}
