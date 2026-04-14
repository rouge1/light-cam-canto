/*
 * rs_toggle_test — Toggle GPIO 49 (940nm IR LED) at 500Hz for rolling shutter testing.
 *
 * Produces a 500Hz square wave (1ms ON, 1ms OFF) that creates visible
 * stripe patterns when captured by a rolling shutter CMOS sensor.
 *
 * Uses clock_nanosleep with absolute timestamps to avoid cumulative drift.
 *
 * Usage: rs_toggle_test [seconds]   (default: 3)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>

#define IR_GPIO 49
#define HALF_PERIOD_NS 1000000  /* 1ms = 500Hz square wave */

static int gpio_fd = -1;
static volatile int running = 1;

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

static int gpio_init(int pin)
{
    char path[64];
    FILE *fp;

    fp = fopen("/sys/class/gpio/export", "w");
    if (fp) {
        fprintf(fp, "%d", pin);
        fclose(fp);
    }

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    fp = fopen(path, "w");
    if (!fp) {
        perror("gpio direction");
        return -1;
    }
    fprintf(fp, "out");
    fclose(fp);

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    gpio_fd = open(path, O_WRONLY);
    if (gpio_fd < 0) {
        perror("gpio value");
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

/* Add nanoseconds to a timespec, normalizing overflow */
static inline void ts_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec++;
    }
}

int main(int argc, char *argv[])
{
    int duration_s = 3;
    if (argc > 1)
        duration_s = atoi(argv[1]);
    if (duration_s <= 0)
        duration_s = 3;

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    if (gpio_init(IR_GPIO) < 0) {
        fprintf(stderr, "Failed to init GPIO %d\n", IR_GPIO);
        return 1;
    }

    int total_cycles = duration_s * 500;  /* 500 cycles per second at 500Hz */
    fprintf(stderr, "Toggling GPIO %d at 500Hz for %d seconds (%d cycles)\n",
            IR_GPIO, duration_s, total_cycles);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (int i = 0; i < total_cycles && running; i++) {
        gpio_set(1);
        ts_add_ns(&next, HALF_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        gpio_set(0);
        ts_add_ns(&next, HALF_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    gpio_set(0);
    close(gpio_fd);
    fprintf(stderr, "Done.\n");
    return 0;
}
