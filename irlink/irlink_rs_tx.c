/*
 * irlink_rs_tx — Rolling shutter IR transmitter.
 *
 * Encodes a message into Manchester-coded symbols and transmits by toggling
 * the 940nm IR LED at 500Hz. Each Manchester symbol = 1ms half-period.
 * A CMOS rolling shutter camera captures the modulation as stripe patterns.
 *
 * Reuses the same frame format as OOK irlink:
 *   [PREAMBLE 8][SYNC 8][LENGTH 8][PAYLOAD][CRC-8][POSTAMBLE 4]
 *   → Manchester encoded → 500Hz GPIO toggle
 *
 * Usage:
 *   irlink_rs_tx "HELLO"              — send message once
 *   irlink_rs_tx -r 3 "HELLO"        — repeat 3 times (with gap between)
 *   irlink_rs_tx -f 1000             — set frequency to 1kHz (500us half-period)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <getopt.h>

#define IR_GPIO             49
#define MAX_SYMBOLS         4096
#define MAX_PAYLOAD         255
#define DEFAULT_FREQ_HZ     500
#define INTER_MSG_GAP_MS    50    /* gap between repeated transmissions */

/* ---- Sync word (same as irlink.c) ---- */

static const uint8_t SYNC_WORD[] = {1,1,0,0,1,0,1,1};

/* ---- Globals ---- */

static int gpio_fd = -1;
static volatile int running = 1;
static long half_period_ns = 1000000;  /* 1ms default (500Hz) */

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* ---- GPIO (sysfs) ---- */

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

/* ---- CRC-8/CCITT (poly 0x07) ---- */

static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* ---- Bit/byte conversion ---- */

static void byte_to_bits(uint8_t byte, uint8_t *bits)
{
    for (int i = 0; i < 8; i++)
        bits[i] = (byte >> (7 - i)) & 1;
}

/* ---- Manchester encode ---- */

static int encode_manchester(const uint8_t *bits, int n_bits, uint8_t *symbols)
{
    int n = 0;
    for (int i = 0; i < n_bits; i++) {
        if (bits[i] == 0) {
            symbols[n++] = 1;  /* 0 → falling edge (1→0) */
            symbols[n++] = 0;
        } else {
            symbols[n++] = 0;  /* 1 → rising edge (0→1) */
            symbols[n++] = 1;
        }
    }
    return n;
}

/* ---- Build frame symbols ---- */

#define MSG_DATA 0x04

static int build_frame_symbols(uint8_t msg_type, uint8_t seq,
                                const uint8_t *data, int data_len,
                                uint8_t *symbols)
{
    uint8_t bits[MAX_SYMBOLS];
    int nb = 0;

    /* Preamble: 10101010 */
    uint8_t preamble[] = {1,0,1,0,1,0,1,0};
    memcpy(bits + nb, preamble, 8); nb += 8;

    /* Sync: 11001011 */
    memcpy(bits + nb, SYNC_WORD, 8); nb += 8;

    /* Length: msg_type(1) + seq(1) + data_len */
    uint8_t frame_len = 2 + data_len;
    byte_to_bits(frame_len, bits + nb); nb += 8;

    /* Payload: [msg_type, seq, data...] */
    uint8_t payload[MAX_PAYLOAD];
    payload[0] = msg_type;
    payload[1] = seq;
    if (data_len > 0)
        memcpy(payload + 2, data, data_len);

    for (int i = 0; i < frame_len; i++) {
        byte_to_bits(payload[i], bits + nb);
        nb += 8;
    }

    /* CRC over [length, payload...] */
    uint8_t crc_data[MAX_PAYLOAD + 1];
    crc_data[0] = frame_len;
    memcpy(crc_data + 1, payload, frame_len);
    uint8_t crc = crc8(crc_data, frame_len + 1);
    byte_to_bits(crc, bits + nb); nb += 8;

    /* Postamble: 1010 */
    uint8_t postamble[] = {1,0,1,0};
    memcpy(bits + nb, postamble, 4); nb += 4;

    /* Manchester encode */
    return encode_manchester(bits, nb, symbols);
}

/* ---- Timespec helpers ---- */

static inline void ts_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec++;
    }
}

/* ---- Transmit symbols at rolling shutter rate ---- */

static void rs_transmit(const uint8_t *symbols, int n_sym)
{
    struct timespec next;

    /* Quiet period before transmission (10ms) */
    gpio_set(0);
    clock_gettime(CLOCK_MONOTONIC, &next);
    ts_add_ns(&next, 10000000L);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

    /* Transmit each symbol for one half-period */
    for (int i = 0; i < n_sym && running; i++) {
        gpio_set(symbols[i]);
        ts_add_ns(&next, half_period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    /* Clean termination */
    gpio_set(0);
}

/* ---- Usage ---- */

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [options] <message>\n", prog);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -f <hz>    Toggle frequency (default: %d)\n", DEFAULT_FREQ_HZ);
    fprintf(stderr, "  -r <n>     Repeat count (default: 1)\n");
    fprintf(stderr, "  -h         Show this help\n");
}

int main(int argc, char *argv[])
{
    int freq_hz = DEFAULT_FREQ_HZ;
    int repeats = 1;
    int opt;

    while ((opt = getopt(argc, argv, "f:r:h")) != -1) {
        switch (opt) {
        case 'f':
            freq_hz = atoi(optarg);
            if (freq_hz < 100 || freq_hz > 5000) {
                fprintf(stderr, "Frequency must be 100-5000 Hz\n");
                return 1;
            }
            break;
        case 'r':
            repeats = atoi(optarg);
            if (repeats < 1) repeats = 1;
            break;
        case 'h':
        default:
            usage(argv[0]);
            return opt == 'h' ? 0 : 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Error: no message specified\n");
        usage(argv[0]);
        return 1;
    }

    const char *message = argv[optind];
    int msg_len = strlen(message);
    if (msg_len > MAX_PAYLOAD - 2) {
        fprintf(stderr, "Message too long (max %d bytes)\n", MAX_PAYLOAD - 2);
        return 1;
    }

    /* Compute half-period from frequency */
    half_period_ns = 1000000000L / (2 * freq_hz);

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    if (gpio_init(IR_GPIO) < 0) {
        fprintf(stderr, "Failed to init GPIO %d\n", IR_GPIO);
        return 1;
    }

    /* Build frame */
    uint8_t symbols[MAX_SYMBOLS];
    int n_sym = build_frame_symbols(MSG_DATA, 0,
                                     (const uint8_t *)message, msg_len,
                                     symbols);

    int data_bits = (8 + 8 + 8 + (2 + msg_len) * 8 + 8 + 4);
    double tx_time_ms = n_sym * (half_period_ns / 1000000.0);
    fprintf(stderr, "Message: \"%s\" (%d bytes)\n", message, msg_len);
    fprintf(stderr, "Frame: %d data bits → %d Manchester symbols\n", data_bits, n_sym);
    fprintf(stderr, "TX time: %.1f ms at %d Hz (half-period: %ld µs)\n",
            tx_time_ms, freq_hz, half_period_ns / 1000);
    fprintf(stderr, "Effective rate: %.0f bps\n",
            msg_len * 8.0 / (tx_time_ms / 1000.0));

    for (int r = 0; r < repeats && running; r++) {
        if (repeats > 1)
            fprintf(stderr, "TX %d/%d\n", r + 1, repeats);
        rs_transmit(symbols, n_sym);

        if (r < repeats - 1 && running) {
            /* Inter-message gap */
            usleep(INTER_MSG_GAP_MS * 1000);
        }
    }

    gpio_set(0);
    close(gpio_fd);
    fprintf(stderr, "Done.\n");
    return 0;
}
