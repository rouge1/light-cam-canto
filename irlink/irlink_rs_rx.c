/*
 * irlink_rs_rx — Rolling shutter IR receiver/decoder.
 *
 * Reads vertical pixel column data from /run/prudynt/stripe_col (written by
 * patched prudynt-t BrightnessMonitor) and decodes stripe patterns created
 * by a 500Hz LED toggle into protocol messages.
 *
 * Each frame (640x360 @ 25fps) captures ~20 stripe periods = ~20 data bits.
 * Multiple frames are stitched together to decode complete messages.
 *
 * Usage:
 *   irlink_rs_rx --listen                  — continuous receive mode
 *   irlink_rs_rx --listen --col 320        — specify column
 *   irlink_rs_rx --calibrate              — find best column for stripes
 *   irlink_rs_rx --once                   — receive one message and exit
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <math.h>
#include <getopt.h>

#define STRIPE_COL_PATH     "/run/prudynt/stripe_col"
#define STRIPE_COL_X_PATH   "/run/prudynt/stripe_col_x"
#define FRAME_HEIGHT        1080   /* channel 0: 1920x1080 */
#define POLL_INTERVAL_US    5000   /* 5ms — faster than 40ms frame rate */
#define MAX_BITS            4096
#define MAX_PAYLOAD         255
#define MAX_FRAMES          50     /* max frames to accumulate */

/* Expected stripe period in rows at 1080p (500Hz, ~37us/row → ~54 rows/period)
 * Measured: ~16 rows, suggesting actual row readout is faster (~20-25us/row) */
#define MIN_STRIPE_PERIOD   10
#define MAX_STRIPE_PERIOD   70
#define EXPECTED_PERIOD     16.0

/* Minimum edge strength to detect stripes */
#define MIN_STRIPE_AMPLITUDE 5

/* Sync word (same as irlink.c) */
static const uint8_t SYNC_WORD[] = {1,1,0,0,1,0,1,1};
#define SYNC_LEN 8

static volatile int running = 1;

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* ---- CRC-8/CCITT ---- */

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

/* ---- Bit/byte helpers ---- */

static uint8_t bits_to_byte(const uint8_t *bits)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++)
        val = (val << 1) | bits[i];
    return val;
}

/* ---- Manchester decode ---- */

static int manchester_decode(const uint8_t *symbols, int n_sym, uint8_t *bits)
{
    if (n_sym % 2 != 0) return -1;
    int n_bits = 0;
    for (int i = 0; i < n_sym; i += 2) {
        if (symbols[i] == 1 && symbols[i+1] == 0)
            bits[n_bits++] = 0;
        else if (symbols[i] == 0 && symbols[i+1] == 1)
            bits[n_bits++] = 1;
        else
            return -1;
    }
    return n_bits;
}

/* ---- Protocol message ---- */

typedef struct {
    uint8_t msg_type;
    uint8_t seq;
    uint8_t data[MAX_PAYLOAD];
    int data_len;
    int valid;
} rx_message_t;

static const char *msg_type_name(uint8_t t)
{
    switch (t) {
    case 0x01: return "SYN";
    case 0x02: return "SYN_ACK";
    case 0x03: return "ACK";
    case 0x04: return "DATA";
    case 0x05: return "CAL_REQ";
    case 0x06: return "CAL_ACK";
    case 0x07: return "CAL_DONE";
    case 0x08: return "PING";
    case 0x09: return "PONG";
    default:   return "???";
    }
}

/* ---- Parse decoded bits into protocol message ---- */

static int parse_frame(const uint8_t *bits, int n_bits, rx_message_t *msg)
{
    int sync_idx = -1;
    for (int i = 0; i <= n_bits - SYNC_LEN; i++) {
        int match = 1;
        for (int j = 0; j < SYNC_LEN; j++) {
            if (bits[i+j] != SYNC_WORD[j]) { match = 0; break; }
        }
        if (match) { sync_idx = i + SYNC_LEN; break; }
    }
    if (sync_idx < 0) return -1;

    int remaining = n_bits - sync_idx;
    const uint8_t *data = bits + sync_idx;
    if (remaining < 24) return -1;

    uint8_t length = bits_to_byte(data);
    if (length < 2) return -1;

    int needed = 8 + length * 8 + 8;
    if (remaining < needed) return -1;

    uint8_t payload[MAX_PAYLOAD];
    uint8_t crc_data[MAX_PAYLOAD + 1];
    crc_data[0] = length;
    for (int i = 0; i < length; i++) {
        payload[i] = bits_to_byte(data + 8 + i * 8);
        crc_data[i + 1] = payload[i];
    }

    uint8_t received_crc = bits_to_byte(data + 8 + length * 8);
    uint8_t expected_crc = crc8(crc_data, length + 1);
    if (received_crc != expected_crc) return -1;

    msg->msg_type = payload[0];
    msg->seq = payload[1];
    msg->data_len = length - 2;
    if (msg->data_len > 0)
        memcpy(msg->data, payload + 2, msg->data_len);
    msg->valid = 1;

    return length;
}

/* ---- Read stripe column from file ---- */

static int read_stripe_col(int64_t *ts_ms, uint8_t *col, int max_rows)
{
    char buf[6144];
    int fd = open(STRIPE_COL_PATH, O_RDONLY);
    if (fd < 0) return -1;
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return -1;
    buf[n] = '\0';

    long long ts;
    char *p = buf;
    if (sscanf(p, "%lld", &ts) != 1) return -1;
    *ts_ms = (int64_t)ts;

    while (*p && *p != ' ') p++;

    int count = 0;
    while (*p && count < max_rows) {
        unsigned val;
        while (*p == ' ') p++;
        if (sscanf(p, "%u", &val) != 1) break;
        col[count++] = (uint8_t)(val > 255 ? 255 : val);
        while (*p && *p != ' ' && *p != '\n') p++;
    }
    return count;
}

/* ---- Stripe analysis: estimate period and extract symbols ---- */

/*
 * Estimate the dominant stripe period by counting zero-crossings
 * of the mean-subtracted signal.
 */
static double estimate_period(const uint8_t *col, int n)
{
    /* Compute mean */
    double sum = 0;
    for (int i = 0; i < n; i++) sum += col[i];
    double mean = sum / n;

    /* Count zero crossings */
    int crossings = 0;
    int prev_sign = (col[0] > mean) ? 1 : -1;
    for (int i = 1; i < n; i++) {
        int sign = (col[i] > mean) ? 1 : -1;
        if (sign != prev_sign) {
            crossings++;
            prev_sign = sign;
        }
    }

    if (crossings < 4) return 0;  /* not enough oscillation */

    /* Period = 2 * n / crossings (each period has 2 crossings) */
    return 2.0 * n / crossings;
}

/*
 * Compute signal amplitude (max - min of smoothed signal).
 */
static int stripe_amplitude(const uint8_t *col, int n)
{
    uint8_t mn = 255, mx = 0;
    for (int i = 0; i < n; i++) {
        if (col[i] < mn) mn = col[i];
        if (col[i] > mx) mx = col[i];
    }
    return mx - mn;
}

/*
 * Extract Manchester symbols from stripe column.
 *
 * Divides the column into half-period windows and classifies each as
 * bright (1) or dark (0) relative to the threshold.
 *
 * Returns number of symbols extracted.
 */
static int extract_symbols(const uint8_t *col, int n, double period,
                           uint8_t *symbols, int max_sym)
{
    double half_period = period / 2.0;
    if (half_period < 3) return 0;

    /* Compute threshold (midpoint of brightness range) */
    uint8_t mn = 255, mx = 0;
    for (int i = 0; i < n; i++) {
        if (col[i] < mn) mn = col[i];
        if (col[i] > mx) mx = col[i];
    }
    uint8_t threshold = mn + (mx - mn) / 2;

    /* Find first strong edge to align phase */
    int start_row = 0;
    for (int i = 1; i < n - 1; i++) {
        int diff = (int)col[i+1] - (int)col[i-1];
        if (abs(diff) > (mx - mn) / 3) {
            /* Found a strong edge — back up to the start of this half-period */
            start_row = i;
            break;
        }
    }

    /* Sample each half-period window, averaging the middle 50% */
    int n_sym = 0;
    double pos = start_row;
    while (pos + half_period <= n && n_sym < max_sym) {
        int win_start = (int)(pos + half_period * 0.25);
        int win_end = (int)(pos + half_period * 0.75);
        if (win_end >= n) break;

        int sum = 0, count = 0;
        for (int r = win_start; r <= win_end; r++) {
            sum += col[r];
            count++;
        }
        if (count == 0) break;

        symbols[n_sym++] = ((sum / count) >= threshold) ? 1 : 0;
        pos += half_period;
    }

    return n_sym;
}

/* ---- Calibration mode ---- */

static void do_calibrate(void)
{
    fprintf(stderr, "Calibrating: scanning columns for strongest stripe signal...\n");
    fprintf(stderr, "Toggle the TX camera's LED at 500Hz now (rs_toggle_test)\n");

    /* Collect frames and test multiple columns */
    int best_col = -1;
    double best_score = 0;

    /* Test columns across the frame width */
    for (int test_col = 16; test_col < 1920; test_col += 32) {
        /* Write the test column to the control file */
        {
            char buf[8];
            int len = snprintf(buf, sizeof(buf), "%d", test_col);
            int fd = open(STRIPE_COL_X_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fd >= 0) {
                write(fd, buf, len);
                close(fd);
            }
        }

        /* Wait for a few frames with this column */
        usleep(200000);  /* 200ms = ~5 frames */

        /* Read and analyze */
        double total_amp = 0;
        int good_frames = 0;
        for (int f = 0; f < 3; f++) {
            uint8_t col[FRAME_HEIGHT];
            int64_t ts;
            int nrows = read_stripe_col(&ts, col, FRAME_HEIGHT);
            if (nrows < FRAME_HEIGHT) continue;

            double period = estimate_period(col, nrows);
            int amp = stripe_amplitude(col, nrows);

            if (period > MIN_STRIPE_PERIOD && period < MAX_STRIPE_PERIOD && amp > MIN_STRIPE_AMPLITUDE) {
                total_amp += amp;
                good_frames++;
            }
            usleep(50000);  /* 50ms between reads */
        }

        double score = good_frames > 0 ? total_amp / good_frames : 0;
        if (score > best_score) {
            best_score = score;
            best_col = test_col;
        }

        if (test_col % 128 == 0)
            fprintf(stderr, "  col %3d: score=%.1f\n", test_col, score);
    }

    if (best_col >= 0 && best_score > MIN_STRIPE_AMPLITUDE) {
        fprintf(stderr, "Best column: %d (amplitude=%.1f)\n", best_col, best_score);

        /* Write to control file */
        char buf[8];
        int len = snprintf(buf, sizeof(buf), "%d", best_col);
        int fd = open(STRIPE_COL_X_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            write(fd, buf, len);
            close(fd);
        }

        printf("%d\n", best_col);
    } else {
        fprintf(stderr, "No stripe signal detected on any column.\n");
        fprintf(stderr, "Check: is the TX LED toggling? Is the IR cut filter open?\n");
    }
}

/* ---- Receive loop ---- */

static int do_receive(int once)
{
    /* Accumulated symbols across frames */
    uint8_t all_symbols[MAX_BITS * 2];
    int n_all_sym = 0;
    int64_t last_ts = 0;
    int idle_frames = 0;
    int active = 0;

    fprintf(stderr, "RX: listening for rolling shutter stripes...\n");

    while (running) {
        uint8_t col[FRAME_HEIGHT];
        int64_t ts;
        int nrows = read_stripe_col(&ts, col, FRAME_HEIGHT);

        if (nrows < FRAME_HEIGHT || ts == last_ts) {
            usleep(POLL_INTERVAL_US);
            continue;
        }
        last_ts = ts;

        /* Analyze this frame's stripe pattern */
        double period = estimate_period(col, nrows);
        int amp = stripe_amplitude(col, nrows);

        if (period < MIN_STRIPE_PERIOD || period > MAX_STRIPE_PERIOD ||
            amp < MIN_STRIPE_AMPLITUDE) {
            /* No stripes in this frame */
            if (active) {
                idle_frames++;
                if (idle_frames >= 3) {
                    /* End of transmission — try to decode accumulated symbols */
                    if (n_all_sym > 16) {
                        fprintf(stderr, "RX: end of TX, %d symbols accumulated, "
                                "period was ~%.1f rows\n", n_all_sym, period);

                        /* Try Manchester decode + frame parse */
                        /* Try multiple trim offsets for robustness */
                        rx_message_t msg;
                        int decoded = 0;

                        for (int trim = 0; trim < 4 && !decoded; trim++) {
                            for (int polarity = 0; polarity < 2 && !decoded; polarity++) {
                                uint8_t syms[MAX_BITS * 2];
                                int ns = n_all_sym - trim;
                                if (ns < 4) continue;

                                for (int i = 0; i < ns; i++) {
                                    syms[i] = polarity
                                        ? (1 - all_symbols[trim + i])
                                        : all_symbols[trim + i];
                                }

                                /* Make even for Manchester */
                                if (ns % 2 != 0) ns--;

                                uint8_t bits[MAX_BITS];
                                int nb = manchester_decode(syms, ns, bits);
                                if (nb < 0) continue;

                                memset(&msg, 0, sizeof(msg));
                                if (parse_frame(bits, nb, &msg) > 0) {
                                    decoded = 1;
                                    fprintf(stderr, "RX: decoded %s seq=%d "
                                            "(trim=%d, polarity=%s)\n",
                                            msg_type_name(msg.msg_type),
                                            msg.seq,
                                            trim,
                                            polarity ? "inverted" : "normal");

                                    /* Print message data */
                                    if (msg.data_len > 0) {
                                        msg.data[msg.data_len] = '\0';
                                        printf("MSG %s seq=%d: %s\n",
                                               msg_type_name(msg.msg_type),
                                               msg.seq, msg.data);
                                        fflush(stdout);
                                    } else {
                                        printf("MSG %s seq=%d\n",
                                               msg_type_name(msg.msg_type),
                                               msg.seq);
                                        fflush(stdout);
                                    }
                                }
                            }
                        }

                        if (!decoded) {
                            fprintf(stderr, "RX: failed to decode %d symbols\n",
                                    n_all_sym);
                        }

                        if (once && decoded)
                            return 0;
                    }

                    /* Reset */
                    n_all_sym = 0;
                    active = 0;
                    idle_frames = 0;
                }
            }
            usleep(POLL_INTERVAL_US);
            continue;
        }

        /* Valid stripes detected */
        idle_frames = 0;
        if (!active) {
            active = 1;
            n_all_sym = 0;
            fprintf(stderr, "RX: stripes detected (period=%.1f rows, amp=%d)\n",
                    period, amp);
        }

        /* Extract symbols from this frame and append */
        uint8_t frame_syms[FRAME_HEIGHT];
        int n_fsym = extract_symbols(col, nrows, period,
                                     frame_syms, FRAME_HEIGHT);

        if (n_fsym > 0 && n_all_sym + n_fsym < (int)sizeof(all_symbols)) {
            memcpy(all_symbols + n_all_sym, frame_syms, n_fsym);
            n_all_sym += n_fsym;
            fprintf(stderr, "RX: frame +%d symbols (total %d)\n",
                    n_fsym, n_all_sym);
        }

        usleep(POLL_INTERVAL_US);
    }

    return 0;
}

/* ---- Usage ---- */

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "Modes:\n");
    fprintf(stderr, "  --listen       Continuous receive mode\n");
    fprintf(stderr, "  --once         Receive one message and exit\n");
    fprintf(stderr, "  --calibrate    Find best column for stripe detection\n");
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  --col <x>      Set stripe column (default: from control file or 320)\n");
    fprintf(stderr, "  -h, --help     Show this help\n");
}

int main(int argc, char *argv[])
{
    int mode_listen = 0, mode_once = 0, mode_calibrate = 0;
    int col_x = -1;

    static struct option long_options[] = {
        {"listen",    no_argument,       0, 'l'},
        {"once",      no_argument,       0, 'o'},
        {"calibrate", no_argument,       0, 'c'},
        {"col",       required_argument, 0, 'x'},
        {"help",      no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "locx:h", long_options, NULL)) != -1) {
        switch (opt) {
        case 'l': mode_listen = 1; break;
        case 'o': mode_once = 1; break;
        case 'c': mode_calibrate = 1; break;
        case 'x':
            col_x = atoi(optarg);
            if (col_x < 0 || col_x >= 1920) {
                fprintf(stderr, "Column must be 0-639\n");
                return 1;
            }
            break;
        case 'h':
        default:
            usage(argv[0]);
            return opt == 'h' ? 0 : 1;
        }
    }

    if (!mode_listen && !mode_once && !mode_calibrate) {
        fprintf(stderr, "Error: specify --listen, --once, or --calibrate\n");
        usage(argv[0]);
        return 1;
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    /* Set column if specified on command line */
    if (col_x >= 0) {
        char buf[8];
        int len = snprintf(buf, sizeof(buf), "%d", col_x);
        int fd = open(STRIPE_COL_X_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            write(fd, buf, len);
            close(fd);
        }
        fprintf(stderr, "Set stripe column to %d\n", col_x);
    }

    if (mode_calibrate) {
        do_calibrate();
        return 0;
    }

    return do_receive(mode_once);
}
