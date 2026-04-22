/*
 * irlink — Full-duplex IR link transceiver for Wyze V3 cameras.
 *
 * Combined TX + RX with protocol layer supporting:
 * - SYN/SYN_ACK/ACK handshake (TCP-like connection setup)
 * - DATA with ACK/retransmit
 * - CAL_REQ/CAL_ACK/CAL_DONE for over-the-link ROI calibration
 *
 * TX: toggles 940nm IR LEDs via sysfs GPIO
 * RX: reads brightness grid from patched prudynt-t BrightnessMonitor
 *
 * Protocol frame payload: [msg_type] [seq_num] [data...]
 * Wrapped in Manchester-encoded frame: preamble + sync + len + payload + CRC-8 + postamble
 *
 * Usage:
 *   irlink listen              — wait for incoming connection
 *   irlink connect             — initiate connection to peer
 *   irlink send <message>      — send data (after connected)
 *   irlink calibrate           — calibrate ROI block
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>

/* ---- Configuration ---- */

#define IR_GPIO_940         49
#define IR_GPIO_850         47
#define BRIGHTNESS_PATH     "/run/prudynt/brightness"
#define GRID_PATH           "/run/prudynt/brightness_grid"
#define AE_FREEZE_PATH      "/run/prudynt/ae_freeze"
#define ROI_CONFIG_PATH     "/run/prudynt/roi_config"
#define ROI_PATH            "/run/prudynt/brightness_roi"
#define POLL_INTERVAL_US    5000
#define MAX_SAMPLES         8192
#define MAX_SYMBOLS         4096
#define MAX_PAYLOAD         255
#define SETTLE_MS           400     /* quiet time to detect end of TX (longer = fewer false splits) */
#define MIN_BRIGHTNESS_DELTA 5
#define GRID_BLOCKS         240     /* 20x12 grid (32x30 pixel blocks) */
#define MAX_RETRIES         3

/* Runtime-configurable speed */
static int symbol_ms = 160;        /* default: 160ms/symbol (~3 bps after Manchester, ~3 frames/symbol at 18fps) */

/* Computed from symbol_ms at startup */
static int ack_timeout_ms = 60000; /* auto-scaled to 3x max frame round-trip */

/* ---- Message types ---- */

#define MSG_SYN         0x01
#define MSG_SYN_ACK     0x02
#define MSG_ACK         0x03
#define MSG_DATA        0x04
#define MSG_CAL_REQ     0x05
#define MSG_CAL_ACK     0x06
#define MSG_CAL_DONE    0x07
#define MSG_PING        0x08
#define MSG_PONG        0x09

/* ---- Sync word ---- */

static const uint8_t SYNC_WORD[] = {1,1,0,0,1,0,1,1};
#define SYNC_LEN 8

/* ---- Globals ---- */

static volatile int running = 1;
static volatile int tx_active = 0;  /* suppress RX during TX */
static int tracked_block = -1;
static int pixel_x = -1, pixel_y = -1;  /* pixel-level ROI center (-1 = disabled) */
static int pixel_roi_size = 15;          /* ROI square size */
static pthread_mutex_t tx_mutex = PTHREAD_MUTEX_INITIALIZER;

/* RX callback: called when a complete message is decoded */
typedef struct {
    uint8_t msg_type;
    uint8_t seq;
    uint8_t data[MAX_PAYLOAD];
    int data_len;
    int valid;  /* set to 1 when a new message is ready */
} rx_message_t;

static rx_message_t rx_msg;
static pthread_mutex_t rx_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rx_cond = PTHREAD_COND_INITIALIZER;

/* ---- Link counters (reported via `stats` command) ---- */
static int tx_count = 0;
static int rx_count = 0;
static int crc_fail_count = 0;
static int retransmit_count = 0;
static int dpll_loss_count = 0;

/* ---- Carrier-aware ACK timeout ----
 * Timestamp of last peer-carrier sample seen by rx_thread (rx_thread is gated
 * by `tx_active`, so updates only happen when we're not transmitting).
 * Used by wait_for_msg() to extend ACK deadline while peer is mid-TX, instead
 * of timing out and retransmitting on top of an in-flight peer DATA. */
static volatile int64_t last_carrier_ms = 0;
#define CARRIER_RECENT_MS    1500   /* "peer is TXing now" if seen within this window */
#define ACK_HARD_CEILING_X   3      /* hard ceiling = ACK_HARD_CEILING_X * ack_timeout_ms */

/* ---- Signal handler ---- */

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

/* ---- Monotonic time ---- */

static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* ---- AE freeze control (via prudynt BrightnessMonitor) ---- */

static void set_ae_freeze(int freeze)
{
    int fd = open(AE_FREEZE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
        write(fd, freeze ? "1" : "0", 1);
        close(fd);
        fprintf(stderr, "AE: freeze %s\n", freeze ? "ON" : "OFF");
    }
}

/* ================================================================
 *  GPIO TX
 * ================================================================ */

static int gpio_fd_940 = -1;
static int gpio_fd_850 = -1;

static int gpio_open(int pin)
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
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("gpio value");
        return -1;
    }

    return fd;
}

static int gpio_init(int pin940, int pin850)
{
    gpio_fd_940 = gpio_open(pin940);
    if (gpio_fd_940 < 0) return -1;
    gpio_fd_850 = gpio_open(pin850);
    if (gpio_fd_850 < 0)
        fprintf(stderr, "GPIO: 850nm (pin %d) not available, using 940nm only\n", pin850);
    return 0;
}

static inline void gpio_set(int val)
{
    if (val) {
        if (gpio_fd_940 >= 0) write(gpio_fd_940, "1", 1);
        if (gpio_fd_850 >= 0) write(gpio_fd_850, "1", 1);
    } else {
        if (gpio_fd_940 >= 0) write(gpio_fd_940, "0", 1);
        if (gpio_fd_850 >= 0) write(gpio_fd_850, "0", 1);
    }
}

/* ---- Manchester encode data bits into symbols ---- */

static int encode_manchester(const uint8_t *bits, int n_bits, uint8_t *symbols)
{
    int n = 0;
    for (int i = 0; i < n_bits; i++) {
        if (bits[i] == 0) {
            symbols[n++] = 1;
            symbols[n++] = 0;
        } else {
            symbols[n++] = 0;
            symbols[n++] = 1;
        }
    }
    return n;
}

/* ---- Encode a frame to symbols ---- */

static void byte_to_bits(uint8_t byte, uint8_t *bits)
{
    for (int i = 0; i < 8; i++)
        bits[i] = (byte >> (7 - i)) & 1;
}

static int build_frame_symbols(uint8_t msg_type, uint8_t seq,
                                const uint8_t *data, int data_len,
                                uint8_t *symbols)
{
    /* Build data bits: preamble + sync + length + payload + CRC + postamble */
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

/* ---- Transmit symbols (thread-safe) ---- */

static void transmit_symbols(const uint8_t *symbols, int n_sym)
{
    pthread_mutex_lock(&tx_mutex);
    tx_active = 1;

    /* Quiet before */
    gpio_set(0);
    usleep((symbol_ms * 1000) * 2);

    for (int i = 0; i < n_sym; i++) {
        gpio_set(symbols[i]);
        usleep((symbol_ms * 1000));
    }

    /* Trailer */
    gpio_set(0);
    usleep((symbol_ms * 1000));
    gpio_set(1);
    usleep((symbol_ms * 1000));
    gpio_set(0);

    /* Inter-message gap: exceed SETTLE_MS so peer detects end of TX */
    usleep((SETTLE_MS + 100) * 1000);

    tx_active = 0;
    pthread_mutex_unlock(&tx_mutex);
}

/* ---- Send a protocol message ---- */

static void send_message(uint8_t msg_type, uint8_t seq,
                          const uint8_t *data, int data_len)
{
    uint8_t symbols[MAX_SYMBOLS];
    int n = build_frame_symbols(msg_type, seq, data, data_len, symbols);

    const char *type_names[] = {
        [MSG_SYN] = "SYN", [MSG_SYN_ACK] = "SYN_ACK", [MSG_ACK] = "ACK",
        [MSG_DATA] = "DATA", [MSG_CAL_REQ] = "CAL_REQ",
        [MSG_CAL_ACK] = "CAL_ACK", [MSG_CAL_DONE] = "CAL_DONE",
        [MSG_PING] = "PING", [MSG_PONG] = "PONG"
    };
    const char *name = (msg_type <= MSG_PONG) ? type_names[msg_type] : "???";
    fprintf(stderr, "TX: %s seq=%d (%d symbols)\n", name, seq, n);

    tx_count++;
    transmit_symbols(symbols, n);
}

/* ================================================================
 *  Brightness RX
 * ================================================================ */

typedef struct {
    int64_t ts_ms;
    uint8_t brightness;
} sample_t;

/* ---- Read grid file ---- */

static int read_grid(int64_t *ts_ms, uint8_t *blocks, int max_blocks)
{
    char buf[2048];
    int fd = open(GRID_PATH, O_RDONLY);
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
    while (*p && count < max_blocks) {
        unsigned val;
        while (*p == ' ') p++;
        if (sscanf(p, "%u", &val) != 1) break;
        blocks[count++] = (uint8_t)(val > 255 ? 255 : val);
        while (*p && *p != ' ' && *p != '\n') p++;
    }
    return count;
}

/* ---- Write pixel ROI config for BrightnessMonitor ---- */

static void write_roi_config(int x, int y, int size)
{
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "%d %d %d\n", x, y, size);
    int fd = open(ROI_CONFIG_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
        (void)write(fd, buf, len);
        close(fd);
    }
}

/* ---- Read pixel ROI brightness from BrightnessMonitor ---- */

static int read_roi(int64_t *ts_ms, uint8_t *brightness)
{
    char buf[64];
    int fd = open(ROI_PATH, O_RDONLY);
    if (fd < 0) return -1;
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return -1;
    buf[n] = '\0';

    long long ts;
    unsigned val;
    if (sscanf(buf, "%lld %u", &ts, &val) == 2) {
        *ts_ms = (int64_t)ts;
        *brightness = (uint8_t)(val > 255 ? 255 : val);
        return 0;
    }
    return -1;
}

/* ---- Read brightness: pixel ROI, grid block, or global ---- */

static int read_brightness(int64_t *ts_ms, uint8_t *brightness)
{
    /* Prefer pixel ROI if configured */
    if (pixel_x >= 0 && pixel_y >= 0) {
        return read_roi(ts_ms, brightness);
    }

    if (tracked_block >= 0) {
        uint8_t blocks[GRID_BLOCKS];
        int64_t grid_ts;
        int nblocks = read_grid(&grid_ts, blocks, GRID_BLOCKS);
        if (nblocks > tracked_block) {
            *ts_ms = grid_ts;
            *brightness = blocks[tracked_block];
            return 0;
        }
    }

    char buf[64];
    int fd = open(BRIGHTNESS_PATH, O_RDONLY);
    if (fd < 0) return -1;
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return -1;
    buf[n] = '\0';

    long long ts;
    unsigned val, max_val;
    if (sscanf(buf, "%lld %u %u", &ts, &val, &max_val) == 3) {
        *ts_ms = (int64_t)ts;
        *brightness = (uint8_t)(val > 255 ? 255 : val);
        return 0;
    }
    if (sscanf(buf, "%lld %u", &ts, &val) == 2) {
        *ts_ms = (int64_t)ts;
        *brightness = (uint8_t)(val > 255 ? 255 : val);
        return 0;
    }
    return -1;
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

static uint8_t bits_to_byte(const uint8_t *bits)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++)
        val = (val << 1) | bits[i];
    return val;
}

/* ---- Parse decoded bits into protocol message ---- */

static int parse_frame(const uint8_t *bits, int n_bits, rx_message_t *msg)
{
    /* Find sync word */
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
    if (length < 2) return -1;  /* need at least msg_type + seq */

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

/* ---- DPLL decode: track clock from raw sample edges ---- */

static int decode_samples_dpll(sample_t *samp, int n, rx_message_t *msg)
{
    if (n < 10) return -1;

    uint8_t bmin = 255, bmax = 0;
    for (int i = 1; i < n; i++) {
        if (samp[i].brightness < bmin) bmin = samp[i].brightness;
        if (samp[i].brightness > bmax) bmax = samp[i].brightness;
    }

    int delta = bmax - bmin;
    if (delta < MIN_BRIGHTNESS_DELTA) return -1;

    uint8_t threshold = (bmin + bmax) / 2;

    fprintf(stderr, "RX: DPLL %d samples, brightness %d-%d, delta %d, threshold %d\n",
            n, bmin, bmax, delta, threshold);

    /* Digitize raw samples */
    int8_t dig[MAX_SAMPLES];
    for (int i = 0; i < n; i++)
        dig[i] = samp[i].brightness >= threshold ? 1 : 0;

    /* Find first rising edge */
    int64_t first_edge_t = -1;
    for (int i = 1; i < n; i++) {
        if (dig[i] == 1 && dig[i-1] == 0) {
            first_edge_t = (samp[i-1].ts_ms + samp[i].ts_ms) / 2;
            break;
        }
    }
    if (first_edge_t < 0) return -1;

    double T = (double)symbol_ms;
    static const double gains[] = {0.15, 0.25, 0.35};
    static const double phase_offs[] = {0.5, 0.3, 0.7};

    for (int gi = 0; gi < 3; gi++) {
        for (int pi = 0; pi < 3; pi++) {
            double gain = gains[gi];
            double phase = first_edge_t + T * phase_offs[pi];

            uint8_t symbols[MAX_SYMBOLS];
            int n_sym = 0;
            int64_t last_ts = samp[n-1].ts_ms;

            while (phase < last_ts - T * 0.3 && n_sym < MAX_SYMBOLS) {
                /* Collect raw samples in center 60% of symbol window */
                double w_lo = phase - T * 0.3;
                double w_hi = phase + T * 0.3;

                int sum = 0, count = 0;
                for (int i = 0; i < n; i++) {
                    if (samp[i].ts_ms >= w_lo && samp[i].ts_ms <= w_hi) {
                        sum += dig[i];
                        count++;
                    }
                }

                if (count == 0) {
                    /* No samples — use nearest */
                    int best_i = 0;
                    int64_t best_dist = llabs(samp[0].ts_ms - (int64_t)phase);
                    for (int i = 1; i < n; i++) {
                        int64_t dist = llabs(samp[i].ts_ms - (int64_t)phase);
                        if (dist < best_dist) {
                            best_dist = dist;
                            best_i = i;
                        }
                    }
                    symbols[n_sym++] = dig[best_i];
                } else {
                    symbols[n_sym++] = (sum * 2 >= count) ? 1 : 0;
                }

                /* Phase correction: find nearest transition near boundary */
                double boundary = phase + T * 0.5;
                double search_lo = boundary - T * 0.4;
                double search_hi = boundary + T * 0.4;

                double best_edge = 0;
                double best_dist = T * 0.4 + 1;
                int found_edge = 0;

                for (int j = 1; j < n; j++) {
                    if (samp[j].ts_ms < search_lo || samp[j-1].ts_ms > search_hi)
                        continue;
                    if (dig[j] != dig[j-1]) {
                        double edge_t = (samp[j-1].ts_ms + samp[j].ts_ms) / 2.0;
                        double dist = edge_t > boundary ? edge_t - boundary : boundary - edge_t;
                        if (dist < best_dist) {
                            best_dist = dist;
                            best_edge = edge_t;
                            found_edge = 1;
                        }
                    }
                }

                if (found_edge) {
                    double error = best_edge - boundary;
                    phase += T + gain * error;
                } else {
                    phase += T;
                }
            }

            if (n_sym < 20) continue;

            /* Try decode with trims and both polarities */
            for (int inv = 0; inv < 2; inv++) {
                uint8_t *syms = symbols;
                if (inv) {
                    for (int i = 0; i < n_sym; i++)
                        symbols[i] = symbols[i] ? 0 : 1;
                }

                for (int trim_s = 0; trim_s < 16; trim_s++) {
                    for (int trim_e = 0; trim_e < 16; trim_e++) {
                        int sc = n_sym - trim_s - trim_e;
                        if (sc < 20 || sc % 2 != 0) continue;

                        uint8_t bits[MAX_SYMBOLS / 2];
                        int nb = manchester_decode(syms + trim_s, sc, bits);
                        if (nb < 0) continue;

                        if (parse_frame(bits, nb, msg) > 0) {
                            fprintf(stderr, "RX: DPLL decoded (gain=%.2f, phase_off=%.1f, trim [%d:%d]%s)\n",
                                    gain, phase_offs[pi], trim_s, trim_e, inv ? " inv" : "");
                            return 0;
                        }
                    }
                }

                /* Undo inversion for next loop iteration */
                if (inv) {
                    for (int i = 0; i < n_sym; i++)
                        symbols[i] = symbols[i] ? 0 : 1;
                }
            }
        }
    }

    return -1;
}

/* ---- Decode collected samples (fixed-grid, original method) ---- */

static int decode_samples(sample_t *samp, int n, rx_message_t *msg)
{
    if (n < 4) return -1;

    /* Use sample[0] (synthetic baseline) as the "LED off" reference.
     * bmin/bmax can be skewed by transient noise or AE settling. */
    uint8_t baseline = samp[0].brightness;
    uint8_t bmin = 255, bmax = 0;
    for (int i = 1; i < n; i++) {
        if (samp[i].brightness < bmin) bmin = samp[i].brightness;
        if (samp[i].brightness > bmax) bmax = samp[i].brightness;
    }

    int delta = bmax - bmin;
    if (delta < MIN_BRIGHTNESS_DELTA) return -1;

    /* Threshold = midpoint between baseline and peak, not bmin and bmax.
     * This avoids transient dips pulling the threshold below baseline. */
    uint8_t threshold = (baseline + bmax) / 2;
    if (threshold <= baseline) threshold = baseline + delta / 4;

    fprintf(stderr, "RX: %d samples, brightness %d-%d, delta %d, threshold %d\n",
            n, bmin, bmax, delta, threshold);

    /* Find TX start: first sample that crosses threshold.
     * Skip sample[0] which is a synthetic baseline placeholder. */
    int tx_start = -1;
    for (int i = 1; i < n; i++) {
        int diff = (int)samp[i].brightness - (int)samp[0].brightness;
        if (diff < 0) diff = -diff;
        if (diff >= delta / 3) {
            /* Back up 1 sample for context, but not to the synthetic sample[0] */
            tx_start = i > 1 ? i - 1 : i;
            break;
        }
    }
    if (tx_start < 0) return -1;

    /* Use half a symbol before the first real transition as t0 */
    int64_t t0 = samp[tx_start].ts_ms - symbol_ms / 2;

    fprintf(stderr, "RX: tx_start=%d, t0=%lld, first_ts=%lld\n",
            tx_start, (long long)t0, (long long)samp[tx_start].ts_ms);

    /* Extract symbols */
    uint8_t symbols[MAX_SYMBOLS];
    int n_sym = 0;

    for (int sym = 0; sym < MAX_SYMBOLS; sym++) {
        int64_t win_start = t0 + (int64_t)symbol_ms * sym + symbol_ms / 4;
        int64_t win_end   = t0 + (int64_t)symbol_ms * sym + symbol_ms * 3 / 4;

        int sum = 0, count = 0;
        for (int i = tx_start; i < n; i++) {
            if (samp[i].ts_ms >= win_start && samp[i].ts_ms <= win_end) {
                sum += samp[i].brightness;
                count++;
            }
            if (samp[i].ts_ms > win_end) break;
        }
        if (count == 0) break;
        symbols[n_sym++] = (sum / count) >= threshold ? 1 : 0;
    }

    fprintf(stderr, "RX: %d symbols from tx_start=%d\n", n_sym, tx_start);

    if (n_sym > 0) {
        fprintf(stderr, "RX: symbols: ");
        int lim = n_sym < 120 ? n_sym : 120;
        for (int i = 0; i < lim; i++) fprintf(stderr, "%d", symbols[i]);
        fprintf(stderr, "\n");
    }

    if (n_sym < 20) return -1;

    /* Try decoding with trim offsets */
    for (int trim_s = 0; trim_s < 12; trim_s++) {
        for (int trim_e = 0; trim_e < 12; trim_e++) {
            int sc = n_sym - trim_s - trim_e;
            if (sc < 20 || sc % 2 != 0) continue;

            uint8_t bits[MAX_SYMBOLS / 2];
            int nb = manchester_decode(symbols + trim_s, sc, bits);
            if (nb < 0) continue;

            if (parse_frame(bits, nb, msg) > 0) {
                fprintf(stderr, "RX: decoded trim [%d:%d]\n", trim_s, trim_e);
                return 0;
            }
        }
    }

    /* Try inverted */
    for (int i = 0; i < n_sym; i++) symbols[i] = symbols[i] ? 0 : 1;

    for (int trim_s = 0; trim_s < 12; trim_s++) {
        for (int trim_e = 0; trim_e < 12; trim_e++) {
            int sc = n_sym - trim_s - trim_e;
            if (sc < 20 || sc % 2 != 0) continue;

            uint8_t bits[MAX_SYMBOLS / 2];
            int nb = manchester_decode(symbols + trim_s, sc, bits);
            if (nb < 0) continue;

            if (parse_frame(bits, nb, msg) > 0) {
                fprintf(stderr, "RX: decoded inverted trim [%d:%d]\n", trim_s, trim_e);
                return 0;
            }
        }
    }

    return -1;
}

/* ---- Calibration: AE freeze + raw delta (LED OFF vs LED ON) ---- */

static int calibrate(void)
{
    fprintf(stderr, "CALIBRATE: freezing AE...\n");
    set_ae_freeze(1);
    usleep(500000); /* 500ms for AE to apply final update */

    /* Step 1: Capture baseline grid (peer LED OFF) — average several frames */
    fprintf(stderr, "CALIBRATE: capturing baseline (LED OFF)...\n");
    int baseline[GRID_BLOCKS] = {0};
    int n_baseline = 0;
    int64_t last_ts = 0;
    int64_t start = now_ms();

    while (now_ms() - start < 1000 && n_baseline < 30) {
        int64_t ts;
        uint8_t blocks[GRID_BLOCKS];
        int n = read_grid(&ts, blocks, GRID_BLOCKS);
        if (n > 0 && ts != last_ts) {
            last_ts = ts;
            for (int i = 0; i < GRID_BLOCKS; i++)
                baseline[i] += blocks[i];
            n_baseline++;
        }
        usleep(POLL_INTERVAL_US);
    }

    if (n_baseline < 5) {
        fprintf(stderr, "CALIBRATE: too few baseline frames (%d)\n", n_baseline);
        set_ae_freeze(0);
        return -1;
    }

    for (int i = 0; i < GRID_BLOCKS; i++)
        baseline[i] /= n_baseline;

    fprintf(stderr, "CALIBRATE: got %d baseline frames\n", n_baseline);

    /* Step 2: Wait for peer LED ON — capture grid for 4 seconds */
    fprintf(stderr, "CALIBRATE: turn ON the peer's IR LED now! (4 second window)\n");

    int led_on[GRID_BLOCKS] = {0};
    int n_led = 0;
    last_ts = 0;
    start = now_ms();

    while (now_ms() - start < 4000 && n_led < 120) {
        int64_t ts;
        uint8_t blocks[GRID_BLOCKS];
        int n = read_grid(&ts, blocks, GRID_BLOCKS);
        if (n > 0 && ts != last_ts) {
            last_ts = ts;
            for (int i = 0; i < GRID_BLOCKS; i++)
                led_on[i] += blocks[i];
            n_led++;
        }
        usleep(POLL_INTERVAL_US);
    }

    if (n_led < 5) {
        fprintf(stderr, "CALIBRATE: too few LED-ON frames (%d)\n", n_led);
        set_ae_freeze(0);
        return -1;
    }

    for (int i = 0; i < GRID_BLOCKS; i++)
        led_on[i] /= n_led;

    fprintf(stderr, "CALIBRATE: got %d LED-ON frames\n", n_led);

    /* Step 3: Find block with biggest positive delta (LED ON - LED OFF) */
    int best = -1, best_delta = 0;
    int deltas[GRID_BLOCKS];

    for (int b = 0; b < GRID_BLOCKS; b++) {
        deltas[b] = led_on[b] - baseline[b];
        if (deltas[b] > best_delta) {
            best_delta = deltas[b];
            best = b;
        }
    }

    set_ae_freeze(0);

    if (best < 0 || best_delta < 3) {
        fprintf(stderr, "CALIBRATE: no IR block detected (max delta=%d)\n", best_delta);
        return -1;
    }

    fprintf(stderr, "CALIBRATE: block=%d (row=%d, col=%d), delta=%d (OFF=%d, ON=%d)\n",
            best, best / 20, best % 20, best_delta, baseline[best], led_on[best]);

    /* Print delta grid */
    fprintf(stderr, "CALIBRATE: delta grid (20x12):\n");
    for (int r = 0; r < 12; r++) {
        fprintf(stderr, "  ");
        for (int c = 0; c < 20; c++) {
            int idx = r * 20 + c;
            if (idx >= GRID_BLOCKS) break;
            if (idx == best)
                fprintf(stderr, "[%3d]", deltas[idx]);
            else
                fprintf(stderr, " %3d ", deltas[idx]);
        }
        fprintf(stderr, "\n");
    }

    printf("%d\n", best);
    return best;
}

/* ================================================================
 *  RX Thread
 * ================================================================ */

static sample_t rx_samples[MAX_SAMPLES]; /* static to avoid stack overflow */

static void *rx_thread(void *arg)
{
    (void)arg;

    sample_t *samples = rx_samples;
    int n_samples = 0;

    enum { IDLE, ACTIVE } state = IDLE;
    uint8_t baseline = 0;
    int baseline_set = 0;
    int64_t last_change_ms = 0;
    int64_t last_ts = 0;
    int64_t tx_end_time = 0;  /* timestamp when tx_active last went 0 */
    int was_tx_active = 0;
    int active_count = 0;    /* consecutive frames above threshold */

    fprintf(stderr, "RX: thread started (block=%d)\n", tracked_block);

    /* Log first few brightness readings for debugging */
    int debug_count = 0;

    while (running) {
        /* Suppress RX during our own TX to avoid self-interference. */
        if (tx_active) {
            if (state == ACTIVE) {
                state = IDLE;
                n_samples = 0;
            }
            was_tx_active = 1;
            usleep(POLL_INTERVAL_US);
            continue;
        }

        /* Record when TX ended for post-TX baseline settling */
        if (was_tx_active) {
            was_tx_active = 0;
            tx_end_time = now_ms();
            baseline_set = 0;
        }

        int64_t ts_ms;
        uint8_t brightness;

        if (read_brightness(&ts_ms, &brightness) < 0) {
            usleep(POLL_INTERVAL_US);
            continue;
        }

        if (ts_ms == last_ts) {
            usleep(POLL_INTERVAL_US);
            continue;
        }
        last_ts = ts_ms;

        /* After self-TX ends, observe a 500ms settling window.
           Track MIN (assumed-quiet: LED off) rather than last-sample, so
           that if the peer starts TXing during this window (fast turnaround
           in half-duplex), we don't mistake the peer's bright state for
           baseline. Emit baseline = min after the window closes. */
        static uint8_t settle_min = 255;
        static uint8_t settle_max = 0;
        if (tx_end_time > 0 && now_ms() - tx_end_time < 500) {
            if (brightness < settle_min) settle_min = brightness;
            if (brightness > settle_max) settle_max = brightness;
            usleep(POLL_INTERVAL_US);
            continue;
        }
        if (tx_end_time > 0) {
            /* Window closed — commit baseline using min seen. */
            baseline = settle_min;
            baseline_set = 1;
            last_change_ms = now_ms();
            fprintf(stderr, "RX: post-TX baseline=%d (min over 500ms, range=%d)\n",
                    baseline, (int)settle_max - (int)settle_min);
            settle_min = 255;
            settle_max = 0;
            tx_end_time = 0;
            debug_count = 0;  /* re-arm brightness debug */
        }

        if (!baseline_set) {
            baseline = brightness;
            baseline_set = 1;
            last_change_ms = now_ms();
            fprintf(stderr, "RX: baseline set to %d\n", baseline);
        }

        if (debug_count < 10) {
            fprintf(stderr, "RX: [%d] brightness=%d baseline=%d diff=%d\n",
                    debug_count, brightness, baseline,
                    ((int)brightness - (int)baseline) < 0 ?
                    -((int)brightness - (int)baseline) :
                    ((int)brightness - (int)baseline));
            debug_count++;
        }

        int diff = (int)brightness - (int)baseline;
        if (diff < 0) diff = -diff;

        switch (state) {
        case IDLE:
            if (diff >= MIN_BRIGHTNESS_DELTA) {
                active_count++;
                /* Carrier hint: even a single above-threshold sample is enough
                   to bump the ACK-wait deadline — false positives only delay,
                   never lose data. */
                last_carrier_ms = now_ms();
                if (active_count >= 2) {  /* require 2 consecutive frames (AE frozen = clean signal) */
                    fprintf(stderr, "RX: activity detected (baseline=%d, now=%d, delta=%d)\n",
                            baseline, brightness, diff);
                    state = ACTIVE;
                    n_samples = 0;
                    samples[n_samples].ts_ms = ts_ms - symbol_ms;
                    samples[n_samples].brightness = baseline;
                    n_samples++;
                }
            } else {
                active_count = 0;
                baseline = (uint8_t)(((int)baseline * 7 + (int)brightness + 4) / 8);
            }
            break;

        case ACTIVE:
            if (n_samples < MAX_SAMPLES) {
                samples[n_samples].ts_ms = ts_ms;
                samples[n_samples].brightness = brightness;
                n_samples++;
            }
            /* Carrier confirmed — keep extending the ACK deadline. */
            last_carrier_ms = now_ms();

            /* Use higher threshold to keep settle timer running —
             * only reset on strong signal, not baseline drift */
            if (diff >= MIN_BRIGHTNESS_DELTA * 2)
                last_change_ms = now_ms();

            if (now_ms() - last_change_ms > SETTLE_MS) {
                fprintf(stderr, "RX: end of TX (%d samples)\n", n_samples);

                rx_message_t msg;
                memset(&msg, 0, sizeof(msg));
                int dpll_ok = (decode_samples_dpll(samples, n_samples, &msg) == 0);
                int decoded = dpll_ok;
                if (!decoded) {
                    decoded = (decode_samples(samples, n_samples, &msg) == 0);
                    if (decoded) dpll_loss_count++;
                }
                if (decoded) {
                    rx_count++;
                    const char *names[] = {
                        [0]="?", [MSG_SYN]="SYN", [MSG_SYN_ACK]="SYN_ACK",
                        [MSG_ACK]="ACK", [MSG_DATA]="DATA",
                        [MSG_CAL_REQ]="CAL_REQ", [MSG_CAL_ACK]="CAL_ACK",
                        [MSG_CAL_DONE]="CAL_DONE",
                        [MSG_PING]="PING", [MSG_PONG]="PONG"
                    };
                    const char *n = (msg.msg_type <= MSG_PONG) ? names[msg.msg_type] : "?";
                    fprintf(stderr, "RX: <<< %s seq=%d len=%d >>>\n",
                            n, msg.seq, msg.data_len);

                    if (msg.msg_type == MSG_DATA && msg.data_len > 0) {
                        /* Hex form is binary-safe (app-layer payloads may contain \0).
                           Text form kept for legacy/human use but will truncate at \0. */
                        printf("MSG-HEX: ");
                        for (int i = 0; i < msg.data_len; i++)
                            printf("%02x", msg.data[i]);
                        printf("\n");
                        msg.data[msg.data_len] = '\0';
                        printf("MSG: %s\n", (char *)msg.data);
                        fflush(stdout);
                    }

                    /* Signal the protocol thread */
                    pthread_mutex_lock(&rx_mutex);
                    rx_msg = msg;
                    pthread_cond_signal(&rx_cond);
                    pthread_mutex_unlock(&rx_mutex);

                    /* Auto-ACK DATA / auto-PONG PING from rx_thread.
                       The interactive_mode main loop blocks in fgets() with no
                       way to be woken by rx_cond — without this, an incoming
                       DATA sits in rx_msg until the orchestrator's NEXT stdin
                       command arrives, deadlocking any wait=True send_app on
                       the peer side. wait_for_msg consumers still see the
                       message; they just no longer need to ACK it themselves.
                       CAL_REQ stays main-handled (needs LED-on hold). */
                    if (msg.msg_type == MSG_DATA) {
                        send_message(MSG_ACK, msg.seq, NULL, 0);
                    } else if (msg.msg_type == MSG_PING) {
                        send_message(MSG_PONG, 0, NULL, 0);
                    }
                } else {
                    crc_fail_count++;
                    fprintf(stderr, "RX: decode failed\n");
                }

                state = IDLE;
                baseline = brightness;
                last_change_ms = now_ms();
                n_samples = 0;
            }
            break;
        }

        usleep(POLL_INTERVAL_US);
    }

    fprintf(stderr, "RX: thread stopped\n");
    return NULL;
}

/* ---- Wait for a specific message type (with timeout) ---- */

static int wait_for_msg(uint8_t expected_type, rx_message_t *out, int timeout_ms)
{
    /* Carrier-aware deadline:
     *   - base deadline = now + timeout_ms
     *   - hard ceiling  = now + ACK_HARD_CEILING_X * timeout_ms
     *   On each timeout, if rx_thread observed peer carrier within
     *   CARRIER_RECENT_MS, push the deadline out by another `timeout_ms` (capped
     *   at hard ceiling). Prevents premature retransmit on top of an in-flight
     *   peer DATA frame, while still failing fast on a truly dead link. */
    int64_t start_ms = now_ms();
    int64_t hard_deadline_ms = start_ms + (int64_t)timeout_ms * ACK_HARD_CEILING_X;
    int64_t deadline_ms = start_ms + timeout_ms;
    int extensions = 0;

    pthread_mutex_lock(&rx_mutex);
    while (running) {
        if (rx_msg.valid && rx_msg.msg_type == expected_type) {
            *out = rx_msg;
            rx_msg.valid = 0;
            pthread_mutex_unlock(&rx_mutex);
            return 0;
        }
        if (rx_msg.valid) {
            /* Non-matching message — rx_thread already auto-ACKed/auto-PONGed
               DATA/PING, so just consume here and keep waiting for the
               expected reply. CAL_REQ is left for main-loop handling. */
            rx_msg.valid = 0;
            continue;
        }

        /* Compute REALTIME timespec for this iteration's deadline. */
        struct timespec ts_realtime;
        clock_gettime(CLOCK_REALTIME, &ts_realtime);
        int64_t now_mono = now_ms();
        int64_t remaining_ms = deadline_ms - now_mono;
        if (remaining_ms < 0) remaining_ms = 0;
        ts_realtime.tv_sec += remaining_ms / 1000;
        ts_realtime.tv_nsec += (remaining_ms % 1000) * 1000000L;
        if (ts_realtime.tv_nsec >= 1000000000L) {
            ts_realtime.tv_sec++;
            ts_realtime.tv_nsec -= 1000000000L;
        }

        int ret = pthread_cond_timedwait(&rx_cond, &rx_mutex, &ts_realtime);
        if (ret == ETIMEDOUT) {
            int64_t now2 = now_ms();
            int64_t since_carrier = now2 - last_carrier_ms;
            if (now2 < hard_deadline_ms && since_carrier < CARRIER_RECENT_MS) {
                /* Peer is still TXing — extend deadline. */
                deadline_ms = now2 + timeout_ms;
                if (deadline_ms > hard_deadline_ms) deadline_ms = hard_deadline_ms;
                extensions++;
                fprintf(stderr,
                        "PROTO: ACK wait extended (peer carrier %lldms ago, ext=%d, %lldms left to ceiling)\n",
                        (long long)since_carrier, extensions,
                        (long long)(hard_deadline_ms - now2));
                continue;
            }
            pthread_mutex_unlock(&rx_mutex);
            return -1;
        }
    }
    pthread_mutex_unlock(&rx_mutex);
    return -1;
}

/* ---- Wait for any message (with timeout) ---- */

static int wait_for_any_msg(rx_message_t *out, int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }

    pthread_mutex_lock(&rx_mutex);
    while (running) {
        if (rx_msg.valid) {
            *out = rx_msg;
            rx_msg.valid = 0;
            pthread_mutex_unlock(&rx_mutex);
            return 0;
        }

        int ret = pthread_cond_timedwait(&rx_cond, &rx_mutex, &ts);
        if (ret == ETIMEDOUT) {
            pthread_mutex_unlock(&rx_mutex);
            return -1;
        }
    }
    pthread_mutex_unlock(&rx_mutex);
    return -1;
}

/* ================================================================
 *  Protocol Commands
 * ================================================================ */

/* ---- Connect: initiate handshake ---- */

static int do_connect(void)
{
    fprintf(stderr, "PROTO: initiating connection (SYN)...\n");

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        send_message(MSG_SYN, 0, NULL, 0);

        fprintf(stderr, "PROTO: waiting for SYN_ACK...\n");
        rx_message_t reply;
        if (wait_for_msg(MSG_SYN_ACK, &reply, ack_timeout_ms) == 0) {
            fprintf(stderr, "PROTO: got SYN_ACK! Sending ACK...\n");
            send_message(MSG_ACK, 0, NULL, 0);
            fprintf(stderr, "PROTO: connected!\n");
            return 0;
        }
        fprintf(stderr, "PROTO: SYN_ACK timeout, retry %d/%d\n",
                attempt + 1, MAX_RETRIES);
    }

    fprintf(stderr, "PROTO: connection failed after %d attempts\n", MAX_RETRIES);
    return -1;
}

/* ---- Listen: wait for incoming connection ---- */

static int do_listen(void)
{
    fprintf(stderr, "PROTO: listening for SYN...\n");

    while (running) {
        rx_message_t msg;
        /* Wait indefinitely (60s chunks to check running flag) */
        if (wait_for_msg(MSG_SYN, &msg, 60000) == 0) {
            fprintf(stderr, "PROTO: got SYN! Sending SYN_ACK...\n");
            send_message(MSG_SYN_ACK, 0, NULL, 0);

            fprintf(stderr, "PROTO: waiting for ACK...\n");
            rx_message_t ack;
            if (wait_for_msg(MSG_ACK, &ack, ack_timeout_ms) == 0) {
                fprintf(stderr, "PROTO: connected!\n");
                return 0;
            }
            fprintf(stderr, "PROTO: ACK timeout, back to listening\n");
        }
    }
    return -1;
}

/* ---- Send data with ACK ---- */

static int do_send_bytes(const uint8_t *data, int len)
{
    if (len > MAX_PAYLOAD - 2) {
        fprintf(stderr, "PROTO: message too long (%d, max %d)\n", len, MAX_PAYLOAD - 2);
        return -1;
    }

    static uint8_t seq = 1;

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        if (attempt > 0) retransmit_count++;
        send_message(MSG_DATA, seq, data, len);

        fprintf(stderr, "PROTO: waiting for ACK seq=%d...\n", seq);
        rx_message_t reply;
        if (wait_for_msg(MSG_ACK, &reply, ack_timeout_ms) == 0) {
            if (reply.seq == seq) {
                fprintf(stderr, "PROTO: ACK received for seq=%d\n", seq);
                seq++;
                return 0;
            }
            fprintf(stderr, "PROTO: ACK seq mismatch (got %d, want %d)\n",
                    reply.seq, seq);
        }
        fprintf(stderr, "PROTO: ACK timeout, retry %d/%d\n",
                attempt + 1, MAX_RETRIES);
    }

    fprintf(stderr, "PROTO: send failed after %d retries\n", MAX_RETRIES);
    return -1;
}

static int do_send(const char *text)
{
    return do_send_bytes((const uint8_t *)text, strlen(text));
}

/* Decode an ASCII hex string into raw bytes. Allows spaces between bytes.
   Returns number of bytes written, or -1 on malformed input. */
static int hex_decode(const char *hex, uint8_t *out, int max_out)
{
    int n = 0;
    while (*hex && n < max_out) {
        while (*hex == ' ' || *hex == '\t') hex++;
        if (!*hex) break;
        int hi, lo;
        char c = *hex++;
        if      (c >= '0' && c <= '9') hi = c - '0';
        else if (c >= 'a' && c <= 'f') hi = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') hi = c - 'A' + 10;
        else return -1;
        c = *hex++;
        if (!c) return -1;
        if      (c >= '0' && c <= '9') lo = c - '0';
        else if (c >= 'a' && c <= 'f') lo = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') lo = c - 'A' + 10;
        else return -1;
        out[n++] = (uint8_t)((hi << 4) | lo);
    }
    return n;
}

/* ---- Calibrate over the link ---- */

static int do_cal_request(void)
{
    /* We need the peer to hold their IR LED ON so we can find the block.
       Send CAL_REQ, wait for CAL_ACK, then scan grid, send CAL_DONE. */

    fprintf(stderr, "PROTO: requesting calibration...\n");

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        send_message(MSG_CAL_REQ, 0, NULL, 0);

        rx_message_t reply;
        if (wait_for_msg(MSG_CAL_ACK, &reply, ack_timeout_ms) == 0) {
            fprintf(stderr, "PROTO: peer acknowledged, scanning...\n");
            /* Peer is now holding LED ON — run calibration */
            int block = calibrate();
            if (block >= 0) {
                tracked_block = block;
                fprintf(stderr, "PROTO: calibrated to block %d\n", block);
            } else {
                fprintf(stderr, "PROTO: calibration scan failed\n");
            }
            send_message(MSG_CAL_DONE, 0, NULL, 0);
            return block;
        }
        fprintf(stderr, "PROTO: CAL_ACK timeout, retry %d/%d\n",
                attempt + 1, MAX_RETRIES);
    }
    return -1;
}

/* ---- Handle incoming calibration request ---- */

static void handle_cal_request(void)
{
    fprintf(stderr, "PROTO: peer requested calibration, holding LED ON for 5s...\n");
    send_message(MSG_CAL_ACK, 0, NULL, 0);

    /* Hold LED toggling for 5 seconds so peer can calibrate */
    pthread_mutex_lock(&tx_mutex);
    tx_active = 1;
    for (int i = 0; i < 10 && running; i++) {
        gpio_set(1);
        usleep(250000);  /* 250ms ON */
        gpio_set(0);
        usleep(250000);  /* 250ms OFF */
    }
    gpio_set(0);
    tx_active = 0;
    pthread_mutex_unlock(&tx_mutex);

    /* Wait for CAL_DONE */
    rx_message_t msg;
    if (wait_for_msg(MSG_CAL_DONE, &msg, 10000) == 0) {
        fprintf(stderr, "PROTO: peer calibration complete\n");
    } else {
        fprintf(stderr, "PROTO: CAL_DONE timeout\n");
    }
}

/* ================================================================
 *  Interactive mode: RX thread + protocol event loop
 * ================================================================ */

static int interactive_mode(int is_listener)
{
    /* Start RX thread */
    pthread_t rx_tid;
    if (pthread_create(&rx_tid, NULL, rx_thread, NULL) != 0) {
        perror("pthread_create");
        return 1;
    }

    /* Handshake */
    int connected;
    if (is_listener) {
        connected = do_listen();
    } else {
        connected = do_connect();
    }

    if (connected != 0) {
        fprintf(stderr, "PROTO: handshake failed\n");
        running = 0;
        pthread_join(rx_tid, NULL);
        return 1;
    }

    /* Event loop: read stdin commands, handle incoming messages */
    fprintf(stderr, "\nirlink: connected! Commands:\n");
    fprintf(stderr, "  send <text>        — send data message (text, stops at NUL)\n");
    fprintf(stderr, "  send-hex <hex>     — send binary-safe DATA (hex-encoded)\n");
    fprintf(stderr, "  ping               — measure round-trip time\n");
    fprintf(stderr, "  cal                — request peer calibration\n");
    fprintf(stderr, "  stats              — print link counters\n");
    fprintf(stderr, "  quit               — exit\n\n");

    char line[1024];
    while (running && fgets(line, sizeof(line), stdin)) {
        /* Strip newline */
        int len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';

        if (strncmp(line, "send-hex ", 9) == 0) {
            uint8_t buf[MAX_PAYLOAD];
            int nbytes = hex_decode(line + 9, buf, sizeof(buf));
            if (nbytes < 0) {
                fprintf(stderr, "Bad hex: %s\n", line + 9);
            } else {
                do_send_bytes(buf, nbytes);
            }
        } else if (strncmp(line, "send ", 5) == 0) {
            do_send(line + 5);
        } else if (strcmp(line, "stats") == 0) {
            printf("STATS: tx=%d rx=%d crc=%d rtx=%d dll=%d\n",
                   tx_count, rx_count, crc_fail_count,
                   retransmit_count, dpll_loss_count);
            fflush(stdout);
        } else if (strcmp(line, "ping") == 0) {
            int64_t t0 = now_ms();
            send_message(MSG_PING, 0, NULL, 0);
            rx_message_t reply;
            if (wait_for_msg(MSG_PONG, &reply, ack_timeout_ms) == 0) {
                int64_t rtt = now_ms() - t0;
                fprintf(stderr, "PING: pong received, RTT=%lld ms\n", (long long)rtt);
                printf("PONG: RTT=%lld ms\n", (long long)rtt);
                fflush(stdout);
            } else {
                fprintf(stderr, "PING: timeout\n");
            }
        } else if (strcmp(line, "cal") == 0) {
            do_cal_request();
        } else if (strcmp(line, "quit") == 0 || strcmp(line, "exit") == 0) {
            break;
        } else if (len > 0) {
            fprintf(stderr, "Unknown command: %s\n", line);
        }

        /* rx_thread auto-ACKs DATA and auto-PONGs PING. Just drain rx_msg
           here; CAL_REQ still needs main-thread handling because it has to
           hold the LED on for several seconds. */
        pthread_mutex_lock(&rx_mutex);
        if (rx_msg.valid) {
            if (rx_msg.msg_type == MSG_CAL_REQ) {
                rx_msg.valid = 0;
                pthread_mutex_unlock(&rx_mutex);
                handle_cal_request();
            } else {
                rx_msg.valid = 0;
                pthread_mutex_unlock(&rx_mutex);
            }
        } else {
            pthread_mutex_unlock(&rx_mutex);
        }
    }

    running = 0;
    pthread_join(rx_tid, NULL);
    return 0;
}

/* ================================================================
 *  Daemon mode: auto-ACK + handle CAL_REQ, print DATA to stdout
 * ================================================================ */

static int daemon_mode(int is_listener)
{
    pthread_t rx_tid;
    if (pthread_create(&rx_tid, NULL, rx_thread, NULL) != 0) {
        perror("pthread_create");
        return 1;
    }

    /* Handshake */
    int connected;
    if (is_listener) {
        connected = do_listen();
    } else {
        connected = do_connect();
    }

    if (connected != 0) {
        fprintf(stderr, "PROTO: handshake failed\n");
        running = 0;
        pthread_join(rx_tid, NULL);
        return 1;
    }

    fprintf(stderr, "PROTO: connected, entering daemon mode\n");

    /* Event loop: just handle incoming messages */
    while (running) {
        rx_message_t msg;
        if (wait_for_any_msg(&msg, 1000) == 0) {
            /* rx_thread already auto-ACKed DATA and auto-PONGed PING;
               daemon just needs to handle the heavier protocol cases. */
            switch (msg.msg_type) {
            case MSG_CAL_REQ:
                handle_cal_request();
                break;
            case MSG_SYN:
                send_message(MSG_SYN_ACK, 0, NULL, 0);
                break;
            default:
                break;
            }
        }
    }

    pthread_join(rx_tid, NULL);
    return 0;
}

/* ================================================================
 *  Main
 * ================================================================ */

int main(int argc, char *argv[])
{
    /* Unbuffered stderr for real-time logging with nohup/redirect */
    setbuf(stderr, NULL);
    setbuf(stdout, NULL);

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <command> [options]\n", argv[0]);
        fprintf(stderr, "Commands:\n");
        fprintf(stderr, "  listen [--block N] [--speed MS]     — wait for connection\n");
        fprintf(stderr, "  connect [--block N] [--speed MS]    — initiate connection\n");
        fprintf(stderr, "  daemon-listen [--block N] [--speed MS] — listen + auto-respond\n");
        fprintf(stderr, "  daemon-connect [--block N] [--speed MS] — connect + auto-respond\n");
        fprintf(stderr, "  send <text> [--block N] [--speed MS]  — connect, send, exit\n");
        fprintf(stderr, "  calibrate              — manual ROI calibration\n");
        fprintf(stderr, "\nOptions:\n");
        fprintf(stderr, "  --block N    track grid block N (0-59) for ROI\n");
        fprintf(stderr, "  --speed MS   symbol duration in ms (default 333, min 40)\n");
        return 1;
    }

    /* Parse --block, --pixel, and --speed from any position */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--block") == 0 && i + 1 < argc) {
            tracked_block = atoi(argv[i + 1]);
            fprintf(stderr, "irlink: tracking block %d\n", tracked_block);
        } else if (strcmp(argv[i], "--pixel") == 0 && i + 1 < argc) {
            if (sscanf(argv[i + 1], "%d,%d", &pixel_x, &pixel_y) != 2) {
                fprintf(stderr, "irlink: --pixel must be x,y (e.g. --pixel 385,178)\n");
                return 1;
            }
            fprintf(stderr, "irlink: pixel ROI at (%d, %d) size %d\n",
                    pixel_x, pixel_y, pixel_roi_size);
        } else if (strcmp(argv[i], "--roi-size") == 0 && i + 1 < argc) {
            pixel_roi_size = atoi(argv[i + 1]);
            if (pixel_roi_size < 3 || pixel_roi_size > 31) {
                fprintf(stderr, "irlink: --roi-size must be 3-31\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
            symbol_ms = atoi(argv[i + 1]);
            if (symbol_ms < 40 || symbol_ms > 2000) {
                fprintf(stderr, "irlink: --speed must be 40-2000 ms\n");
                return 1;
            }
            fprintf(stderr, "irlink: symbol speed %d ms\n", symbol_ms);
        }
    }

    /* If pixel mode, write ROI config for BrightnessMonitor */
    if (pixel_x >= 0 && pixel_y >= 0) {
        write_roi_config(pixel_x, pixel_y, pixel_roi_size);
        fprintf(stderr, "irlink: ROI config written to %s\n", ROI_CONFIG_PATH);
    }

    /* Scale ACK timeout to symbol speed: 3x max frame round-trip
     * One round-trip:
     *   - max DATA frame (16B app payload, the single-frame ceiling): ~324 sym
     *     (preamble+sync+length+payload+crc Manchester-encoded + postamble)
     *   - max ACK frame (1B seq):                                      ~84 sym
     *   - inter-message gap + AE settle + decode slack                 ~5000 ms
     *
     * Carrier-aware extension in wait_for_msg() pushes this out by up to
     * ACK_HARD_CEILING_X if the rx_thread is still seeing peer carrier — so we
     * keep the BASE timeout tight (faster retransmit on truly dead links) and
     * lean on the extension to cover legitimate slow flows. */
    ack_timeout_ms = 450 * symbol_ms + 5000;
    fprintf(stderr, "irlink: ack timeout %d ms (hard ceiling %d ms via carrier extension)\n",
            ack_timeout_ms, ack_timeout_ms * ACK_HARD_CEILING_X);

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    /* Init GPIO */
    if (gpio_init(IR_GPIO_940, IR_GPIO_850) < 0) {
        fprintf(stderr, "irlink: GPIO init failed (are you root?)\n");
        return 1;
    }
    gpio_set(0);

    const char *cmd = argv[1];

    if (strcmp(cmd, "calibrate") == 0) {
        /* AE stays active during calibration — we need it to stabilize */
        int block = calibrate();
        if (block >= 0) {
            printf("%d\n", block);
            return 0;
        }
        return 1;
    }

    /* All communication modes: freeze AE first, unfreeze on exit */

    if (strcmp(cmd, "listen") == 0) {
        set_ae_freeze(1);
        usleep(1000000); /* 1s for AE to apply final update and settle */
        int ret = interactive_mode(1);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "connect") == 0) {
        set_ae_freeze(1);
        usleep(1000000); /* 1s AE settle */
        int ret = interactive_mode(0);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "daemon-listen") == 0) {
        set_ae_freeze(1);
        usleep(1000000); /* 1s AE settle */
        int ret = daemon_mode(1);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "daemon-connect") == 0) {
        set_ae_freeze(1);
        usleep(1000000); /* 1s AE settle */
        int ret = daemon_mode(0);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "send") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Usage: irlink send <text> [--block N]\n");
            return 1;
        }
        /* Find the text argument (skip --block N and --speed N) */
        const char *text = NULL;
        for (int i = 2; i < argc; i++) {
            if ((strcmp(argv[i], "--block") == 0 || strcmp(argv[i], "--speed") == 0)
                && i + 1 < argc) {
                i++; /* skip value */
                continue;
            }
            text = argv[i];
            break;
        }
        if (!text) {
            fprintf(stderr, "No message text provided\n");
            return 1;
        }

        set_ae_freeze(1);
        usleep(1000000); /* 1s AE settle */

        /* Start RX, connect, send, done */
        pthread_t rx_tid;
        pthread_create(&rx_tid, NULL, rx_thread, NULL);

        if (do_connect() == 0) {
            do_send(text);
        }

        running = 0;
        pthread_join(rx_tid, NULL);
        set_ae_freeze(0);
        return 0;
    }

    if (strcmp(cmd, "tx") == 0) {
        /* Raw TX: blast a Manchester DATA frame with no handshake or RX.
           For use with host-side pixel_rx which decodes from the RTSP stream. */
        if (argc < 3) {
            fprintf(stderr, "Usage: irlink tx <text> [--speed N]\n");
            return 1;
        }
        const char *text = NULL;
        for (int i = 2; i < argc; i++) {
            if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
                i++;
                continue;
            }
            if (strcmp(argv[i], "--block") == 0 && i + 1 < argc) {
                i++;
                continue;
            }
            text = argv[i];
            break;
        }
        if (!text) {
            fprintf(stderr, "No message text provided\n");
            return 1;
        }

        set_ae_freeze(1);
        usleep(1000000);

        int len = strlen(text);
        if (len > MAX_PAYLOAD - 2) len = MAX_PAYLOAD - 2;

        fprintf(stderr, "TX: raw send \"%s\" (%d bytes, speed=%dms)\n",
                text, len, symbol_ms);
        send_message(MSG_DATA, 1, (const uint8_t *)text, len);
        fprintf(stderr, "TX: done\n");

        set_ae_freeze(0);
        return 0;
    }

    fprintf(stderr, "Unknown command: %s\n", cmd);
    return 1;
}
