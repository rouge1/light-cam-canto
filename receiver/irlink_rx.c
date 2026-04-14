/*
 * irlink_rx — On-camera IR receiver for Manchester-encoded frames.
 *
 * Reads brightness values from /run/prudynt/brightness (written by
 * the BrightnessMonitor patch in prudynt-t), detects IR LED
 * transitions, decodes Manchester symbols, and outputs decoded messages.
 *
 * Protocol: [PREAMBLE 10101010] [SYNC 11001011] [LEN 8] [PAYLOAD N*8] [CRC8] [POSTAMBLE 1010]
 *           All Manchester-encoded (2 symbols per data bit).
 *
 * Usage: irlink_rx [symbol_duration_ms]
 *        Default symbol duration: 333 ms (matches tx_pwm Phase 2)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>

#define BRIGHTNESS_PATH     "/run/prudynt/brightness"
#define GRID_PATH           "/run/prudynt/brightness_grid"
#define POLL_INTERVAL_US    15000   /* 15ms — faster than frame rate */
#define MAX_SAMPLES         8192
#define MAX_SYMBOLS         4096
#define MAX_PAYLOAD         255
#define SETTLE_MS           1500    /* quiet time after TX — must exceed trailer */
#define MIN_BRIGHTNESS_DELTA 3      /* minimum delta to consider "active" */
#define GRID_BLOCKS         60      /* 10x6 grid */

/* Sync word: 11001011 */
static const uint8_t SYNC_WORD[] = {1,1,0,0,1,0,1,1};
#define SYNC_LEN 8

/* CRC-8/CCITT polynomial 0x07 */
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

/* ---- Sample buffer ---- */

typedef struct {
    int64_t ts_ms;
    uint8_t brightness;
} sample_t;

static sample_t samples[MAX_SAMPLES];
static int n_samples = 0;

static volatile int running = 1;

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* ---- Read brightness file ---- */

/* ---- Read grid file: returns all block values ---- */

static int read_grid(int64_t *ts_ms, uint8_t *blocks, int max_blocks)
{
    char buf[512];
    int fd = open(GRID_PATH, O_RDONLY);
    if (fd < 0)
        return -1;
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return -1;
    buf[n] = '\0';

    long long ts;
    char *p = buf;
    if (sscanf(p, "%lld", &ts) != 1)
        return -1;
    *ts_ms = (int64_t)ts;

    /* Skip past timestamp */
    while (*p && *p != ' ') p++;

    int count = 0;
    while (*p && count < max_blocks) {
        unsigned val;
        while (*p == ' ') p++;
        if (sscanf(p, "%u", &val) != 1)
            break;
        blocks[count++] = (uint8_t)(val > 255 ? 255 : val);
        while (*p && *p != ' ' && *p != '\n') p++;
    }
    return count;
}

/* ---- Tracked block index for ROI (-1 = use whole-frame mean) ---- */

static int tracked_block = -1;

static int read_brightness(int64_t *ts_ms, uint8_t *brightness)
{
    char buf[64];
    int fd = open(BRIGHTNESS_PATH, O_RDONLY);
    if (fd < 0)
        return -1;
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return -1;
    buf[n] = '\0';

    long long ts;
    unsigned val, max_val;
    /* If we have a tracked block, read grid and compute residual (block - mean).
       The residual cancels out AE compensation that shifts all blocks equally.
       We scale: residual = block - mean + 128 (so 128 = no difference). */
    if (tracked_block >= 0) {
        uint8_t blocks[GRID_BLOCKS];
        int64_t grid_ts;
        int nblocks = read_grid(&grid_ts, blocks, GRID_BLOCKS);
        if (nblocks > tracked_block) {
            /* Compute frame mean */
            int sum = 0;
            for (int i = 0; i < nblocks; i++) sum += blocks[i];
            int mean = sum / nblocks;
            /* Residual: how much this block differs from mean */
            int residual = (int)blocks[tracked_block] - mean + 128;
            if (residual < 0) residual = 0;
            if (residual > 255) residual = 255;
            *ts_ms = grid_ts;
            *brightness = (uint8_t)residual;
            return 0;
        }
    }

    /* Otherwise use mean from brightness file */
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

/* ---- Get monotonic time in ms ---- */

static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* ---- Manchester decode: symbol pairs -> data bits ---- */

static int manchester_decode(const uint8_t *symbols, int n_sym, uint8_t *bits)
{
    if (n_sym % 2 != 0)
        return -1;
    int n_bits = 0;
    for (int i = 0; i < n_sym; i += 2) {
        if (symbols[i] == 1 && symbols[i+1] == 0)
            bits[n_bits++] = 0;     /* falling edge = 0 */
        else if (symbols[i] == 0 && symbols[i+1] == 1)
            bits[n_bits++] = 1;     /* rising edge = 1 */
        else
            return -1;              /* invalid pair */
    }
    return n_bits;
}

/* ---- Extract byte from bit array (MSB first) ---- */

static uint8_t bits_to_byte(const uint8_t *bits)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++)
        val = (val << 1) | bits[i];
    return val;
}

/* ---- Parse frame from data bits ---- */

static int parse_frame(const uint8_t *bits, int n_bits, char *msg, int msg_size)
{
    /* Find sync word */
    int sync_idx = -1;
    for (int i = 0; i <= n_bits - SYNC_LEN; i++) {
        int match = 1;
        for (int j = 0; j < SYNC_LEN; j++) {
            if (bits[i+j] != SYNC_WORD[j]) {
                match = 0;
                break;
            }
        }
        if (match) {
            sync_idx = i + SYNC_LEN;
            break;
        }
    }

    if (sync_idx < 0) {
        fprintf(stderr, "RX: sync word not found\n");
        return -1;
    }

    int remaining = n_bits - sync_idx;
    const uint8_t *data = bits + sync_idx;

    /* Need at least length(8) + 1 byte payload(8) + CRC(8) = 24 bits */
    if (remaining < 24) {
        fprintf(stderr, "RX: too few bits after sync (%d)\n", remaining);
        return -1;
    }

    uint8_t length = bits_to_byte(data);
    if (length == 0) {
        fprintf(stderr, "RX: invalid length %d\n", length);
        return -1;
    }

    int needed = 8 + length * 8 + 8;  /* length + payload + CRC */
    if (remaining < needed) {
        fprintf(stderr, "RX: need %d bits, have %d\n", needed, remaining);
        return -1;
    }

    /* Extract payload bytes */
    uint8_t payload[MAX_PAYLOAD];
    uint8_t crc_data[MAX_PAYLOAD + 1];
    crc_data[0] = length;
    for (int i = 0; i < length; i++) {
        payload[i] = bits_to_byte(data + 8 + i * 8);
        crc_data[i + 1] = payload[i];
    }

    uint8_t received_crc = bits_to_byte(data + 8 + length * 8);
    uint8_t expected_crc = crc8(crc_data, length + 1);

    if (received_crc != expected_crc) {
        fprintf(stderr, "RX: CRC mismatch (got 0x%02x, expected 0x%02x)\n",
                received_crc, expected_crc);
        return -1;
    }

    /* Copy to output */
    int copy_len = length < msg_size - 1 ? length : msg_size - 1;
    memcpy(msg, payload, copy_len);
    msg[copy_len] = '\0';

    return length;
}

/* ---- Decode collected samples into a message ---- */

static int decode_samples(sample_t *samp, int n, int symbol_ms, char *msg, int msg_size)
{
    if (n < 4)
        return -1;

    /* Find brightness range */
    uint8_t bmin = 255, bmax = 0;
    for (int i = 0; i < n; i++) {
        if (samp[i].brightness < bmin) bmin = samp[i].brightness;
        if (samp[i].brightness > bmax) bmax = samp[i].brightness;
    }

    int delta = bmax - bmin;
    if (delta < MIN_BRIGHTNESS_DELTA) {
        fprintf(stderr, "RX: brightness delta too small (%d)\n", delta);
        return -1;
    }

    /* Adaptive threshold: midpoint of range */
    uint8_t threshold = bmin + delta / 2;

    fprintf(stderr, "RX: %d samples, brightness %d-%d, threshold %d, delta %d\n",
            n, bmin, bmax, threshold, delta);

    /* Convert to binary symbols using threshold */
    /* First, find the start of transmission: first significant change from baseline */
    uint8_t baseline = samp[0].brightness;
    int tx_start = -1;
    for (int i = 1; i < n; i++) {
        int diff = (int)samp[i].brightness - (int)baseline;
        if (diff < 0) diff = -diff;
        if (diff >= delta / 2) {
            /* Back up a few samples to catch the very start */
            tx_start = i > 2 ? i - 2 : 0;
            break;
        }
    }

    if (tx_start < 0) {
        fprintf(stderr, "RX: could not find TX start\n");
        return -1;
    }

    int64_t t0 = samp[tx_start].ts_ms;

    /* Sample at symbol boundaries: average all samples in the middle 50% of each symbol */
    uint8_t symbols[MAX_SYMBOLS];
    int n_sym = 0;

    for (int sym = 0; sym < MAX_SYMBOLS; sym++) {
        /* Average samples in the middle 50% of each symbol window */
        int64_t win_start = t0 + (int64_t)symbol_ms * sym + symbol_ms / 4;
        int64_t win_end   = t0 + (int64_t)symbol_ms * sym + symbol_ms * 3 / 4;

        int sum = 0, count = 0;
        for (int i = tx_start; i < n; i++) {
            if (samp[i].ts_ms >= win_start && samp[i].ts_ms <= win_end) {
                sum += samp[i].brightness;
                count++;
            }
            if (samp[i].ts_ms > win_end)
                break;
        }

        if (count == 0)
            break;  /* No more samples in range */

        int avg = sum / count;
        symbols[n_sym++] = avg >= threshold ? 1 : 0;
    }

    fprintf(stderr, "RX: extracted %d symbols from tx_start=%d (t0=%lld)\n",
            n_sym, tx_start, (long long)t0);

    /* Dump first symbols for debugging */
    if (n_sym > 0) {
        fprintf(stderr, "RX: symbols: ");
        for (int i = 0; i < n_sym && i < 120; i++)
            fprintf(stderr, "%d", symbols[i]);
        fprintf(stderr, "\n");
    }

    if (n_sym < 20)  /* Minimum: preamble(16) + sync(16) = 32 Manchester symbols */
        return -1;

    /* Try decoding with several trim offsets (brute force alignment) */
    for (int trim_start = 0; trim_start < 12; trim_start++) {
        for (int trim_end = 0; trim_end < 12; trim_end++) {
            int sym_count = n_sym - trim_start - trim_end;
            if (sym_count < 20 || sym_count % 2 != 0)
                continue;

            uint8_t bits[MAX_SYMBOLS / 2];
            int n_bits = manchester_decode(symbols + trim_start, sym_count, bits);
            if (n_bits < 0)
                continue;

            int ret = parse_frame(bits, n_bits, msg, msg_size);
            if (ret > 0) {
                fprintf(stderr, "RX: decoded with trim [%d:%d]\n", trim_start, trim_end);
                return ret;
            }
        }
    }

    /* Try inverted polarity */
    for (int i = 0; i < n_sym; i++)
        symbols[i] = symbols[i] ? 0 : 1;

    fprintf(stderr, "RX: trying inverted polarity\n");

    for (int trim_start = 0; trim_start < 12; trim_start++) {
        for (int trim_end = 0; trim_end < 12; trim_end++) {
            int sym_count = n_sym - trim_start - trim_end;
            if (sym_count < 20 || sym_count % 2 != 0)
                continue;

            uint8_t bits[MAX_SYMBOLS / 2];
            int n_bits = manchester_decode(symbols + trim_start, sym_count, bits);
            if (n_bits < 0)
                continue;

            int ret = parse_frame(bits, n_bits, msg, msg_size);
            if (ret > 0) {
                fprintf(stderr, "RX: decoded with inverted polarity, trim [%d:%d]\n",
                        trim_start, trim_end);
                return ret;
            }
        }
    }

    fprintf(stderr, "RX: decode failed\n");
    return -1;
}

/* ---- Calibrate: find which grid block has highest variance ---- */
/* Called with --calibrate: reads grid for a few seconds, finds the block
   with the most variation (= the IR LED spot) and prints its index. */

static int calibrate(void)
{
    fprintf(stderr, "CALIBRATE: reading grid for 5 seconds...\n");
    fprintf(stderr, "CALIBRATE: toggle the transmitter LED during this time!\n");

    /* Collect all grid snapshots to compute per-block correlation with mean */
    #define MAX_CAL_FRAMES 200
    uint8_t frames[MAX_CAL_FRAMES][GRID_BLOCKS];
    uint8_t means[MAX_CAL_FRAMES];
    int n_frames = 0;

    int64_t start = now_ms();
    int64_t last_ts = 0;

    while (now_ms() - start < 5000 && n_frames < MAX_CAL_FRAMES) {
        int64_t ts;
        uint8_t blocks[GRID_BLOCKS];
        int n = read_grid(&ts, blocks, GRID_BLOCKS);
        if (n > 0 && ts != last_ts) {
            last_ts = ts;
            memcpy(frames[n_frames], blocks, GRID_BLOCKS);
            /* Compute mean */
            int sum = 0;
            for (int i = 0; i < n; i++) sum += blocks[i];
            means[n_frames] = sum / n;
            n_frames++;
        }
        usleep(POLL_INTERVAL_US);
    }

    fprintf(stderr, "CALIBRATE: %d grid frames captured\n", n_frames);
    if (n_frames < 10) {
        fprintf(stderr, "CALIBRATE: too few frames\n");
        return -1;
    }

    /* For each block, compute how much it DIFFERS from the mean's movement.
       When AE darkens the frame, all blocks get darker — but the IR block
       gets LESS dark (because IR light partially offsets). We want the block
       whose brightness change is most OPPOSITE to the mean. */

    /* Compute: for each block, the average of (block[i] - mean[i]) across frames.
       Then find the block where this residual has the HIGHEST variance. */
    int best = -1;
    int best_range = 0;
    int block_residual_min[GRID_BLOCKS];
    int block_residual_max[GRID_BLOCKS];

    for (int b = 0; b < GRID_BLOCKS; b++) {
        int rmin = 999, rmax = -999;
        for (int f = 0; f < n_frames; f++) {
            int residual = (int)frames[f][b] - (int)means[f];
            if (residual < rmin) rmin = residual;
            if (residual > rmax) rmax = residual;
        }
        block_residual_min[b] = rmin;
        block_residual_max[b] = rmax;
        int range = rmax - rmin;
        if (range > best_range) {
            best_range = range;
            best = b;
        }
    }

    if (best < 0 || best_range < 3) {
        fprintf(stderr, "CALIBRATE: no IR block detected (max residual range=%d)\n", best_range);
        return -1;
    }

    int row = best / 10;
    int col = best % 10;
    fprintf(stderr, "CALIBRATE: best block=%d (row=%d, col=%d), residual range=%d\n",
            best, row, col, best_range);

    /* Print grid of residual ranges */
    fprintf(stderr, "CALIBRATE: residual range grid (10x6) — IR block stands out:\n");
    for (int r = 0; r < 6; r++) {
        fprintf(stderr, "  ");
        for (int c = 0; c < 10; c++) {
            int idx = r * 10 + c;
            int range = block_residual_max[idx] - block_residual_min[idx];
            if (idx == best)
                fprintf(stderr, "[%3d]", range);
            else
                fprintf(stderr, " %3d ", range);
        }
        fprintf(stderr, "\n");
    }

    printf("%d\n", best);
    return best;
}

/* ---- Main: poll brightness, detect TX, decode ---- */

int main(int argc, char *argv[])
{
    int symbol_ms = 333;  /* Default: Phase 2 timing */
    int do_calibrate = 0;
    int block_arg = -1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--calibrate") == 0) {
            do_calibrate = 1;
        } else if (strcmp(argv[i], "--block") == 0 && i + 1 < argc) {
            block_arg = atoi(argv[++i]);
        } else {
            symbol_ms = atoi(argv[i]);
            if (symbol_ms < 50 || symbol_ms > 5000) {
                fprintf(stderr, "Usage: %s [symbol_ms] [--calibrate] [--block N]\n", argv[0]);
                return 1;
            }
        }
    }

    if (do_calibrate) {
        int result = calibrate();
        return result >= 0 ? 0 : 1;
    }

    if (block_arg >= 0 && block_arg < GRID_BLOCKS) {
        tracked_block = block_arg;
        fprintf(stderr, "irlink_rx: tracking block %d (row=%d, col=%d)\n",
                block_arg, block_arg / 10, block_arg % 10);
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    fprintf(stderr, "irlink_rx: listening (symbol=%dms, poll=%dms)\n",
            symbol_ms, POLL_INTERVAL_US / 1000);

    /* State machine:
     * IDLE -> ACTIVE (brightness changes beyond threshold)
     * ACTIVE -> DECODE (brightness stable for SETTLE_MS)
     */
    enum { IDLE, ACTIVE } state = IDLE;
    uint8_t baseline = 0;
    int baseline_set = 0;
    int64_t last_change_ms = 0;
    int64_t last_ts = 0;  /* last brightness file timestamp, to avoid duplicates */

    n_samples = 0;

    while (running) {
        int64_t ts_ms;
        uint8_t brightness;

        if (read_brightness(&ts_ms, &brightness) < 0) {
            usleep(POLL_INTERVAL_US);
            continue;
        }

        /* Skip if same timestamp (no new frame yet) */
        if (ts_ms == last_ts) {
            usleep(POLL_INTERVAL_US);
            continue;
        }
        last_ts = ts_ms;

        if (!baseline_set) {
            baseline = brightness;
            baseline_set = 1;
            last_change_ms = now_ms();
        }

        int diff = (int)brightness - (int)baseline;
        if (diff < 0) diff = -diff;

        switch (state) {
        case IDLE:
            if (diff >= MIN_BRIGHTNESS_DELTA) {
                /* Transmission starting! */
                fprintf(stderr, "RX: activity detected (baseline=%d, now=%d, delta=%d)\n",
                        baseline, brightness, diff);
                state = ACTIVE;
                n_samples = 0;

                /* Record a few baseline samples for context */
                samples[n_samples].ts_ms = ts_ms - symbol_ms;
                samples[n_samples].brightness = baseline;
                n_samples++;
            } else {
                /* Update baseline with exponential moving average */
                baseline = (uint8_t)((int)baseline * 7 / 8 + (int)brightness * 1 / 8);
            }
            break;

        case ACTIVE:
            /* Record sample */
            if (n_samples < MAX_SAMPLES) {
                samples[n_samples].ts_ms = ts_ms;
                samples[n_samples].brightness = brightness;
                n_samples++;
            }

            if (diff >= MIN_BRIGHTNESS_DELTA) {
                last_change_ms = now_ms();
            }

            /* Check if quiet for long enough → end of transmission */
            if (now_ms() - last_change_ms > SETTLE_MS) {
                fprintf(stderr, "RX: end of transmission (%d samples collected)\n", n_samples);

                char msg[MAX_PAYLOAD + 1];
                int ret = decode_samples(samples, n_samples, symbol_ms, msg, sizeof(msg));
                if (ret > 0) {
                    /* Print decoded message to stdout */
                    printf("MSG: %s\n", msg);
                    fflush(stdout);
                    fprintf(stderr, "RX: >>> \"%s\" (%d bytes) <<<\n", msg, ret);
                } else {
                    fprintf(stderr, "RX: could not decode transmission\n");
                }

                /* Reset state */
                state = IDLE;
                baseline = brightness;
                last_change_ms = now_ms();
                n_samples = 0;
            }
            break;
        }

        usleep(POLL_INTERVAL_US);
    }

    fprintf(stderr, "irlink_rx: stopped\n");
    return 0;
}
