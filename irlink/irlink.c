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
#include <stdarg.h>
#include <sys/stat.h>

/* ---- Configuration ---- */

#define IR_GPIO_940         49
#define IR_GPIO_850         47
#define BRIGHTNESS_PATH     "/run/prudynt/brightness"
#define GRID_PATH           "/run/prudynt/brightness_grid"
#define AE_FREEZE_PATH      "/run/prudynt/ae_freeze"
#define ROI_CONFIG_PATH     "/run/prudynt/roi_config"
#define ROI_PATH            "/run/prudynt/brightness_roi"
#define POLL_INTERVAL_US    5000
#define MAX_SAMPLES         16384  /* ~15 min RX cap at 18 fps; was 8192
                                      (~7.5 min) which truncated payloads
                                      ≥128 B at 160 ms/sym mid-frame */
#define MAX_SYMBOLS         4096
#define MAX_PAYLOAD         255
#define SETTLE_MS           400     /* quiet time to detect end of TX (longer = fewer false splits) */
#define MIN_BRIGHTNESS_DELTA 5
#define GRID_BLOCKS         240     /* 20x12 grid (32x30 pixel blocks) */
#define MAX_RETRIES         3

/* Runtime-configurable speed — updated at runtime by rate adaptation. */
static int symbol_ms = 160;        /* default: 160ms/symbol (~3 bps after Manchester, ~3 frames/symbol at 18fps) */

/* Computed from symbol_ms — recomputed on every rate change via recompute_ack_timeout(). */
static int ack_timeout_ms = 60000;

/* ---- Adaptive symbol rate ladder (mirrors host/config.py RATE_LADDER_MS) ---- */
static const int RATE_LADDER_MS[] = {60, 80, 100, 120, 160, 200};
#define RATE_LADDER_LEN ((int)(sizeof(RATE_LADDER_MS) / sizeof(RATE_LADDER_MS[0])))

static int current_rung = 3;            /* default to 120ms (index 3) until --speed sets it */
static int success_at_rate = 0;         /* consecutive ACKed DATA sends at current rate */
static int fail_at_rate = 0;            /* consecutive retransmit-exhausted sends at current rate */
static int probe_in_progress = 0;       /* true while testing a faster rate post-probe-up */
static int pre_probe_rung = -1;         /* rung to restore on failed probe */
static int64_t last_valid_frame_ms = 0; /* for split-brain recovery */
/* Separate debounce for split-brain itself, so that triggering split-brain
   doesn't artificially "reset" last_valid_frame_ms — that conflates "we
   recently decoded a real frame" (used by the grid-watchdog's idle gate)
   with "split-brain shouldn't immediately refire." Without this split,
   firing split-brain bumps last_valid_frame_ms forward, which delays the
   watchdog's idle re-arm by another full IDLE_THRESHOLD_MS. */
static int64_t last_split_brain_fire_ms = 0;
/* Last time we saw a "substantial" failed decode — i.e. a candidate frame
   long enough to plausibly be a real Manchester transmission, not just a
   2-sample ambient brightness drift blip. Split-brain recovery requires
   this to be RECENT before dropping rate. Without this guard, daemon-listen
   sitting idle for hours would accumulate per-second 2-sample drift events
   that fail to decode, eventually trip split-brain, and silently drop the
   rate to 200ms — leaving the daemon mismatched against the next 160ms
   peer that connects. Threshold of 30 samples ≈ 1.6s of carrier at 18fps,
   long enough to be a real frame attempt, short enough to catch genuine
   rate-mismatch (where peer's full 16s frame produces 100+ samples). */
static int64_t last_substantial_decode_fail_ms = 0;
#define SUBSTANTIAL_DECODE_SAMPLES 30

/* Set to 1 while a CAL_REQ/CAL_ACK/CAL_DONE flow is in flight. Suppresses
   the rx_thread split-brain recovery, which would otherwise trip mid-cal:
   a long CAL_DONE wait or multi-attempt CAL_ACK retry can exceed the
   2 × ack_timeout_ms window even though both peers are still alive at the
   same rate. The split-brain recovery dropped this side to 200 ms while
   the peer stayed at 160 ms — a silent rate desync that deadlocked
   bicall phase 2. See irlink/CLAUDE.md "Bicall split-brain" pitfall. */
static volatile int cal_flow_active = 0;
static int64_t failed_probe_cooldown_ms[RATE_LADDER_LEN] = {0};

/* Lock: stay at the slowest rung during startup + the first cal flow, so that
   a half-decoded handshake or one missed RATE_CHANGE can't trip split-brain
   before either side has cal-derived confidence in the pixel. Set in
   daemon_mode at startup, cleared by the first successful cal (handle_cal_request,
   handle_monocal_request, or handle_watchdog_trigger). While set, maybe_probe_up
   short-circuits — peer-driven RATE_CHANGE still applies (explicit peer commands
   are always honored). The helper is defined later (after log_event). */
static volatile int rate_locked_at_floor = 0;
static void unlock_rate_after_cal(const char *reason);
static void lock_rate_at_floor(const char *mode);

#define PROBE_UP_AFTER          5       /* successful ACKs before probing faster */
#define FALLBACK_AFTER          3       /* retransmit-exhausted sends before stepping slower */
#define SPLIT_BRAIN_TIMEOUT_MS  15000
#define PROBE_COOLDOWN_MS       30000

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
#define MSG_RATE_CHANGE 0x0A

/* App-layer message-type byte (sits inside MSG_DATA payload). Mirrors
   protocol/app.py. We define it here so do_cal_request_internal can ship
   a CAL_VISUAL right after CAL_DONE as a single 15-byte DATA frame. */
#define APP_CAL_VISUAL  0x0E

/* ---- Sync word ---- */

static const uint8_t SYNC_WORD[] = {1,1,0,0,1,0,1,1};
#define SYNC_LEN 8

/* ---- Resync framing (mirrors protocol/frame.py) ----
 * Long protocol frames at 200ms/sym (CAL_REQ/CAL_ACK/CAL_DONE) overrun the
 * DPLL's drift budget — extracted symbol stream slips, Manchester pairs go
 * invalid, decode fails. Resync framing fixes this by injecting raw 1010...
 * blocks every chunk_syms data symbols. The DPLL re-locks on each block's
 * edges as cleanly as on the initial preamble.
 *
 * Wire layout (after preamble+sync, all in symbol space):
 *     [CHUNK chunk_syms data] [RESYNC resync_syms 1010...] [CHUNK ...] ...
 *     [CHUNK final ≤ chunk_syms]   ← no trailing resync
 *
 * Receiver finds SYNC at the symbol level (Manchester-encoded SYNC pattern),
 * strips resync blocks at chunk_syms intervals, then Manchester-decodes.
 *
 * Defaults match protocol/frame.py. chunk_syms=48 = 24 bits = 3 bytes per
 * chunk, alignment-safe for byte-boundary CRC. */
#define DEFAULT_CHUNK_SYMS   48
#define DEFAULT_RESYNC_SYMS  16

/* SYNC_WORD `11001011` Manchester-encoded (1→01, 0→10): 16 symbols. */
static const uint8_t SYNC_MANCHESTER[16] = {
    0,1, 0,1, 1,0, 1,0, 0,1, 1,0, 0,1, 0,1
};

/* ---- CAL_DONE payload compression (6 bytes → 2 bytes) ----
 *
 * 4-px quantization. payload[0]=x/4, payload[1]=y/4. Re-centered on RX as
 * x=p0*4+2, y=p1*4+2. Max round-trip error ±2 px per axis — well inside the
 * 15-px ROI tolerance, so the cal pixel is effectively unchanged in practice.
 *
 * Cuts CAL_DONE wire time from ~50s (248 syms with resync) to ~30s (152 syms),
 * which significantly reduces DPLL drift exposure on the longest frame in the
 * protocol. peak_brightness and grid_delta are dropped — they were
 * informational only (already published via /run/irlink-status.json on the
 * scanner side, fetchable out-of-band via /x/cal-status.cgi).
 *
 * Both peers must run the new binary; no version bit. The decoder accepts
 * data_len >= 2 as the success path; data_len == 0 still means scan failed. */
#define CAL_DONE_PAYLOAD_LEN  2
#define CAL_DONE_QUANT_PX     4

static void pack_cal_done(int x, int y, uint8_t payload[CAL_DONE_PAYLOAD_LEN])
{
    int qx = x / CAL_DONE_QUANT_PX;
    int qy = y / CAL_DONE_QUANT_PX;
    if (qx < 0) qx = 0;
    if (qx > 255) qx = 255;
    if (qy < 0) qy = 0;
    if (qy > 255) qy = 255;
    payload[0] = (uint8_t)qx;
    payload[1] = (uint8_t)qy;
}

static void unpack_cal_done(const uint8_t *payload, int *x, int *y)
{
    *x = (int)payload[0] * CAL_DONE_QUANT_PX + CAL_DONE_QUANT_PX / 2;
    *y = (int)payload[1] * CAL_DONE_QUANT_PX + CAL_DONE_QUANT_PX / 2;
}

/* ---- Globals ---- */

static volatile int running = 1;
static volatile int tx_active = 0;  /* suppress RX during TX */
static int tracked_block = -1;
static int pixel_x = -1, pixel_y = -1;  /* pixel-level ROI center (-1 = disabled) */
/* Default 7×7 (49 cells). The IR LED spot at typical cam-cam distance is
   ~5×5 pixels, so a 7×7 average has ~50% lit cells and reads Δ≈110 when
   the peer holds LEDs solid (vs Δ≈37 with the old 15×15 default — the
   larger window dilutes the spot across mostly-dark cells and pushes
   Manchester decode below the noise margin). Override per-invocation
   with --roi-size if your geometry differs. Range 3-31. */
static int pixel_roi_size = 7;
/* Reverse-monocal mode (Phase 2 of bidirectional cal): we are the SCANNER,
   peer is the HOLDER. Set via `irlink monocal --reverse`. CAL_REQ payload[0]
   stays flags=0 (regular cal — peer holds, we scan); the responder routes
   that to its existing handle_cal_request(0). Only the requestor's local
   role inverts; cam1 needs no code change to support this. */
static int monocal_reverse = 0;
/* Fused bidirectional mode (`--both`): run Phase 1 (we hold, peer scans)
   then Phase 2 (we scan, peer holds) inside ONE irlink session — same
   handshake, same rx_thread, same AE-freeze epoch. Saves the ~55 s second
   handshake the old "two separate triggers" approach paid. Cam1 needs no
   change: it just sees two consecutive CAL_REQs and handles each via its
   existing dispatch (peer_scans=1 → handle_monocal_request, then flags=0
   → handle_cal_request). Profiled: 295 s → ~200 s (~32% faster). */
static int monocal_both = 0;
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

/* forward decls — these are all defined later in the file but used by
   prof_mark / status_on_exit which appear up here. */
static void status_set(const char *state, const char *event);
static int64_t now_ms(void);
static int64_t status_started_ms;  /* tentative; real definition further down */

/* ================================================================
 *  Profiler — emits PROF lines on stderr with absolute (since irlink
 *  launch) and relative (since last PROF) ms deltas. Compiled in
 *  unconditionally because it's tiny; turn off by not parsing the
 *  PROF: lines on the laptop side. status_started_ms is set in main()
 *  before the first prof_mark could fire, so the "+ms" is always
 *  meaningful.
 * ================================================================ */
static int64_t prof_last_ms = 0;
static void prof_mark(const char *tag)
{
    int64_t now = now_ms();
    int64_t since_start = now - status_started_ms;
    int64_t since_last  = (prof_last_ms == 0) ? 0 : (now - prof_last_ms);
    fprintf(stderr, "PROF +%lldms (+%lldms): %s\n",
            (long long)since_start, (long long)since_last, tag);
    prof_last_ms = now;
}

/* atexit() handler: leave /run/irlink-status.json in a sane state so
   the webui doesn't show stale in-flight state (e.g. "send/connected"
   from a HELLO test) after irlink has actually exited. Without this,
   the JSON keeps the last state forever and only the proc.irlink=0
   sentinel in the all-status CGI keeps the webui honest — fine, but
   confuses anyone reading the JSON directly. Fires on normal return
   and on SIGTERM (sighandler sets running=0, main loop exits, atexit
   runs); does NOT fire on SIGKILL (use this for `daemon-listen` kills
   which currently need `kill -9`). */
static void status_on_exit(void)
{
    /* status_set takes the same mutex as the heartbeat thread; safe
       to call here because the heartbeat thread is daemonized and
       libc's atexit happens after all other threads are gone in
       practice (and even if not, the mutex serializes us). */
    status_set("exited", "irlink stopped");
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

/* ---- Adaptive symbol rate helpers ---- */

static int rung_for_rate_ms(int ms)
{
    /* Best-effort: exact match preferred; otherwise clamp to closest rung by value. */
    int best_idx = 0;
    int best_diff = 1 << 30;
    for (int i = 0; i < RATE_LADDER_LEN; i++) {
        int d = RATE_LADDER_MS[i] - ms;
        if (d < 0) d = -d;
        if (d < best_diff) {
            best_diff = d;
            best_idx = i;
        }
    }
    return best_idx;
}

static void recompute_ack_timeout(void)
{
    /* Same formula as startup: 450 * symbol_ms + 5000. See comment in main(). */
    ack_timeout_ms = 450 * symbol_ms + 5000;
}

static void apply_rate(int new_ms, int new_rung, const char *reason)
{
    symbol_ms = new_ms;
    current_rung = new_rung;
    recompute_ack_timeout();
    fprintf(stderr, "RATE: ms=%d rung=%d reason=%s\n", new_ms, new_rung, reason);
    /* Also surface on stdout so orchestrators can log it. */
    printf("RATE: ms=%d rung=%d reason=%s\n", new_ms, new_rung, reason);
    fflush(stdout);
}

/* ---- Append-only event log (post-mortem diagnostic ring) ----
 *
 * Tagged one-line records appended to /tmp/irlink-events.log (tmpfs, doesn't
 * survive reboots — that's intentional, the overlay is tiny). Self-truncates
 * to zero when over EVENT_LOG_MAX so the file caps at ~64 KB without rotation
 * machinery. Scrape post-mortem with `ssh dacamN cat /tmp/irlink-events.log`.
 */

#define EVENT_LOG_PATH "/tmp/irlink-events.log"
#define EVENT_LOG_MAX  (64 * 1024)
static pthread_mutex_t evlog_mutex = PTHREAD_MUTEX_INITIALIZER;

static void log_event(const char *tag, const char *fmt, ...)
{
    pthread_mutex_lock(&evlog_mutex);
    int flags = O_WRONLY | O_CREAT | O_APPEND;
    struct stat st;
    if (stat(EVENT_LOG_PATH, &st) == 0 && st.st_size > EVENT_LOG_MAX)
        flags |= O_TRUNC;
    int fd = open(EVENT_LOG_PATH, flags, 0644);
    if (fd >= 0) {
        char line[256];
        int n = snprintf(line, sizeof(line), "%lld %s ",
                         (long long)now_ms(), tag);
        if (n > 0 && n < (int)sizeof(line)) {
            va_list ap;
            va_start(ap, fmt);
            int m = vsnprintf(line + n, sizeof(line) - n, fmt, ap);
            va_end(ap);
            if (m > 0) n += m;
            if (n >= (int)sizeof(line) - 1) n = (int)sizeof(line) - 2;
            line[n++] = '\n';
            (void)write(fd, line, n);
        }
        close(fd);
    }
    pthread_mutex_unlock(&evlog_mutex);
}

/* Lock helpers for the rate floor — defined here because they call log_event.
   The forward declarations live up by the rate-state vars. */

/* Force the slowest rung and lock probe-up until first successful cal. Called
   from main() at the entry of every communication mode (listen/connect/
   daemon-listen/daemon-connect/monocal) so both peers start at the same rate
   regardless of --speed. Without the lock, cam1's daemon-listen at 200 vs
   cam2's monocal at 160 would silently fail SYN decode. */
/* Floor at 160ms (rung 4). 160ms is where SYN/SYN_ACK round-trips actually
   decode reliably — testing 2026-05-10 confirmed 200ms still fails the
   round-trip even with ROI 7 (cam1 sends SYN_ACK, cam2 can't decode it).
   So the floor stays at 160ms for actual messaging.

   Split-brain divergence (the previous failure mode where cam1 dropped
   to 200ms while cam2 stayed at 160ms) is eliminated by making
   split-brain recovery ALSO drop to the floor (160ms) instead of the
   slowest rung — see the split-brain block in the watchdog/recovery
   thread. Both cams converge at the same rate on recovery. */
/* Floor tried at LEN-3 (120ms) on 2026-05-14 with fused MONOCAL BOTH —
   SYN handshake decode failed reliably, the run fell through to bootstrap
   fallback every time. Confirms memory's warning: 120ms × 18fps =
   ~2.2 samples/sym is too marginal for DPLL Manchester decode at the
   current grid_delta ≈ 37-52. Reverted to LEN-2 (160ms) which has ~2.7
   samples/sym headroom. The path to 120ms is to first improve SNR
   (stronger cal, tighter ROI) or move the rate negotiation in-session
   AFTER handshake completes (handshake at 160, cal frames at 120 via
   RATE_CHANGE). For now: 160 is the validated reliable floor. */
#define RATE_FLOOR_RUNG (RATE_LADDER_LEN - 2)
static void lock_rate_at_floor(const char *mode)
{
    int floor_ms = RATE_LADDER_MS[RATE_FLOOR_RUNG];
    if (symbol_ms != floor_ms) {
        fprintf(stderr,
            "RATE: %s startup — clamping %dms → %dms (floor) until first cal\n",
            mode, symbol_ms, floor_ms);
    }
    apply_rate(floor_ms, RATE_FLOOR_RUNG, "startup-floor");
    rate_locked_at_floor = 1;
    log_event("RATE", "locked-at-floor ms=%d mode=%s", floor_ms, mode);
}

static void unlock_rate_after_cal(const char *reason)
{
    if (rate_locked_at_floor) {
        rate_locked_at_floor = 0;
        fprintf(stderr, "RATE: probe-up unlocked (reason=%s)\n", reason);
        log_event("RATE", "unlocked-after-cal reason=%s", reason);
    }
}

/* ---- AE freeze control (via prudynt BrightnessMonitor) ----
 *
 * `ae_freeze_intent` tracks the most recent value irlink wrote, so the
 * heartbeat thread can detect drift caused by external writers (cam_setup.sh,
 * aim_assist preflight, manual SSH echo, the webui CAL worker) and restore.
 * -1 means "no opinion yet" (pre-mode-init); the heartbeat doesn't enforce
 * in that state. cal_flow_active gates enforcement so do_calibrate_pixel_to
 * can legitimately drop AE mid-flow without the heartbeat fighting it.
 */

static volatile int ae_freeze_intent = -1;

static int read_ae_freeze_file(void)
{
    int fd = open(AE_FREEZE_PATH, O_RDONLY);
    if (fd < 0) return -1;
    char ch;
    int n = read(fd, &ch, 1);
    close(fd);
    if (n != 1) return -1;
    return (ch == '1') ? 1 : 0;
}

static void write_ae_freeze_file(int freeze)
{
    int fd = open(AE_FREEZE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
        write(fd, freeze ? "1" : "0", 1);
        close(fd);
    }
}

static void set_ae_freeze(int freeze)
{
    ae_freeze_intent = freeze ? 1 : 0;
    write_ae_freeze_file(freeze);
    fprintf(stderr, "AE: freeze %s\n", freeze ? "ON" : "OFF");
    log_event("AE", "freeze=%d", freeze);
}

/* ================================================================
 *  Status JSON snapshot
 *
 * Writes a small JSON file the laptop can poll via HTTP (through a CGI
 * script in /var/www/x/) instead of SSHing into the camera. Atomic via
 * write-then-rename so HTTP readers never see partial JSON. Updated at
 * key protocol events (handshake, cal phase, rate change, error).
 * ================================================================ */

#define STATUS_PATH       "/run/irlink-status.json"
#define STATUS_TMP_PATH   "/run/irlink-status.json.tmp"
#define STATUS_EVENT_LEN  96

static int64_t status_started_ms = 0;
static char status_mode[24] = "init";        /* daemon-listen|connect|listen|... */
static char status_state[32] = "starting";   /* idle|connected|cal_holding|cal_scanning|... */
static char status_event[STATUS_EVENT_LEN] = "";  /* short last-event description */
static int64_t status_event_ms = 0;
static int status_pixel_x = -1, status_pixel_y = -1;
static int status_cal_phase = 0;             /* 0=none, 1=phase 1, 2=phase 2 */
static int status_cal_bidi = 0;
static int status_cal_retries = 0;
static int status_peak_x = -1, status_peak_y = -1;
static int status_peak_brightness = 0, status_peak_delta = 0, status_peak_block = -1;
/* Removed status_tx_count / status_rx_count / status_decode_fail —
   they were declared as separate aggregates but never incremented anywhere.
   write_irlink_status now reads tx_count / rx_count / crc_fail_count
   directly, which are the real counters maintained by send_message and
   rx_thread. Without this, /api/{cam}/status reported decode_fail:0
   forever even while rx_thread was actively failing decodes. */
/* Mirrors of rx_thread locals — written by rx_thread, read by
   write_irlink_status. Lets the laptop see "is cam1's ROI seeing peer
   IR right now?" via /api/cam1/status without SSHing for stderr. */
static volatile uint8_t status_rx_brightness = 0;
static volatile uint8_t status_rx_baseline = 0;
static volatile int     status_rx_active = 0;   /* 0=IDLE, 1=ACTIVE */

/* protect concurrent write_status() from rx_thread + main thread */
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;

static void write_irlink_status(void)
{
    pthread_mutex_lock(&status_mutex);
    int ae_freeze_val = -1;
    int fdr = open(AE_FREEZE_PATH, O_RDONLY);
    if (fdr >= 0) {
        char ch;
        if (read(fdr, &ch, 1) == 1) ae_freeze_val = (ch == '1') ? 1 : 0;
        close(fdr);
    }
    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"ts_ms\":%lld,"
        "\"started_ms\":%lld,"
        "\"mode\":\"%s\","
        "\"state\":\"%s\","
        "\"pixel\":[%d,%d],"
        "\"rate_ms\":%d,"
        "\"rung\":%d,"
        "\"ae_frozen\":%d,"
        "\"cal\":{"
            "\"active\":%d,"
            "\"phase\":%d,"
            "\"bidi\":%d,"
            "\"retries\":%d,"
            "\"peak\":[%d,%d],"
            "\"peak_brightness\":%d,"
            "\"peak_delta\":%d,"
            "\"peak_block\":%d"
        "},"
        "\"counters\":{"
            "\"tx\":%d,"
            "\"rx\":%d,"
            "\"decode_fail\":%d,"
            "\"retransmit\":%d,"
            "\"dpll_loss\":%d"
        "},"
        /* Diagnostic snapshot of rx_thread state. Lets the laptop see via
           HTTP whether cam1's pixel ROI is seeing the peer's IR (carrier
           recent → ROI is on-target) and whether decodes are succeeding
           (valid_frame recent → Manchester/DPLL OK). When carrier ticks
           but valid_frame stays at 0, the SYN is reaching cam1 but the
           decode is failing — points at rate/AE/sample-density issues
           rather than aim/ROI issues. */
        "\"rx\":{"
            "\"state\":\"%s\","
            "\"brightness\":%d,"
            "\"baseline\":%d,"
            "\"diff\":%d,"
            "\"last_carrier_ms\":%lld,"
            "\"last_valid_frame_ms\":%lld"
        "},"
        "\"last_event\":\"%s\","
        "\"last_event_ms\":%lld"
        "}\n",
        (long long)now_ms(),
        (long long)status_started_ms,
        status_mode, status_state,
        status_pixel_x, status_pixel_y,
        symbol_ms, current_rung,
        ae_freeze_val,
        cal_flow_active, status_cal_phase, status_cal_bidi, status_cal_retries,
        status_peak_x, status_peak_y,
        status_peak_brightness, status_peak_delta, status_peak_block,
        /* status_tx_count / status_rx_count / status_decode_fail were
           declared as separate aggregates but never incremented anywhere —
           the real counters live in rx_count / tx_count / crc_fail_count
           (rx_thread + send_message). Read those directly so the webapi
           stops reporting `decode_fail:0` when decodes are actually failing.
           retransmit_count + dpll_loss_count added so the JSON exposes the
           same counter set as SSH `STATS:` — operators monitoring via
           webapi can now diagnose flaky-link (retransmit ticks) vs
           DPLL-struggle (dpll_loss ticks) without needing SSH. */
        tx_count, rx_count, crc_fail_count, retransmit_count, dpll_loss_count,
        status_rx_active ? "ACTIVE" : "IDLE",
        (int)status_rx_brightness, (int)status_rx_baseline,
        (int)status_rx_brightness - (int)status_rx_baseline,
        (long long)last_carrier_ms,
        (long long)last_valid_frame_ms,
        status_event,
        (long long)status_event_ms);
    if (n > 0 && n < (int)sizeof(buf)) {
        int fd = open(STATUS_TMP_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            ssize_t w = write(fd, buf, n);
            close(fd);
            if (w == n)
                rename(STATUS_TMP_PATH, STATUS_PATH);
        }
    }
    pthread_mutex_unlock(&status_mutex);
}

/* Periodic re-write of /run/irlink-status.json so ts_ms ticks even when no
   protocol event fires. Diagnostic-only — without this, a daemon stuck in
   any steady state (e.g. "listening", or worse, a wedged "monocal_resp_done")
   looks identical to a daemon that's actively servicing requests. With this
   running, frozen ts_ms across two polls is a clear hang signal. */
static void *heartbeat_thread(void *unused)
{
    (void)unused;
    while (running) {
        usleep(1000 * 1000);  /* 1 Hz */
        /* AE-drift defense: if some external writer flipped /run/prudynt/ae_freeze
           away from what we last set, restore it. Skipped while a cal flow is
           live, since do_calibrate_pixel_to legitimately drops AE mid-flow.
           This catches the failure mode where (e.g.) a webui CAL worker shells
           `echo 0 > /run/prudynt/ae_freeze` and our daemon-listen rx_thread
           then can't decode incoming SYNs — without this, we'd have no way to
           recover except a manual restart. */
        if (ae_freeze_intent >= 0 && !cal_flow_active) {
            int actual = read_ae_freeze_file();
            if (actual >= 0 && actual != ae_freeze_intent) {
                log_event("AE", "drift file=%d intent=%d, restoring",
                          actual, ae_freeze_intent);
                fprintf(stderr, "AE: drift detected (file=%d, want=%d), restoring\n",
                        actual, ae_freeze_intent);
                write_ae_freeze_file(ae_freeze_intent);
            }
        }
        write_irlink_status();
    }
    return NULL;
}

/* Update the (state, last_event) tuple and re-write the status file. The
   event message is truncated to STATUS_EVENT_LEN-1 chars and JSON-escaped
   minimally (only quote/backslash). */
static void status_set(const char *state, const char *event)
{
    if (state) {
        strncpy(status_state, state, sizeof(status_state) - 1);
        status_state[sizeof(status_state) - 1] = '\0';
    }
    if (event) {
        /* minimal JSON escaping: replace " and \ with _ to keep things simple */
        size_t i = 0;
        for (const char *p = event; *p && i < STATUS_EVENT_LEN - 1; p++) {
            char c = *p;
            if (c == '"' || c == '\\') c = '_';
            if (c == '\n' || c == '\r' || c == '\t') c = ' ';
            status_event[i++] = c;
        }
        status_event[i] = '\0';
        status_event_ms = now_ms();
    }
    write_irlink_status();
    log_event("STATE", "%s | %s",
              state ? state : status_state,
              event ? event : "");
}

/* ================================================================
 *  Monocal status JSON (/run/monocal-status.json)
 *  ----------------------------------------------------------------
 *  Independent from /run/irlink-status.json. Written only by the
 *  `monocal` subcommand (cam2 / requestor). Atomic via temp+rename.
 *  Webui polls /x/monocal-status.cgi on cam2 to drive the M1..M5
 *  step stack. Race tolerated — readers see HTTP 503 / empty body
 *  during the brief rename window and retry.
 * ================================================================ */

#define MONOCAL_STATUS_PATH       "/run/monocal-status.json"
#define MONOCAL_STATUS_TMP_PATH   "/run/monocal-status.json.tmp"

static int64_t monocal_started_ms = 0;
static int64_t monocal_updated_ms = 0;
static int64_t monocal_ended_ms = 0;
static char    monocal_state[40] = "idle";
static int     monocal_ok = -1;   /* -1=null (running/init), 0=false, 1=true */
static char    monocal_error[STATUS_EVENT_LEN] = "";
static int64_t monocal_ack_recv_ms = 0;
static int64_t monocal_hold_start_ms = 0;
static int64_t monocal_hold_end_ms = 0;
static int     monocal_have_peer_pixel = 0;
static int     monocal_peer_x = 0, monocal_peer_y = 0;
static int     monocal_peer_b = 0, monocal_peer_d = 0;

static pthread_mutex_t monocal_status_mutex = PTHREAD_MUTEX_INITIALIZER;

static void write_monocal_status(void)
{
    pthread_mutex_lock(&monocal_status_mutex);
    monocal_updated_ms = now_ms();

    char peer_pixel[96];
    if (monocal_have_peer_pixel) {
        snprintf(peer_pixel, sizeof(peer_pixel),
                 "{\"x\":%d,\"y\":%d,\"peak_b\":%d,\"grid_delta\":%d}",
                 monocal_peer_x, monocal_peer_y, monocal_peer_b, monocal_peer_d);
    } else {
        snprintf(peer_pixel, sizeof(peer_pixel), "null");
    }

    char ok_str[8];
    if (monocal_ok < 0)      snprintf(ok_str, sizeof(ok_str), "null");
    else if (monocal_ok == 0) snprintf(ok_str, sizeof(ok_str), "false");
    else                      snprintf(ok_str, sizeof(ok_str), "true");

    char ended_str[32];
    if (monocal_ended_ms == 0) snprintf(ended_str, sizeof(ended_str), "null");
    else snprintf(ended_str, sizeof(ended_str), "%lld", (long long)monocal_ended_ms);

    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"schema\":1,"
        "\"state\":\"%s\","
        "\"started_ms\":%lld,"
        "\"updated_ms\":%lld,"
        "\"ended_ms\":%s,"
        "\"ok\":%s,"
        "\"error\":\"%s\","
        "\"ack_received_ms\":%lld,"
        "\"led_hold_started_ms\":%lld,"
        "\"led_hold_ended_ms\":%lld,"
        "\"peer_pixel\":%s,"
        "\"speed_ms\":%d"
        "}\n",
        monocal_state,
        (long long)monocal_started_ms,
        (long long)monocal_updated_ms,
        ended_str,
        ok_str,
        monocal_error,
        (long long)monocal_ack_recv_ms,
        (long long)monocal_hold_start_ms,
        (long long)monocal_hold_end_ms,
        peer_pixel,
        symbol_ms);

    if (n > 0 && n < (int)sizeof(buf)) {
        int fd = open(MONOCAL_STATUS_TMP_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            ssize_t w = write(fd, buf, n);
            close(fd);
            if (w == n)
                rename(MONOCAL_STATUS_TMP_PATH, MONOCAL_STATUS_PATH);
        }
    }
    pthread_mutex_unlock(&monocal_status_mutex);
}

static void monocal_set_state(const char *state, const char *err)
{
    if (state) {
        strncpy(monocal_state, state, sizeof(monocal_state) - 1);
        monocal_state[sizeof(monocal_state) - 1] = '\0';
    }
    if (err) {
        size_t i = 0;
        for (const char *p = err; *p && i < STATUS_EVENT_LEN - 1; p++) {
            char c = *p;
            if (c == '"' || c == '\\') c = '_';
            if (c == '\n' || c == '\r' || c == '\t') c = ' ';
            monocal_error[i++] = c;
        }
        monocal_error[i] = '\0';
    }
    write_monocal_status();
    /* Mirror status_set: emit a STATE event so the events log reflects
     * cam2's progression through monocal_* / monocal_rev_* states. Without
     * this, cam2's events log only sees the few status_set calls in main()
     * (starting/connected/exited) — leaving operator-facing logs and the
     * webui event stack with ~3 entries per run vs cam1's ~10. */
    log_event("STATE", "%s | %s",
              state ? state : monocal_state,
              err ? err : "");
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

/* Resync-framed encoder: same frame contents as build_frame_symbols, but
 * splits the data portion into chunks separated by raw 1010... resync blocks.
 * Header (preamble+sync) Manchester-encoded as one piece (no resync inside);
 * data (length+payload+crc+postamble) Manchester-encoded then chunked.
 * Output is ~33% longer than classic for the same payload — overhead bought
 * by reliable decode of long frames at the slowest rate rung. */
static int build_frame_symbols_resync(uint8_t msg_type, uint8_t seq,
                                       const uint8_t *data, int data_len,
                                       uint8_t *symbols)
{
    /* Header: preamble + sync, Manchester-encoded together. */
    uint8_t header_bits[16];
    uint8_t preamble[] = {1,0,1,0,1,0,1,0};
    memcpy(header_bits, preamble, 8);
    memcpy(header_bits + 8, SYNC_WORD, 8);
    uint8_t header_syms[32];
    int n_header = encode_manchester(header_bits, 16, header_syms);

    /* Data bits: length + payload + CRC + postamble. */
    uint8_t data_bits[MAX_SYMBOLS];
    int nb = 0;
    uint8_t frame_len = 2 + data_len;
    byte_to_bits(frame_len, data_bits + nb); nb += 8;

    uint8_t payload[MAX_PAYLOAD];
    payload[0] = msg_type;
    payload[1] = seq;
    if (data_len > 0)
        memcpy(payload + 2, data, data_len);
    for (int i = 0; i < frame_len; i++) {
        byte_to_bits(payload[i], data_bits + nb);
        nb += 8;
    }

    uint8_t crc_data[MAX_PAYLOAD + 1];
    crc_data[0] = frame_len;
    memcpy(crc_data + 1, payload, frame_len);
    uint8_t crc = crc8(crc_data, frame_len + 1);
    byte_to_bits(crc, data_bits + nb); nb += 8;

    uint8_t postamble[] = {1,0,1,0};
    memcpy(data_bits + nb, postamble, 4); nb += 4;

    /* Manchester-encode data. */
    uint8_t data_syms[MAX_SYMBOLS];
    int n_data = encode_manchester(data_bits, nb, data_syms);

    /* Header + chunked data with resync blocks between chunks. */
    int out = 0;
    memcpy(symbols + out, header_syms, n_header); out += n_header;
    int i = 0;
    while (i < n_data) {
        int chunk = (n_data - i > DEFAULT_CHUNK_SYMS) ? DEFAULT_CHUNK_SYMS
                                                       : (n_data - i);
        memcpy(symbols + out, data_syms + i, chunk);
        out += chunk;
        i += chunk;
        if (i < n_data) {
            for (int k = 0; k < DEFAULT_RESYNC_SYMS; k++)
                symbols[out++] = (k & 1) ? 0 : 1;  /* alternating 1010... */
        }
    }
    return out;
}

/* Per-msg-type gate: which messages use resync framing. CAL_REQ/CAL_ACK/CAL_DONE
 * are the ones that empirically blow the DPLL drift budget at 200ms/sym
 * (CAL_REQ at 120 syms saw mid-frame Manchester pair invalidation; SYN/SYN_ACK
 * at 104 syms work classic). RATE_CHANGE has 2 bytes payload = 120 syms = same
 * danger zone, include for safety. SYN/ACK/PING/PONG stay classic — they're
 * short enough to dodge drift and changing them would be a flag day. */
static int frame_uses_resync(uint8_t msg_type)
{
    switch (msg_type) {
    case MSG_CAL_REQ:
    case MSG_CAL_ACK:
    case MSG_CAL_DONE:
    case MSG_RATE_CHANGE:
    case MSG_DATA:
        return 1;
    default:
        return 0;
    }
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
    int n;
    if (frame_uses_resync(msg_type)) {
        n = build_frame_symbols_resync(msg_type, seq, data, data_len, symbols);
    } else {
        n = build_frame_symbols(msg_type, seq, data, data_len, symbols);
    }

    const char *type_names[] = {
        [MSG_SYN] = "SYN", [MSG_SYN_ACK] = "SYN_ACK", [MSG_ACK] = "ACK",
        [MSG_DATA] = "DATA", [MSG_CAL_REQ] = "CAL_REQ",
        [MSG_CAL_ACK] = "CAL_ACK", [MSG_CAL_DONE] = "CAL_DONE",
        [MSG_PING] = "PING", [MSG_PONG] = "PONG",
        [MSG_RATE_CHANGE] = "RATE_CHANGE"
    };
    const char *name = (msg_type <= MSG_RATE_CHANGE) ? type_names[msg_type] : "???";
    fprintf(stderr, "TX: %s seq=%d (%d symbols)\n", name, seq, n);
    log_event("TX", "%s seq=%d len=%d sym=%d", name, seq, data_len, n);

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

static int try_parse_resync(const uint8_t *symbols, int n_sym,
                             rx_message_t *msg);  /* fwd decl */

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

            /* Resync-framing fallback: same DPLL extraction succeeded; the
             * raw symbol stream may contain mid-frame resync blocks that
             * the strict trim+manchester path can't recover from. */
            if (try_parse_resync(symbols, n_sym, msg)) return 0;
        }
    }

    return -1;
}

/* Try parse with resync framing — search for SYNC at the symbol level (allows
 * up to 1 bit error in the 16-sym pattern for noise tolerance), strip resync
 * blocks at chunk_syms intervals, Manchester-decode the result, prepend the
 * known SYNC bits, and call parse_frame.
 *
 * Returns 1 on success (msg populated), 0 on no decode. Tries both polarities.
 * Caller is responsible for the symbol stream `symbols` having at least the
 * preamble+sync+chunk window present (~50+ symbols). */
static int try_parse_resync(const uint8_t *symbols, int n_sym, rx_message_t *msg)
{
    if (n_sym < 32) return 0;  /* too short for even header+1-byte data */

    for (int inv = 0; inv < 2; inv++) {
        for (int sync_pos = 0; sync_pos + 16 <= n_sym; sync_pos++) {
            /* Hamming distance of SYNC pattern at this position. Allow ≤1
             * mismatch — extracted symbols can be 1 bit off from threshold
             * jitter at low SNR. Tighter than that adds false positives. */
            int errs = 0;
            for (int j = 0; j < 16 && errs <= 1; j++) {
                uint8_t s = symbols[sync_pos + j];
                if (inv) s = !s;
                if (s != SYNC_MANCHESTER[j]) errs++;
            }
            if (errs > 1) continue;

            /* Strip resync blocks: keep DEFAULT_CHUNK_SYMS, skip
             * DEFAULT_RESYNC_SYMS, repeat. */
            uint8_t stripped[MAX_SYMBOLS];
            int out = 0;
            int pos = sync_pos + 16;
            while (pos < n_sym && out < MAX_SYMBOLS) {
                int take = DEFAULT_CHUNK_SYMS;
                if (pos + take > n_sym) take = n_sym - pos;
                if (out + take > MAX_SYMBOLS) take = MAX_SYMBOLS - out;
                for (int k = 0; k < take; k++) {
                    uint8_t s = symbols[pos + k];
                    stripped[out++] = inv ? !s : s;
                }
                pos += DEFAULT_CHUNK_SYMS;
                if (pos < n_sym) pos += DEFAULT_RESYNC_SYMS;
            }

            /* Try Manchester decode at multiple even-length truncations.
             * The strip may leave a stray sym at the tail. */
            for (int trim_e = 0; trim_e <= 4 && trim_e < out; trim_e++) {
                int sc = out - trim_e;
                if (sc < 16 || sc % 2 != 0) continue;
                uint8_t bits[MAX_SYMBOLS / 2 + 8];
                /* Prepend SYNC bits so parse_frame's sync search succeeds. */
                memcpy(bits, SYNC_WORD, SYNC_LEN);
                int nb = manchester_decode(stripped, sc, bits + SYNC_LEN);
                if (nb < 0) continue;
                if (parse_frame(bits, SYNC_LEN + nb, msg) > 0) {
                    fprintf(stderr,
                        "RX: resync decoded (sync_pos=%d errs=%d trim_e=%d%s)\n",
                        sync_pos, errs, trim_e, inv ? " inv" : "");
                    return 1;
                }
            }
        }
    }
    return 0;
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

    /* Last resort: resync framing. Symbols are currently inverted from the
     * loop above; try_parse_resync handles both polarities itself, so flip
     * back to the original first. */
    for (int i = 0; i < n_sym; i++) symbols[i] = symbols[i] ? 0 : 1;
    if (try_parse_resync(symbols, n_sym, msg)) return 0;

    return -1;
}

/* ---- Calibration: AE freeze + raw delta (LED OFF vs LED ON) ---- */

typedef struct {
    int block_idx;     /* peak block in 20x12 grid */
    int baseline_val;  /* mean brightness at peak block, LED OFF */
    int active_val;    /* mean brightness at peak block, LED ON */
    int delta;         /* active - baseline */
    /* 4x4 sub-area of grid deltas centered on peak block (peak at cell (1,1)).
       Quantized 0..15 (4-bit). Used as the over-light CAL_VISUAL payload. */
    uint8_t zoom_4x4[16];
} grid_cal_t;

/* Grid-based coarse calibration: AE freeze, baseline scan, prompt for LED on,
   active scan, find peak block. Leaves AE FROZEN — caller must unfreeze. */
static int do_grid_calibration(grid_cal_t *out)
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
        return -1;
    }

    for (int i = 0; i < GRID_BLOCKS; i++)
        led_on[i] /= n_led;

    fprintf(stderr, "CALIBRATE: got %d LED-ON frames\n", n_led);

    /* Step 3: Find block with biggest positive delta (LED ON - LED OFF) */
    int deltas[GRID_BLOCKS];
    for (int b = 0; b < GRID_BLOCKS; b++)
        deltas[b] = led_on[b] - baseline[b];

    /* OSD masking — channel 1 carries a per-second timestamp + name +
       uptime in the top 30 px and a "thingino" watermark in the bottom
       30 px. Their per-frame character changes inject false delta into
       block rows 0 (by=0) and 11 (by=11). The watermark also bleeds
       upward into row 10 (observed 2026-05-10: watchdog wrote saved cal
       (17,302) — row 10 — when only rows 0/11 were masked). Without
       masking, argmax can pick an OSD-edge block (e.g. block 21 at
       y=30-59) over the true TX block. Mirrors cal_procedure.py:206-209
       (extended down one row) so on-camera and laptop scans converge. */
    for (int bx = 0; bx < 20; bx++) {
        deltas[0  * 20 + bx] = 0;   /* by=0,  y∈[0,29]    — OSD strip       */
        deltas[10 * 20 + bx] = 0;   /* by=10, y∈[300,329] — watermark bleed */
        deltas[11 * 20 + bx] = 0;   /* by=11, y∈[330,359] — watermark       */
    }

    int best = -1, best_delta = 0;
    for (int b = 0; b < GRID_BLOCKS; b++) {
        if (deltas[b] > best_delta) {
            best_delta = deltas[b];
            best = b;
        }
    }

    /* Dump full delta grid to /run/grid-deltas.json (atomic temp+rename).
       Served by /x/grid-deltas.cgi so the laptop can fetch the post-mask
       deltas + best block + runner-up over HTTP — no SSH needed. Lets
       the next round of monocal-misbehavior debugging see exactly which
       block won and what the runner-up was without redeploying. */
    {
        const char *tmp = "/run/grid-deltas.json.tmp";
        const char *path = "/run/grid-deltas.json";
        int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            char buf[64];
            int n;
            n = snprintf(buf, sizeof(buf),
                         "{\"ts_ms\":%lld,\"best_block\":%d,\"best_delta\":%d,\"deltas\":[",
                         (long long)now_ms(), best, best_delta);
            (void)write(fd, buf, n);
            for (int i = 0; i < GRID_BLOCKS; i++) {
                n = snprintf(buf, sizeof(buf),
                             (i == 0) ? "%d" : ",%d", deltas[i]);
                (void)write(fd, buf, n);
            }
            (void)write(fd, "]}\n", 3);
            close(fd);
            (void)rename(tmp, path);
        }
    }

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

    out->block_idx    = best;
    out->baseline_val = baseline[best];
    out->active_val   = led_on[best];
    out->delta        = best_delta;

    /* 4x4 sub-area of grid deltas centered on peak block. Peak at cell (1,1).
       Off-grid cells stay 0. Quantize each delta to 0..15 by /16. */
    int peak_bx = best % 20;
    int peak_by = best / 20;
    int origin_bx = peak_bx - 1;
    int origin_by = peak_by - 1;
    for (int i = 0; i < 16; i++) out->zoom_4x4[i] = 0;
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            int gx = origin_bx + c;
            int gy = origin_by + r;
            if (gx >= 0 && gx < 20 && gy >= 0 && gy < 12) {
                int d = deltas[gy * 20 + gx] / 16;
                if (d < 0) d = 0;
                if (d > 15) d = 15;
                out->zoom_4x4[r * 4 + c] = (uint8_t)d;
            }
        }
    }
    return 0;
}

static int calibrate(void)
{
    grid_cal_t cal;
    int rc = do_grid_calibration(&cal);
    set_ae_freeze(0);
    if (rc < 0) return -1;
    printf("%d\n", cal.block_idx);
    return cal.block_idx;
}

/* ---- Pixel-level refinement: stride-5 ROI sweep within a grid block ---- */

/* Wait for a fresh ROI sample (timestamp != prev_ts). */
static int read_roi_fresh(int64_t prev_ts, int64_t *out_ts, uint8_t *out_b, int timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline) {
        int64_t ts;
        uint8_t b;
        if (read_roi(&ts, &b) == 0 && ts != prev_ts) {
            *out_ts = ts;
            *out_b = b;
            return 0;
        }
        usleep(POLL_INTERVAL_US);
    }
    return -1;
}

/* Sample N fresh ROI frames and return the mean. */
static int sample_roi_avg(int n_frames, uint8_t *out_avg)
{
    int sum = 0, count = 0;
    int64_t last_ts = 0;
    for (int i = 0; i < n_frames; i++) {
        int64_t ts;
        uint8_t b;
        if (read_roi_fresh(last_ts, &ts, &b, 300) < 0) break;
        sum += b;
        count++;
        last_ts = ts;
    }
    if (count == 0) return -1;
    *out_avg = (uint8_t)(sum / count);
    return 0;
}

/* Sample one ROI position (5x5, averaged over 3 fresh frames after a 2-frame
   discard for the ROI change to take effect). Updates best_* if brighter. */
static void scan_pos(int cx, int cy, int *best_x, int *best_y, uint8_t *best_b)
{
    write_roi_config(cx, cy, 5);
    int64_t junk_ts = 0;
    uint8_t junk_b;
    read_roi_fresh(0, &junk_ts, &junk_b, 300);
    read_roi_fresh(junk_ts, &junk_ts, &junk_b, 300);

    uint8_t avg;
    if (sample_roi_avg(3, &avg) == 0) {
        fprintf(stderr, "  (%3d,%3d) = %3d\n", cx, cy, avg);
        if ((int)avg > (int)*best_b) {
            *best_b = avg;
            *best_x = cx;
            *best_y = cy;
        }
    }
}

/* Frame bounds for ROI size 5: center must be in [2, w-3] × [2, h-3].
   Frame is 640x360 → cx ∈ [2,637], cy ∈ [2,357]. */
#define ROI_CX_MIN  2
#define ROI_CX_MAX  637
#define ROI_CY_MIN  2
#define ROI_CY_MAX  357
#define MAX_EDGE_EXPANSIONS 4

/* Find the peak pixel near a grid block via stride-5 ROI sweep. Starts inside
   the given block; if the peak lands on the swept rectangle's edge, expands
   by one stride in that direction and rescans only the new positions. Caps
   expansions to bound runtime when the peak is degenerate (e.g. uniform). */
static int find_peak_pixel_in_block(int block_idx,
                                     int *out_x, int *out_y,
                                     uint8_t *out_brightness)
{
    int bx = block_idx % 20;
    int by = block_idx / 20;

    /* Inclusive bounds of swept ROI-center positions */
    int xmin = bx * 32 + 2;
    int xmax = bx * 32 + 27;
    int ymin = by * 30 + 2;
    int ymax = by * 30 + 27;

    fprintf(stderr, "PIXEL-CAL: initial sweep block %d (x=%d..%d, y=%d..%d)\n",
            block_idx, xmin, xmax, ymin, ymax);

    int best_x = -1, best_y = -1;
    uint8_t best_b = 0;

    for (int y = ymin; y <= ymax; y += 5) {
        for (int x = xmin; x <= xmax; x += 5)
            scan_pos(x, y, &best_x, &best_y, &best_b);
    }
    if (best_x < 0) return -1;

    /* Edge-peak refinement loop. */
    for (int iter = 0; iter < MAX_EDGE_EXPANSIONS; iter++) {
        int new_xmin = xmin, new_xmax = xmax;
        int new_ymin = ymin, new_ymax = ymax;

        if (best_x == xmin && xmin > ROI_CX_MIN)
            new_xmin = (xmin - 5 < ROI_CX_MIN) ? ROI_CX_MIN : xmin - 5;
        if (best_x == xmax && xmax < ROI_CX_MAX)
            new_xmax = (xmax + 5 > ROI_CX_MAX) ? ROI_CX_MAX : xmax + 5;
        if (best_y == ymin && ymin > ROI_CY_MIN)
            new_ymin = (ymin - 5 < ROI_CY_MIN) ? ROI_CY_MIN : ymin - 5;
        if (best_y == ymax && ymax < ROI_CY_MAX)
            new_ymax = (ymax + 5 > ROI_CY_MAX) ? ROI_CY_MAX : ymax + 5;

        if (new_xmin == xmin && new_xmax == xmax &&
            new_ymin == ymin && new_ymax == ymax)
            break;  /* peak interior — done */

        fprintf(stderr, "PIXEL-CAL: peak at (%d,%d)=%d on edge → expand to "
                        "x=%d..%d, y=%d..%d\n",
                best_x, best_y, best_b,
                new_xmin, new_xmax, new_ymin, new_ymax);

        /* Scan only newly-added positions. Top/bottom strips span the full
           expanded x range; left/right strips span only the OLD y range so
           corners aren't double-scanned. */
        if (new_ymin < ymin) {
            for (int y = new_ymin; y < ymin; y += 5)
                for (int x = new_xmin; x <= new_xmax; x += 5)
                    scan_pos(x, y, &best_x, &best_y, &best_b);
        }
        if (new_ymax > ymax) {
            for (int y = ymax + 5; y <= new_ymax; y += 5)
                for (int x = new_xmin; x <= new_xmax; x += 5)
                    scan_pos(x, y, &best_x, &best_y, &best_b);
        }
        if (new_xmin < xmin) {
            for (int x = new_xmin; x < xmin; x += 5)
                for (int y = ymin; y <= ymax; y += 5)
                    scan_pos(x, y, &best_x, &best_y, &best_b);
        }
        if (new_xmax > xmax) {
            for (int x = xmax + 5; x <= new_xmax; x += 5)
                for (int y = ymin; y <= ymax; y += 5)
                    scan_pos(x, y, &best_x, &best_y, &best_b);
        }

        xmin = new_xmin; xmax = new_xmax;
        ymin = new_ymin; ymax = new_ymax;
    }

    *out_x = best_x;
    *out_y = best_y;
    *out_brightness = best_b;
    return 0;
}

/* Return saved peak_brightness from /opt/etc/calibration.json, or -1 if the
 * file is missing / unreadable / unparseable. Used by handle_watchdog_trigger
 * to refuse to overwrite a strong cal with a much weaker one — e.g. previous
 * watchdog runs that latched onto a desk/floor reflection (peak_b=158) and
 * blew away a direct-LED cal (peak_b=235). */
static int read_saved_peak_brightness(void)
{
    int fd = open("/opt/etc/calibration.json", O_RDONLY);
    if (fd < 0) return -1;
    char buf[1024];
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return -1;
    buf[n] = '\0';
    const char *p = strstr(buf, "\"peak_brightness\"");
    if (!p) return -1;
    p = strchr(p, ':');
    if (!p) return -1;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    int v = atoi(p);
    if (v < 0 || v > 255) return -1;
    return v;
}

/* Read saved tx_pixel from /opt/etc/calibration.json into out_x / out_y.
 * Returns 0 on success, -1 if missing/unreadable/unparseable. Used by the
 * watchdog guard to reject far pixel relocations that lack a strong-beacon
 * justification (a static reflection reads as bright as a real LED in a
 * single ON snapshot, so absolute brightness can't tell them apart —
 * distance-from-a-trusted-cal can). */
static int read_saved_pixel(int *out_x, int *out_y)
{
    int fd = open("/opt/etc/calibration.json", O_RDONLY);
    if (fd < 0) return -1;
    char buf[1024];
    int n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return -1;
    buf[n] = '\0';
    const char *p = strstr(buf, "\"tx_pixel\"");
    if (!p) return -1;
    p = strchr(p, '[');
    if (!p) return -1;
    p++;
    int x = atoi(p);
    const char *c = strchr(p, ',');
    if (!c) return -1;
    int y = atoi(c + 1);
    if (x < 0 || x > 639 || y < 0 || y > 359) return -1;
    *out_x = x;
    *out_y = y;
    return 0;
}

static void write_calibration_json(int tx_x, int tx_y, int off_val, int on_val,
                                    int grid_delta, int peak_b)
{
    mkdir("/opt/etc", 0755);  /* harmless if already exists */

    /* Wall-clock timestamp. The camera's CLOCK_REALTIME is kept in sync with
       the laptop via /x/time-sync.cgi (NTP UDP-123 is blocked outbound), so
       this is a real, dateable Unix time. The old CLOCK_MONOTONIC value was
       boot-relative and meaningless across reboots / vs. laptop time — it
       made monocal runs impossible to date. `timestamp_ms` is the
       authoritative TZ-independent value; `timestamp` mirrors
       cal_procedure.py's %Y%m%d_%H%M%S shape (UTC — the camera runs UTC,
       laptop writes local, so compare via timestamp_ms, not the string). */
    struct timespec rt;
    clock_gettime(CLOCK_REALTIME, &rt);
    long long ts_ms = (long long)rt.tv_sec * 1000 + rt.tv_nsec / 1000000;
    char ts_str[24] = "";
    struct tm tmv;
    if (gmtime_r(&rt.tv_sec, &tmv))
        strftime(ts_str, sizeof(ts_str), "%Y%m%d_%H%M%S", &tmv);

    char buf[640];
    int n = snprintf(buf, sizeof(buf),
        "{\n"
        "  \"tx_pixel\": [%d, %d],\n"
        "  \"frame_size\": [640, 360],\n"
        "  \"off_value\": %d,\n"
        "  \"on_value\": %d,\n"
        "  \"grid_delta\": %d,\n"
        "  \"peak_brightness\": %d,\n"
        "  \"timestamp_ms\": %lld,\n"
        "  \"timestamp\": \"%s\",\n"
        "  \"timestamp_tz\": \"UTC\",\n"
        "  \"source\": \"on_camera_pixel_cal\"\n"
        "}\n",
        tx_x, tx_y, off_val, on_val, grid_delta, peak_b, ts_ms, ts_str);
    int fd = open("/opt/etc/calibration.json", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        fprintf(stderr, "PIXEL-CAL: open calibration.json failed: %s\n", strerror(errno));
        return;
    }
    if (write(fd, buf, n) != n)
        fprintf(stderr, "PIXEL-CAL: write calibration.json short/failed\n");
    close(fd);
    fprintf(stderr, "PIXEL-CAL: wrote /opt/etc/calibration.json\n");
}

typedef struct {
    int pixel_x, pixel_y;
    uint8_t peak_b;
    int grid_delta;
    int block_idx;
    uint8_t zoom_4x4[16];   /* 4x4 grid-delta sub-area, 4-bit (0..15 per cell) */
} pixel_cal_result_t;

/* Two-phase calibration: grid coarse → pixel ROI sweep within peak block.
   Peer must keep IR LEDs ON for the full ~8s sequence (4s grid + 4s ROI). */
static int do_calibrate_pixel_to(pixel_cal_result_t *out)
{
    grid_cal_t cal;
    if (do_grid_calibration(&cal) < 0) {
        set_ae_freeze(0);
        return 1;
    }

    /* AE still frozen from do_grid_calibration. Peer should keep LED on. */
    fprintf(stderr, "PIXEL-CAL: keep peer LED ON, refining within block %d (~4s)...\n",
            cal.block_idx);

    int px, py;
    uint8_t pb;
    int rc = find_peak_pixel_in_block(cal.block_idx, &px, &py, &pb);
    set_ae_freeze(0);

    /* Critical: find_peak_pixel_in_block writes roi_config many times during
       its sweep. After scanning, roi_config is at the LAST scanned position,
       NOT at the base pixel set at startup. If we leave it there, subsequent
       protocol RX reads from the wrong pixel — e.g., phase-2 of bicall has
       cam2 (post-scan) listening at a stale ROI for cam1's CAL_REQ, decoding
       fails. Reset to the connect/listen pixel here so RX resumes correctly. */
    if (pixel_x >= 0 && pixel_y >= 0) {
        write_roi_config(pixel_x, pixel_y, pixel_roi_size);
        fprintf(stderr, "PIXEL-CAL: restored RX ROI to (%d, %d) size %d\n",
                pixel_x, pixel_y, pixel_roi_size);
    }

    if (rc < 0) {
        fprintf(stderr, "PIXEL-CAL: ROI sweep failed\n");
        return 1;
    }

    fprintf(stderr, "PIXEL-CAL: peak at (%d, %d), ROI brightness=%d\n", px, py, pb);
    write_calibration_json(px, py, cal.baseline_val, cal.active_val, cal.delta, pb);

    out->pixel_x = px;
    out->pixel_y = py;
    out->peak_b = pb;
    out->grid_delta = cal.delta;
    out->block_idx = cal.block_idx;
    memcpy(out->zoom_4x4, cal.zoom_4x4, 16);
    return 0;
}

static int do_calibrate_pixel(void)
{
    pixel_cal_result_t r;
    int rc = do_calibrate_pixel_to(&r);
    if (rc == 0) {
        /* Stable parseable line for orchestrators */
        printf("PIXEL: %d %d %d %d %d\n",
               r.pixel_x, r.pixel_y, (int)r.peak_b, r.grid_delta, r.block_idx);
    }
    return rc;
}

/* ================================================================
 *  Search — cold-start grid scan ("where is my peer?")
 *
 *  No prior pixel knowledge required. Scans the whole 20x12 grid for any
 *  block whose brightness has been elevated above its rolling baseline by
 *  >=SEARCH_DELTA for >=SEARCH_HOLD_MS continuously. Once acquired, refines
 *  to a precise pixel via the existing find_peak_pixel_in_block (peer must
 *  hold LEDs ON for the full search-hold + ROI-sweep window, ~10-15s).
 *
 *  This is the on-camera, light-coordinated equivalent of cal_procedure.py's
 *  laptop-driven frame-diff. Designed for the autonomous-cam1 deployment
 *  topology where the laptop only reaches one cam (or neither) over IP, so
 *  bootstrap calibration must travel by light alone. Counterpart "hold LEDs
 *  on" mode + bidirectional sequencing land in a follow-up patch — for now,
 *  this is the validating primitive.
 *
 *  Output on success: PIXEL line (same format as do_calibrate_pixel) +
 *  SEARCH line with timing diagnostics. Does NOT write calibration.json
 *  yet — caller / orchestrator decides whether to commit. (Will be flipped
 *  to write-on-success once we wire it into a daemon-listen fallback.)
 * ================================================================ */

/* Tuned against measured cam2-sees-cam1 delta at the current bench geometry:
 *   - AE active (during scan):   strongest block rises +13 when peer is on
 *   - AE frozen (during refine): strongest block rises only +9 (no ISP
 *     compression boosting contrast against dimmed non-LED blocks)
 * SEARCH_DELTA is sized to catch +13 with margin while staying above the
 * per-frame AE-compensation jitter of empty blocks (~±2). Refinement after
 * acquisition runs with AE frozen because find_peak_pixel_in_block needs
 * stable exposure to compare ROI candidates fairly. */
#define SEARCH_DELTA       7      /* min block-brightness rise vs baseline */
#define SEARCH_HOLD_MS     3000   /* must stay elevated this long to acquire */
#define SEARCH_BASELINE_MS 1500   /* baseline-capture window before scanning */
#define SEARCH_SCAN_HZ     20     /* grid-poll cadence during search */

static int do_search(int max_wait_ms,
                     int *out_block, int *out_px, int *out_py,
                     int *out_baseline, int *out_active, int *out_delta)
{
    uint8_t baseline[GRID_BLOCKS];
    int64_t bright_since[GRID_BLOCKS];
    int64_t baseline_taken_ms;
    int64_t start_ms = now_ms();

    /* AE stays ACTIVE during the scan — counter-intuitive but correct.
       When AE is active and the peer's LED comes on, the ISP compresses the
       global exposure: the LED block rises while non-LED blocks dim, giving
       us a CONTRAST delta we can detect (~+13). With AE frozen the global
       exposure is locked, so only the raw LED contribution shows up in the
       block (~+9) — too close to per-block noise. AE gets frozen later, just
       before pixel refinement, where stable exposure is non-negotiable. */

    /* Phase 1: capture baseline. Average a few frames to wash out per-frame
       noise. We assume peer is OFF during this window — caller's
       responsibility (cam2 explicitly delays its beacon by SEARCH_BASELINE_MS
       in the bidirectional flow). */
    fprintf(stderr, "SEARCH: capturing baseline (%dms)...\n",
            SEARCH_BASELINE_MS);
    status_set("search_baseline", "capturing grid baseline (peer LEDs OFF expected)");
    log_event("SEARCH", "baseline-start");

    {
        int sum[GRID_BLOCKS] = {0};
        int n_samples = 0;
        int64_t baseline_until = now_ms() + SEARCH_BASELINE_MS;
        while (now_ms() < baseline_until && running) {
            uint8_t blocks[GRID_BLOCKS];
            int64_t ts;
            if (read_grid(&ts, blocks, GRID_BLOCKS) > 0) {
                for (int i = 0; i < GRID_BLOCKS; i++) sum[i] += blocks[i];
                n_samples++;
            }
            usleep(60 * 1000);
        }
        if (n_samples == 0) {
            fprintf(stderr, "SEARCH: failed to read grid during baseline\n");
            return -1;
        }
        for (int i = 0; i < GRID_BLOCKS; i++)
            baseline[i] = (uint8_t)(sum[i] / n_samples);
        baseline_taken_ms = now_ms();
    }
    fprintf(stderr, "SEARCH: baseline ready, scanning for peer beacon...\n");
    status_set("search_scanning", "scanning grid for peer beacon");
    log_event("SEARCH", "baseline-done, scanning");
    for (int i = 0; i < GRID_BLOCKS; i++) bright_since[i] = 0;

    /* Phase 2: scan loop. Look for any block elevated by SEARCH_DELTA for
       SEARCH_HOLD_MS continuously. Track the strongest current candidate
       so a stronger block displaces a weaker one (handles ambient drift). */
    int useconds_per_scan = 1000000 / SEARCH_SCAN_HZ;
    int64_t deadline = start_ms + max_wait_ms;

    while (now_ms() < deadline && running) {
        uint8_t blocks[GRID_BLOCKS];
        int64_t ts;
        if (read_grid(&ts, blocks, GRID_BLOCKS) <= 0) {
            usleep(useconds_per_scan);
            continue;
        }

        int best_block = -1, best_delta = 0;
        for (int i = 0; i < GRID_BLOCKS; i++) {
            /* Skip OSD-tainted block rows (top + bottom). do_search has the
               same vulnerability as do_grid_calibration: the per-second OSD
               timestamp + name + uptime can sustain enough delta over
               SEARCH_HOLD_MS to win. Observed 2026-05-10: bootstrap
               do_search picked block 34 (row 1, x=448-479, y=30-59,
               adjacent to OSD strip) and wrote tx_pixel=(477, 47) into
               cam2's saved cal — clearly wrong vs the host RTSP cal that
               put cam1 at (294, 173). Row 10 added same day (watermark
               bleed observed via watchdog hit on block 200). Mirrors
               do_grid_calibration's mask + cal_procedure.py:206-209. */
            int by = i / 20;
            if (by == 0 || by == 10 || by == 11) {
                bright_since[i] = 0;
                continue;
            }
            int delta = (int)blocks[i] - (int)baseline[i];
            if (delta >= SEARCH_DELTA) {
                if (bright_since[i] == 0) bright_since[i] = ts;
                if (delta > best_delta) { best_delta = delta; best_block = i; }
            } else {
                bright_since[i] = 0;
            }
        }

        if (best_block >= 0 &&
            (ts - bright_since[best_block]) >= SEARCH_HOLD_MS) {
            int held_ms = (int)(ts - bright_since[best_block]);
            int active_val = blocks[best_block];
            fprintf(stderr,
                "SEARCH: ACQUIRED block %d (delta=%d, baseline=%d, "
                "active=%d, held=%dms)\n",
                best_block, best_delta, baseline[best_block],
                active_val, held_ms);
            log_event("SEARCH",
                "acquired block=%d delta=%d held=%dms",
                best_block, best_delta, held_ms);
            status_peak_block = best_block;
            status_peak_delta = best_delta;
            char ev[STATUS_EVENT_LEN];
            snprintf(ev, sizeof(ev),
                "block %d delta %d; refining to pixel",
                best_block, best_delta);
            status_set("search_refining", ev);

            /* Phase 3: refine to pixel. Freeze AE here — find_peak_pixel
               compares ROI brightness across rapid sweeps and any AE drift
               between samples invalidates the comparison. Peer must still be
               holding LEDs ON for the next ~5s of ROI sweep. */
            set_ae_freeze(1);
            usleep(500 * 1000);  /* AE settle after freeze */
            int px = -1, py = -1;
            uint8_t pb = 0;
            int rc = find_peak_pixel_in_block(best_block, &px, &py, &pb);
            set_ae_freeze(0);
            if (rc < 0) {
                fprintf(stderr, "SEARCH: pixel refinement failed\n");
                log_event("SEARCH", "refine-fail block=%d", best_block);
                status_set("search_failed", "pixel refinement failed");
                return 1;
            }
            *out_block = best_block;
            *out_px = px;
            *out_py = py;
            *out_baseline = baseline[best_block];
            *out_active = active_val;
            *out_delta = best_delta;
            status_peak_x = px;
            status_peak_y = py;
            status_peak_brightness = pb;
            log_event("SEARCH",
                "pixel=(%d,%d) peak_b=%d", px, py, pb);
            char ev2[STATUS_EVENT_LEN];
            snprintf(ev2, sizeof(ev2),
                "pixel (%d,%d) peak_b=%d", px, py, pb);
            status_set("search_done", ev2);
            (void)baseline_taken_ms;  /* reserved for future timing report */
            return 0;
        }

        usleep(useconds_per_scan);
    }

    fprintf(stderr, "SEARCH: timeout (%dms) — no peer beacon detected\n",
            max_wait_ms);
    log_event("SEARCH", "timeout=%dms", max_wait_ms);
    status_set("search_timeout", "no peer beacon detected");
    return 1;
}

/* ================================================================
 *  Light-triggered bootstrap helpers
 *
 *  hold_leds_on()  is reused by:
 *    - cam2's `irlink monocal` bootstrap fallback (long LED hold to wake
 *      cam1's grid-watchdog when the saved-pixel handshake fails)
 *    - cam1's grid-watchdog trigger handler (the responding hold so cam2
 *      can find cam1 in its do_search phase)
 *
 *  The wire-level signal is "LEDs solid for N seconds, no Manchester."
 *  Manchester-encoded frames always transition every symbol period
 *  (≤200ms at the slowest rate), so a multi-second sustained LED-on
 *  pulse is unambiguously NOT a normal protocol frame — cam1's grid-
 *  watchdog uses that to distinguish "peer wants discovery" from
 *  "peer is talking to me normally."  See the grid-watchdog section
 *  below for the detection logic, and do_monocal_request for the cam2
 *  side fallback flow.
 * ================================================================ */

/* Tuning constants for the cam2-side bootstrap fallback in monocal.
 * BOOTSTRAP_HOLD_MS must satisfy:
 *   trigger_window (~5s, watchdog dwell) + scan + refine (~10s) + slack
 * Plus enough margin that cam1 can finish refining BEFORE cam2 turns off
 * and switches to its own search. We pick 25s = generous. */
#define BOOTSTRAP_HOLD_MS         25000  /* cam2 holds LEDs solid this long */
#define BOOTSTRAP_BASELINE_GRACE_MS 2000 /* cam2 OFF before its own do_search */

static void hold_leds_on(int duration_ms)
{
    /* Direct GPIO toggle — no Manchester, no AE concern from RX side
       (we're TXing). 850nm and 940nm together for max emitted power.
       Stays in tx_mutex/tx_active so any rx_thread on this side correctly
       gates its self-suppression. */
    fprintf(stderr, "BOOTSTRAP: LEDs ON for %dms\n", duration_ms);
    log_event("BOOTSTRAP", "leds-on duration=%d", duration_ms);
    pthread_mutex_lock(&tx_mutex);
    tx_active = 1;
    gpio_set(1);
    /* Poll `running` so SIGTERM can interrupt a long hold. */
    int chunks = duration_ms / 100;
    for (int i = 0; i < chunks && running; i++) usleep(100000);
    int remainder = duration_ms - chunks * 100;
    if (remainder > 0 && running) usleep((useconds_t)remainder * 1000);
    gpio_set(0);
    tx_active = 0;
    pthread_mutex_unlock(&tx_mutex);
    fprintf(stderr, "BOOTSTRAP: LEDs OFF\n");
    log_event("BOOTSTRAP", "leds-off");
}

/* ================================================================
 *  Grid-watchdog (cam1-side bootstrap detector)
 *
 *  Runs alongside rx_thread. While daemon-listen is in a state where the
 *  saved pixel might be wrong (no valid frame ever decoded yet, OR no
 *  valid frame in IDLE_THRESHOLD_MS), the watchdog samples the brightness
 *  grid at 1 Hz and looks for any block that stays elevated above its
 *  rolling baseline by ≥WATCHDOG_DELTA for ≥WATCHDOG_TRIGGER_SAMPLES
 *  consecutive samples.
 *
 *  A multi-second sustained brightness rise is unambiguously NOT a
 *  Manchester frame (every symbol transitions ≤200ms); the only thing
 *  that produces it is a deliberate "hold LEDs solid" beacon — which is
 *  exactly what cam2 emits in its monocal bootstrap-fallback path.
 *
 *  When the watchdog fires, it runs handle_watchdog_trigger directly:
 *  refines to a pixel via find_peak_pixel_in_block, writes the new cal,
 *  waits for cam2's beacon to end (the falling edge), 2s OFF grace for
 *  cam2's own do_search baseline, then holds LEDs 15s so cam2's search
 *  finds cam1.
 *
 *  Concurrency:
 *    cal_flow_active=1 during the handler — gates the AE-drift defense
 *    and split-brain timer in rx_thread. tx_active=1 inside hold_leds_on
 *    suppresses self-RX during our own beacon. After the handler returns
 *    rx_thread naturally resumes at the new pixel (write_roi_config takes
 *    effect at the next BrightnessMonitor frame).
 * ================================================================ */

#define IDLE_THRESHOLD_MS         180000  /* 3 min silence → watchdog re-arms */
/* WATCHDOG_DELTA tuning: measured cam2-beacon delta at the bench is
 * +13..+21 on the brightest block. Ambient lighting features (wall
 * edges, lamps, partial sunlight) routinely sustain +7..+10 above the
 * EMA baseline without being a peer beacon. Setting threshold at 15
 * comfortably rejects ambient while still catching real beacons. */
#define WATCHDOG_DELTA              15
/* Sampling math: at 200ms cadence (5 Hz), Manchester at 160ms/sym lands
 * different phases on every sample. Over 5s = 25 consecutive samples,
 * Manchester traffic averages ~50% elevated; the chance of 25 in a row
 * elevated is ≈0.5^25 ≈ 0 — i.e. Manchester WILL produce OFF samples
 * within a 5s window. A solid LED hold (no transitions) keeps every
 * sample elevated, so 25-in-a-row is the correct discriminator between
 * "peer is talking to me normally" and "peer wants discovery". */
#define WATCHDOG_SAMPLE_MS          200   /* 5 Hz scan cadence */
#define WATCHDOG_TRIGGER_SAMPLES    25    /* 25 × 200ms = 5s sustained */
#define WATCHDOG_STARTUP_GRACE_MS 30000   /* don't fire in first 30s — let
                                             EMA settle and let cam2's
                                             short-handshake (~35s) play
                                             out before pivoting */
/* Max wait for cam2's LEDs to drop. Used to be 60s but the falling-edge
 * detection (delta < WATCHDOG_DELTA/2) frequently times out: handle_watchdog_trigger
 * froze AE *after* cam2's LEDs were already on, so AE locked at the compressed
 * exposure. After cam2 drops LEDs, blocks[block_idx] reads at the frozen
 * exposure but watchdog_baseline was captured pre-freeze at active AE — the
 * delta stays large positive and never falls below WATCHDOG_DELTA/2. With a
 * 60s ceiling, cam1's hold started at ~+77s after cam2's hold began, but cam2's
 * 25s search window had already closed at ~+51s — they never overlapped, cam2
 * gave up with "did not see cam1's responding hold". 22s ceiling guarantees
 * cam1's hold lands inside cam2's search window even on AE-baseline failure:
 *   cam1 hold (15s) at +39s..+54s vs cam2 search +26s..+51s → 12s overlap.
 * Proper fix is to re-baseline after AE freeze; this is the safety net. */
#define WATCHDOG_FALLING_EDGE_MS  22000
/* cam1's OFF window must FULLY cover cam2's BOOTSTRAP_BASELINE_GRACE_MS
 * (2s) + cam2's do_search baseline window (~1.5s) + falling-edge detect
 * latency on this side (~150ms) + safety margin. 4.5s gives clear room
 * for cam2's baseline to be captured entirely in cam1's OFF state. */
#define WATCHDOG_OFF_GRACE_MS      4500
#define WATCHDOG_HOLD_MS          15000   /* cam1's responding hold for cam2's search */
/* Guard tuning (replaces the old absolute-brightness "refuse weaker" gate).
 * Rationale: with a single ON snapshot the watchdog cannot tell a bright
 * static reflection from a real LED by brightness — the old `pb+30<saved_pb`
 * test (a) accepted a far reflection that happened to read ~saved_pb and
 * (b) trapped the cal, since the true TX reads dimmer than a reflection
 * when the peer is idle, so it rejected the *correct* pixel as "weaker".
 * Distance from a trusted saved cal is the only robust discriminator:
 *  - within WATCHDOG_NEAR_PX of saved → normal local refinement, always OK
 *    (this is what un-traps recovery: a correct-but-dimmer near pixel now
 *    passes instead of being rejected on brightness).
 *  - a far relocation is only legitimate when a deliberate solid-LED beacon
 *    is present (bumped-camera bootstrap: cam2 holds LEDs → strong sustained
 *    trigger delta). Ambient/reflection that merely cleared the +15 trigger
 *    floor must NOT move a trusted cal far. */
#define WATCHDOG_NEAR_PX            48    /* ≤ this from saved pixel = local refine */
#define WATCHDOG_RELOCATE_DELTA     25    /* far jump needs trigger delta ≥ this
                                             (well above the 15 trigger floor →
                                             a real beacon, not ambient) */

/* Set in daemon_mode (listener side) when watchdog is armed; used to gate
 * the startup grace period before the watchdog can actually fire. */
static int64_t watchdog_armed_at_ms = 0;

static volatile int watchdog_enabled = 0;       /* daemon_mode arms us */
static volatile int watchdog_active = 0;        /* handler currently running */

/* Block-level rolling baselines + consecutive-elevated counters, owned by
 * the watchdog thread. */
static uint8_t  watchdog_baseline[GRID_BLOCKS];
static int      watchdog_run_len[GRID_BLOCKS];
static int      watchdog_baseline_init = 0;

static int watchdog_should_run(void)
{
    if (!watchdog_enabled) return 0;
    if (cal_flow_active) return 0;
    if (watchdog_active) return 0;
    /* Suppress during our own TX. cam1's own LEDs cause self-reflection
     * in its grid; if not gated, a long classic SYN_ACK (20.8s at 200ms/sym)
     * generates enough sustained elevation for the watchdog to false-fire
     * before the peer ever transmits anything. Symptom: watchdog refines
     * onto a self-reflection block (typically the bottom edge of the frame
     * near where cam1's own LEDs sit) and either overwrites the cal or, with
     * the new guard, logs "refuse-overwrite" — either way it preempts the
     * imminent CAL_REQ handling and the protocol stalls. */
    if (tx_active) return 0;
    /* Startup grace: don't fire within the first WATCHDOG_STARTUP_GRACE_MS
       after arming. This gives the EMA baseline time to settle on real
       ambient brightness, AND gives cam2's short-handshake (~35s) room to
       complete on the protocol path before we'd pivot. Without this, an
       ambient bright spot present at daemon-listen launch can fire the
       watchdog before any peer transmission, breaking the happy path. */
    if (watchdog_armed_at_ms > 0
        && now_ms() - watchdog_armed_at_ms < WATCHDOG_STARTUP_GRACE_MS)
        return 0;
    /* While the rate lock is on (no successful cal yet), keep the watchdog
       hot: a half-completed handshake can update last_valid_frame_ms even
       though no cal happened, and the original 3-min idle gate would then
       block bootstrap recovery for the entire IDLE_THRESHOLD_MS window.
       Symptom: cam1 receives SYN + sends SYN_ACK, peer's ACK doesn't
       decode, peer pivots to bootstrap_hold (25s of solid LEDs), but
       cam1's watchdog stays gated because the SYN counts as "recent
       valid frame." Manchester transmission can't satisfy the watchdog's
       "25 consecutive samples ≥ WATCHDOG_DELTA" threshold (LED toggles
       at ~100ms intervals), so being eligible here is safe — only
       sustained LED-on (≥5s) trips it. */
    if (rate_locked_at_floor) return 1;
    /* Post-cal: only fire when no valid frame yet (startup) OR long-idle. */
    if (last_valid_frame_ms == 0) return 1;
    if (now_ms() - last_valid_frame_ms > IDLE_THRESHOLD_MS) return 1;
    return 0;
}

/* The handler. Runs entirely within the watchdog thread. Sets
 * cal_flow_active across its lifetime so other timers (split-brain,
 * AE-drift defense) treat the work as a legitimate cal flow. */
static void handle_watchdog_trigger(int block_idx, int trigger_delta)
{
    log_event("WATCHDOG", "fired block=%d (%d×%d=col,row) trig_delta=%d",
              block_idx, block_idx % 20, block_idx / 20, trigger_delta);
    fprintf(stderr, "WATCHDOG: triggered on block %d — refining pixel\n",
            block_idx);
    status_set("watchdog_refining",
               "bootstrap trigger: refining pixel from grid");
    cal_flow_active = 1;
    watchdog_active = 1;

    /* Phase 1: refine to pixel while peer's LEDs are still on. AE freeze
       is required for stable ROI sweep brightness comparisons. */
    set_ae_freeze(1);
    usleep(500 * 1000);
    int px = -1, py = -1;
    uint8_t pb = 0;
    int rc = find_peak_pixel_in_block(block_idx, &px, &py, &pb);
    if (rc < 0 || px < 0 || py < 0) {
        fprintf(stderr, "WATCHDOG: pixel refinement failed; aborting\n");
        log_event("WATCHDOG", "refine-fail block=%d", block_idx);
        status_set("watchdog_failed", "pixel refinement failed");
        set_ae_freeze(0);
        cal_flow_active = 0;
        watchdog_active = 0;
        return;
    }
    /* Persist new cal. We don't have an "off_value" measurement for the
       json (we only saw the ON state). Use the rolling baseline as the
       OFF estimate — close enough for diagnostic display. */
    int off_v = watchdog_baseline[block_idx];
    int on_v  = off_v + WATCHDOG_DELTA;  /* nominal — find_peak gives us pb */
    int delta_v = WATCHDOG_DELTA;

    /* Guard (replaces the old absolute-brightness "refuse weaker" test —
     * see WATCHDOG_NEAR_PX / WATCHDOG_RELOCATE_DELTA rationale above).
     * A single ON snapshot can't tell a bright static reflection from a
     * real LED, so we anchor on distance from the trusted saved cal:
     * local refinements always pass (this un-traps recovery — a
     * correct-but-dimmer near pixel is no longer rejected on brightness);
     * a FAR relocation is allowed only when a deliberate solid-LED beacon
     * is present (strong sustained trigger delta), i.e. the bumped-camera
     * bootstrap case. peak_b is logged for diagnostics only — no longer a
     * decision input. */
    int saved_pb = read_saved_peak_brightness();
    int sx = -1, sy = -1;
    if (read_saved_pixel(&sx, &sy) == 0) {
        long ddx = px - sx, ddy = py - sy;
        long dist2 = ddx * ddx + ddy * ddy;
        long near2 = (long)WATCHDOG_NEAR_PX * WATCHDOG_NEAR_PX;
        if (dist2 > near2 && trigger_delta < WATCHDOG_RELOCATE_DELTA) {
            fprintf(stderr,
                "WATCHDOG: refusing far relocation — new=(%d,%d) saved=(%d,%d) "
                "dist2=%ld (near2=%ld) trig_delta=%d < %d "
                "(reflection/ambient, not a beacon; pb=%d saved_pb=%d)\n",
                px, py, sx, sy, dist2, near2,
                trigger_delta, WATCHDOG_RELOCATE_DELTA, pb, saved_pb);
            log_event("WATCHDOG",
                "refuse-relocate new=(%d,%d) saved=(%d,%d) dist2=%ld "
                "trig_delta=%d block=%d pb=%d",
                px, py, sx, sy, dist2, trigger_delta, block_idx, pb);
            /* Keep the current ROI/cal — subsequent RX continues at the
             * saved-cal pixel. */
            status_set("watchdog_skipped",
                       "far jump w/o strong beacon; kept saved cal");
            set_ae_freeze(1);
            last_valid_frame_ms = now_ms();
            last_substantial_decode_fail_ms = 0;
            watchdog_baseline_init = 0;
            cal_flow_active = 0;
            watchdog_active = 0;
            return;
        }
    }

    write_calibration_json(px, py, off_v, on_v, delta_v, pb);
    pixel_x = px;
    pixel_y = py;
    /* Also mirror to the status fields the webui reads — `pixel` in
       /run/irlink-status.json reflects the live tracked pixel. Without
       this, the webui keeps showing the daemon's launch-time pixel
       even though rx_thread + roi_config are now at the new one. */
    status_pixel_x = px;
    status_pixel_y = py;
    write_roi_config(pixel_x, pixel_y, pixel_roi_size);
    status_peak_x = px;
    status_peak_y = py;
    status_peak_brightness = pb;
    status_peak_block = block_idx;
    fprintf(stderr,
        "WATCHDOG: cal saved — block=%d pixel=(%d,%d) peak_b=%d\n",
        block_idx, px, py, pb);
    log_event("WATCHDOG", "cal-saved pixel=(%d,%d) peak_b=%d", px, py, pb);

    /* Phase 2: wait for the peer's hold to end. We poll the brightness
       grid for our trigger block to drop; that's the implicit "your turn"
       signal.

       Subtle: the original implementation compared `blocks[block_idx]`
       against `watchdog_baseline[block_idx]`. That baseline was captured
       at ACTIVE-AE before cam2's LEDs ever turned on. Once we froze AE in
       Phase 1, the ISP locked the exposure at the value chosen for the
       LED-bright frame — which is much darker than the ambient exposure
       the baseline was sampled at. After cam2 drops LEDs, the locked-AE
       OFF reading often stays well above the active-AE baseline, so the
       `delta < WATCHDOG_DELTA/2` predicate never fires and the loop
       times out at WATCHDOG_FALLING_EDGE_MS. With a 60s ceiling that
       blew past cam2's ~50s search window; even at the new 22s ceiling
       it relies on dumb luck.

       Fix: capture a fresh `on_ref` NOW (AE frozen, cam2 still holding)
       and watch for that to drop by WATCHDOG_DELTA. Both samples are at
       the same locked exposure, so the comparison is well-defined. */
    int on_ref = -1;
    {
        uint8_t blocks0[GRID_BLOCKS];
        int64_t ts0;
        if (read_grid(&ts0, blocks0, GRID_BLOCKS) > 0)
            on_ref = (int)blocks0[block_idx];
    }
    status_set("watchdog_wait_falling",
               "waiting for peer LEDs to drop (their turn → ours)");
    log_event("WATCHDOG", "wait_falling on_ref=%d", on_ref);
    int64_t falling_deadline = now_ms() + WATCHDOG_FALLING_EDGE_MS;
    while (now_ms() < falling_deadline && running) {
        uint8_t blocks[GRID_BLOCKS];
        int64_t ts;
        if (read_grid(&ts, blocks, GRID_BLOCKS) > 0) {
            int cur = (int)blocks[block_idx];
            if (on_ref > 0) {
                /* Drop by ≥ WATCHDOG_DELTA below the locked-AE on-value
                   means cam2's LEDs are off. */
                if (on_ref - cur >= WATCHDOG_DELTA) break;
            } else {
                /* Fallback (on_ref capture failed): old pre-freeze
                   baseline check. Less reliable but better than nothing. */
                if (cur - (int)watchdog_baseline[block_idx]
                    < WATCHDOG_DELTA / 2) break;
            }
        }
        usleep(150 * 1000);
    }
    log_event("WATCHDOG", "falling-edge detected (or timed out)");

    /* Phase 3: OFF grace — cam2's do_search captures its baseline during
       this window, expecting our LEDs OFF. */
    set_ae_freeze(0);   /* unfreeze briefly so peer's AE matches ours? No
                           — we're just OFF and not RXing. Keep frozen
                           is also fine. Defensively unfreeze: don't
                           interfere with peer's grid view. */
    usleep(WATCHDOG_OFF_GRACE_MS * 1000);

    /* Phase 4: hold LEDs so peer's search can find us. */
    status_set("watchdog_hold",
               "holding LEDs so peer can find us");
    hold_leds_on(WATCHDOG_HOLD_MS);

    /* Done — restore daemon-listen state at the new pixel. The next
       protocol flow (e.g. cam2's normal monocal that follows the
       bootstrap) will succeed because pixel_x/pixel_y are now correct. */
    set_ae_freeze(1);
    last_valid_frame_ms = now_ms();          /* reset split-brain timers */
    last_substantial_decode_fail_ms = 0;
    /* Reset watchdog baselines so we don't re-fire on lingering elevation. */
    watchdog_baseline_init = 0;
    status_set("listening", "watchdog complete; resumed pixel-listen at new pixel");
    log_event("WATCHDOG", "complete pixel=(%d,%d)", px, py);
    unlock_rate_after_cal("handle_watchdog_trigger complete");
    cal_flow_active = 0;
    watchdog_active = 0;
}

static void *grid_watchdog_thread(void *unused)
{
    (void)unused;
    while (running) {
        usleep(WATCHDOG_SAMPLE_MS * 1000);
        if (!watchdog_should_run()) {
            watchdog_baseline_init = 0;
            continue;
        }

        uint8_t blocks[GRID_BLOCKS];
        int64_t ts;
        if (read_grid(&ts, blocks, GRID_BLOCKS) <= 0) continue;

        if (!watchdog_baseline_init) {
            /* Prefer to capture the baseline during a quiet grid window:
               cam2's monocal short-handshake (35s budget) overlaps cam1's
               30s startup grace — if grace expires mid-SYN-burst, baseline
               bakes in the volatile LED-modulating state and the subsequent
               bootstrap_hold's +20 elevation never crosses WATCHDOG_DELTA
               vs the already-elevated baseline. So defer when carrier was
               recent. BUT cap the deferral: if peer LEDs are stuck on
               from a prior run, carrier is *always* recent and we'd never
               capture, leaving the watchdog permanently dead. After
               WATCHDOG_BASELINE_MAX_DEFER_MS we accept whatever the grid
               currently is — the EMA on quiet blocks (loop below) will
               catch up to true ambient if peer LEDs eventually drop. */
            #define WATCHDOG_BASELINE_MAX_DEFER_MS 10000
            static int64_t watchdog_first_defer_ms = 0;
            if (last_carrier_ms > 0
                && now_ms() - last_carrier_ms < 1500
                && (watchdog_first_defer_ms == 0
                    || now_ms() - watchdog_first_defer_ms < WATCHDOG_BASELINE_MAX_DEFER_MS)) {
                if (watchdog_first_defer_ms == 0)
                    watchdog_first_defer_ms = now_ms();
                continue;
            }
            watchdog_first_defer_ms = 0;
            memcpy(watchdog_baseline, blocks, GRID_BLOCKS);
            for (int i = 0; i < GRID_BLOCKS; i++) watchdog_run_len[i] = 0;
            watchdog_baseline_init = 1;
            continue;
        }

        int triggered = -1;
        int best_delta = 0;
        for (int i = 0; i < GRID_BLOCKS; i++) {
            /* Skip OSD-tainted block rows (top + bottom). Channel 1's
               substream carries the per-second timestamp/name/uptime in
               by=0, the `thingino` watermark in by=11, and visible
               watermark-bleed into by=10. Sustained character-shape changes
               in those rows produce real per-block delta that easily clears
               WATCHDOG_DELTA over 5s, so the watchdog can fire on OSD
               jitter and write an OSD-pixel into the saved cal. Mirrors
               the same mask in do_grid_calibration and do_search.
               Surfaced 2026-05-10: first iteration (rows 0+11) saw the
               watchdog fire block=220 → wrote (2,347); second iteration
               (this rev, adds row 10) covers a follow-up where watchdog
               wrote (17,302) — block 200, row 10 — clobbering the cal. */
            int by = i / 20;
            if (by == 0 || by == 10 || by == 11) {
                watchdog_run_len[i] = 0;
                continue;
            }
            int delta = (int)blocks[i] - (int)watchdog_baseline[i];
            if (delta >= WATCHDOG_DELTA) {
                watchdog_run_len[i]++;
                if (watchdog_run_len[i] >= WATCHDOG_TRIGGER_SAMPLES
                    && delta > best_delta) {
                    best_delta = delta;
                    triggered = i;
                }
            } else {
                watchdog_run_len[i] = 0;
                /* Slow EMA on quiet blocks — tracks ambient drift. */
                watchdog_baseline[i] =
                    (uint8_t)((7 * watchdog_baseline[i] + blocks[i]) / 8);
            }
        }

        if (triggered >= 0) {
            handle_watchdog_trigger(triggered, best_delta);
        }
    }
    return NULL;
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

    /* Split-brain timer is armed only after the first valid frame is seen
       (last_valid_frame_ms stays 0 until rx_thread decodes something). This
       prevents the timer from firing during startup while we wait for the
       peer's first SYN/DATA, which can take longer than SPLIT_BRAIN_TIMEOUT_MS
       (a 16B DATA at 200ms/sym is ~70s on-air). */

    while (running) {
        /* Split-brain recovery: if we've seen a valid frame before and it's
           been too long since, AND we're not on the slowest rung → drop to
           slowest rate. Peer (if alive) will hit this too — both reconverge
           at the slowest rung. The last_valid_frame_ms>0 guard prevents
           firing during startup before any decode. Window scales with
           ack_timeout_ms so handshake flows don't false-trigger.
           Note: we do NOT require carrier-absence, because a peer stuck at
           a mismatched rate still produces carrier (we see TX activity) but
           can never produce a valid decode — that's exactly split-brain.
           HOWEVER, we DO require that recent "carrier" was substantial: a
           failed decode of a real-looking frame, not just 2-sample ambient
           drift. Otherwise, daemon-listen sitting idle for hours would trip
           split-brain on slow lighting drift and silently drop the rate. */
        int split_brain_window_ms = 2 * ack_timeout_ms;
        if (split_brain_window_ms < SPLIT_BRAIN_TIMEOUT_MS)
            split_brain_window_ms = SPLIT_BRAIN_TIMEOUT_MS;
        if (!tx_active && !cal_flow_active && last_valid_frame_ms > 0
            && current_rung != RATE_FLOOR_RUNG
            && last_substantial_decode_fail_ms > 0
            && now_ms() - last_substantial_decode_fail_ms < split_brain_window_ms
            && now_ms() - last_valid_frame_ms > split_brain_window_ms
            && now_ms() - last_split_brain_fire_ms > split_brain_window_ms) {
            /* Drop to FLOOR rate, not slowest. Floor is the configured
               reliable rate (160ms); slowest (200ms) is empirically less
               reliable for SYN/SYN_ACK round-trips. Going to floor keeps
               both cams convergent (peer hits this same recovery and lands
               at floor too) without sacrificing decode quality. Previous
               behavior (force slowest) caused split-brain divergence: cam1
               (recovered) at 200ms vs cam2 (fresh monocal) at 160ms could
               never decode each other. */
            apply_rate(RATE_LADDER_MS[RATE_FLOOR_RUNG],
                       RATE_FLOOR_RUNG, "split-brain-recovery");
            /* Debounce via separate timer — DO NOT touch last_valid_frame_ms
               (the grid-watchdog reads that to determine "long-idle" and
               we'd block its re-arm for another IDLE_THRESHOLD_MS). */
            last_split_brain_fire_ms = now_ms();
            success_at_rate = 0;
            fail_at_rate = 0;
            probe_in_progress = 0;
            pre_probe_rung = -1;
        }

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

        /* Mirror to globals so write_irlink_status can serve them via HTTP.
           These are diagnostic-only — no protocol behavior depends on them. */
        status_rx_brightness = brightness;
        status_rx_baseline   = baseline;
        status_rx_active     = (state == ACTIVE) ? 1 : 0;

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
                        [MSG_PING]="PING", [MSG_PONG]="PONG",
                        [MSG_RATE_CHANGE]="RATE_CHANGE"
                    };
                    const char *n = (msg.msg_type <= MSG_RATE_CHANGE) ? names[msg.msg_type] : "?";
                    fprintf(stderr, "RX: <<< %s seq=%d len=%d >>>\n",
                            n, msg.seq, msg.data_len);
                    log_event("RX", "%s seq=%d len=%d dpll=%d",
                              n, msg.seq, msg.data_len, dpll_ok);

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
                       CAL_REQ stays main-handled (needs LED-on hold).
                       RATE_CHANGE is also auto-handled: we ACK at the OLD rate,
                       then switch symbol_ms — keeping both sides in sync. */
                    if (msg.msg_type == MSG_DATA) {
                        send_message(MSG_ACK, msg.seq, NULL, 0);
                        last_valid_frame_ms = now_ms();
                        {
                            char ev[STATUS_EVENT_LEN];
                            snprintf(ev, sizeof(ev),
                                     "RX DATA seq=%d len=%d; ACKed",
                                     msg.seq, msg.data_len);
                            status_set("data_recv", ev);
                        }
                    } else if (msg.msg_type == MSG_PING) {
                        send_message(MSG_PONG, 0, NULL, 0);
                        last_valid_frame_ms = now_ms();
                    } else if (msg.msg_type == MSG_RATE_CHANGE && msg.data_len >= 2) {
                        int new_ms = ((int)msg.data[0] << 8) | msg.data[1];
                        send_message(MSG_ACK, msg.seq, NULL, 0);  /* ACK at OLD rate */
                        if (new_ms >= 40 && new_ms <= 2000) {
                            apply_rate(new_ms, rung_for_rate_ms(new_ms),
                                       "peer-initiated");
                        }
                        last_valid_frame_ms = now_ms();
                    } else {
                        /* All other valid frames still count toward split-brain recovery. */
                        last_valid_frame_ms = now_ms();
                    }
                } else {
                    crc_fail_count++;
                    fprintf(stderr, "RX: decode failed\n");
                    log_event("RX", "decode_fail samples=%d", n_samples);
                    /* Only "substantial" failures are evidence that a real
                       peer is transmitting at a rate we can't decode — the
                       split-brain signal. Brief 2-sample ambient drift blips
                       must not arm split-brain or daemon-listen will rate-
                       drop after hours of idle. Also skip during cal flows:
                       AE-toggling and ROI sweeps inside do_calibrate_pixel
                       cause large failed decodes that aren't rate-mismatch. */
                    if (n_samples >= SUBSTANTIAL_DECODE_SAMPLES
                        && !cal_flow_active)
                        last_substantial_decode_fail_ms = now_ms();
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
        prof_mark("do_connect: SYN TX start");
        send_message(MSG_SYN, 0, NULL, 0);
        prof_mark("do_connect: SYN TX done");

        fprintf(stderr, "PROTO: waiting for SYN_ACK...\n");
        rx_message_t reply;
        prof_mark("do_connect: SYN_ACK wait start");
        if (wait_for_msg(MSG_SYN_ACK, &reply, ack_timeout_ms) == 0) {
            prof_mark("do_connect: SYN_ACK received");
            fprintf(stderr, "PROTO: got SYN_ACK! Sending ACK...\n");
            send_message(MSG_ACK, 0, NULL, 0);
            prof_mark("do_connect: final ACK TX done");
            fprintf(stderr, "PROTO: connected!\n");
            status_set("connected", "got SYN_ACK; sent ACK");
            return 0;
        }
        prof_mark("do_connect: SYN_ACK timeout");
        fprintf(stderr, "PROTO: SYN_ACK timeout, retry %d/%d\n",
                attempt + 1, MAX_RETRIES);
        status_set("connecting", "SYN_ACK timeout, retrying");
    }

    fprintf(stderr, "PROTO: connection failed after %d attempts\n", MAX_RETRIES);
    status_set("error", "handshake failed (no SYN_ACK after retries)");
    return -1;
}

/* Bounded-retry connect with caller-controlled per-attempt timeout. Used by
   monocal's bootstrap-fallback path: send up to `tries` SYNs, returning -1
   fast (after tries * attempt_ms) so we can pivot to a light-only bootstrap
   instead of grinding through do_connect's MAX_RETRIES * ack_timeout_ms
   (~3.5min). One SYN→SYN_ACK round trip at 160ms/sym is ~35-37s, so a single
   SYN with a 35s budget (the old behavior) frequently expired even on a good
   decode and gave the first SYN ZERO retransmit — one transient miss on the
   peer (mid-watchdog / AE-settle / baseline) doomed the protocol path. A
   small bounded retry rescues a lone miss while still failing fast enough to
   keep the bootstrap pivot snappy when the peer's pixel is genuinely wrong.
   On the happy path (peer ACKs on the first try) behavior is unchanged. */
static int do_connect_short(int attempt_ms, int tries)
{
    for (int attempt = 0; attempt < tries && running; attempt++) {
        fprintf(stderr, "PROTO: short-handshake SYN (try %d/%d, %dms wait)...\n",
                attempt + 1, tries, attempt_ms);
        send_message(MSG_SYN, 0, NULL, 0);

        rx_message_t reply;
        if (wait_for_msg(MSG_SYN_ACK, &reply, attempt_ms) == 0) {
            fprintf(stderr, "PROTO: got SYN_ACK! Sending ACK...\n");
            send_message(MSG_ACK, 0, NULL, 0);
            fprintf(stderr, "PROTO: connected!\n");
            status_set("connected", "got SYN_ACK; sent ACK");
            return 0;
        }
        fprintf(stderr,
            "PROTO: short-handshake SYN_ACK timeout (try %d/%d, %dms)\n",
            attempt + 1, tries, attempt_ms);
    }
    return -1;
}

/* ---- Listen: wait for incoming connection ---- */

static int do_listen(void)
{
    fprintf(stderr, "PROTO: listening for SYN...\n");
    status_set("listening", "waiting for SYN");

    while (running) {
        rx_message_t msg;
        /* Wait indefinitely (60s chunks to check running flag) */
        if (wait_for_msg(MSG_SYN, &msg, 60000) == 0) {
            fprintf(stderr, "PROTO: got SYN! Sending SYN_ACK...\n");
            status_set("connecting", "got SYN; sent SYN_ACK");
            send_message(MSG_SYN_ACK, 0, NULL, 0);

            fprintf(stderr, "PROTO: waiting for ACK...\n");
            rx_message_t ack;
            if (wait_for_msg(MSG_ACK, &ack, ack_timeout_ms) == 0) {
                fprintf(stderr, "PROTO: connected!\n");
                status_set("connected", "got ACK");
                return 0;
            }
            fprintf(stderr, "PROTO: ACK timeout, back to listening\n");
            status_set("listening", "ACK timeout, retrying");
        }
    }
    return -1;
}

/* ---- Rate-change helper: send RATE_CHANGE, wait for ACK, switch on success ---- */

static int do_rate_change(int new_ms, const char *reason)
{
    uint8_t payload[2] = { (uint8_t)((new_ms >> 8) & 0xff), (uint8_t)(new_ms & 0xff) };
    /* Use seq=0 (out-of-band, same as SYN/PING). DATA has its own seq counter. */
    send_message(MSG_RATE_CHANGE, 0, payload, 2);

    rx_message_t reply;
    if (wait_for_msg(MSG_ACK, &reply, ack_timeout_ms) == 0) {
        apply_rate(new_ms, rung_for_rate_ms(new_ms), reason);
        return 0;
    }
    fprintf(stderr, "RATE: change to %dms failed (no ACK)\n", new_ms);
    return -1;
}

/* Maybe probe a faster rung before the next DATA send. */
static void maybe_probe_up(void)
{
    /* Locked at floor until first cal completes — prevents startup-window
       split-brain. See rate_locked_at_floor. */
    if (rate_locked_at_floor) return;
    if (probe_in_progress) return;
    if (current_rung <= 0) return;
    if (success_at_rate < PROBE_UP_AFTER) return;

    int faster_rung = current_rung - 1;
    int faster_ms = RATE_LADDER_MS[faster_rung];
    int64_t now = now_ms();
    if (now - failed_probe_cooldown_ms[faster_rung] < PROBE_COOLDOWN_MS) return;

    fprintf(stderr, "RATE: probing up %dms → %dms (success_at_rate=%d)\n",
            symbol_ms, faster_ms, success_at_rate);
    pre_probe_rung = current_rung;
    if (do_rate_change(faster_ms, "probe-up") == 0) {
        probe_in_progress = 1;
        success_at_rate = 0;
    } else {
        failed_probe_cooldown_ms[faster_rung] = now;
        success_at_rate = 0;  /* reset so we don't hammer */
    }
}

/* Maybe step to a slower rung after retransmit-exhausted sends. */
static void maybe_fallback_down(void)
{
    if (probe_in_progress && pre_probe_rung >= 0) {
        /* Failed right after a probe-up — roll back to the known-good rung immediately. */
        int prev_ms = RATE_LADDER_MS[pre_probe_rung];
        fprintf(stderr, "RATE: probe failed, rolling back %dms → %dms\n",
                symbol_ms, prev_ms);
        failed_probe_cooldown_ms[current_rung] = now_ms();
        do_rate_change(prev_ms, "probe-rollback");
        probe_in_progress = 0;
        pre_probe_rung = -1;
        fail_at_rate = 0;
        return;
    }

    if (fail_at_rate < FALLBACK_AFTER) return;
    if (current_rung >= RATE_LADDER_LEN - 1) return;

    int slower_ms = RATE_LADDER_MS[current_rung + 1];
    fprintf(stderr, "RATE: falling back %dms → %dms (fail_at_rate=%d)\n",
            symbol_ms, slower_ms, fail_at_rate);
    if (do_rate_change(slower_ms, "fallback") == 0) {
        fail_at_rate = 0;
    }
    /* On RATE_CHANGE timeout: split-brain timer in rx_thread will recover. */
}

/* ---- Send data with ACK ---- */

static int do_send_bytes(const uint8_t *data, int len)
{
    if (len > MAX_PAYLOAD - 2) {
        fprintf(stderr, "PROTO: message too long (%d, max %d)\n", len, MAX_PAYLOAD - 2);
        return -1;
    }

    static uint8_t seq = 1;

    /* Before sending, maybe probe a faster rate. */
    maybe_probe_up();

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        if (attempt > 0) retransmit_count++;
        {
            char ev[STATUS_EVENT_LEN];
            snprintf(ev, sizeof(ev),
                     "TX DATA seq=%d len=%d%s",
                     seq, len, attempt > 0 ? " (retry)" : "");
            status_set("sending_data", ev);
        }
        prof_mark("do_send_bytes: DATA TX start");
        send_message(MSG_DATA, seq, data, len);
        prof_mark("do_send_bytes: DATA TX done");

        fprintf(stderr, "PROTO: waiting for ACK seq=%d...\n", seq);
        {
            char ev[STATUS_EVENT_LEN];
            snprintf(ev, sizeof(ev), "waiting for ACK seq=%d", seq);
            status_set("data_ack_wait", ev);
        }
        rx_message_t reply;
        prof_mark("do_send_bytes: ACK wait start");
        if (wait_for_msg(MSG_ACK, &reply, ack_timeout_ms) == 0) {
            prof_mark("do_send_bytes: ACK received");
            if (reply.seq == seq) {
                fprintf(stderr, "PROTO: ACK received for seq=%d\n", seq);
                {
                    char ev[STATUS_EVENT_LEN];
                    snprintf(ev, sizeof(ev), "ACK received for seq=%d", seq);
                    status_set("connected", ev);
                }
                seq++;
                success_at_rate++;
                fail_at_rate = 0;
                /* Probe payload survived — commit to the faster rate. */
                probe_in_progress = 0;
                pre_probe_rung = -1;
                return 0;
            }
            fprintf(stderr, "PROTO: ACK seq mismatch (got %d, want %d)\n",
                    reply.seq, seq);
        }
        fprintf(stderr, "PROTO: ACK timeout, retry %d/%d\n",
                attempt + 1, MAX_RETRIES);
    }

    fprintf(stderr, "PROTO: send failed after %d retries\n", MAX_RETRIES);
    {
        char ev[STATUS_EVENT_LEN];
        snprintf(ev, sizeof(ev),
                 "no ACK for seq=%d after %d retries", seq, MAX_RETRIES);
        status_set("tx_failed", ev);
    }
    fail_at_rate++;
    success_at_rate = 0;
    maybe_fallback_down();
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

/* ---- Calibrate over the link ----
   `bidi` flag in CAL_REQ payload[0]: when 1, the responder swaps roles after
   the initial cycle finishes — it scans the original requestor in reverse,
   so both sides end up with fresh pixel coords from a single trigger. */

static int do_cal_request_internal(int bidi);  /* fwd decl for bidi swap */
static void handle_cal_request(int bidi);      /* fwd decl for bicall */

static int do_cal_request_internal(int bidi)
{
    fprintf(stderr, "PROTO: requesting pixel calibration (bidi=%d)...\n", bidi);
    cal_flow_active = 1;
    status_cal_phase = (bidi == 1) ? 1 : 2;
    status_cal_bidi = bidi;
    status_cal_retries = 0;
    status_set("cal_scanning", bidi ? "phase 1: sending CAL_REQ as scanner"
                                     : "phase 2: sending CAL_REQ as scanner");
    /* Reset to "now" so the split-brain timer doesn't pre-fire from stale
       quiet time leading into the cal flow. */
    last_valid_frame_ms = now_ms();

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        status_cal_retries = attempt;
        uint8_t req_payload[1] = { (uint8_t)bidi };
        send_message(MSG_CAL_REQ, 0, req_payload, 1);

        rx_message_t reply;
        if (wait_for_msg(MSG_CAL_ACK, &reply, ack_timeout_ms) == 0) {
            fprintf(stderr, "PROTO: peer acknowledged, starting pixel cal...\n");

            pixel_cal_result_t r;
            int rc = do_calibrate_pixel_to(&r);

            uint8_t payload[CAL_DONE_PAYLOAD_LEN];
            if (rc == 0) {
                pack_cal_done(r.pixel_x, r.pixel_y, payload);
                int rx_x, rx_y;
                unpack_cal_done(payload, &rx_x, &rx_y);
                fprintf(stderr,
                    "PROTO: pixel cal OK, sending CAL_DONE "
                    "(x=%d y=%d b=%d d=%d → quantized payload [%d, %d] → recovered (%d, %d))\n",
                    r.pixel_x, r.pixel_y, r.peak_b, r.grid_delta,
                    payload[0], payload[1], rx_x, rx_y);
                status_peak_x = r.pixel_x;
                status_peak_y = r.pixel_y;
                status_peak_brightness = r.peak_b;
                status_peak_delta = r.grid_delta;
                status_peak_block = r.block_idx;
                status_set("cal_sending_done", "scan complete; sending CAL_DONE");
                /* Stable parseable line for orchestrators (this side's scan
                   result, i.e. peer's TX position in OUR view). */
                printf("PIXEL: %d %d %d %d %d\n",
                       r.pixel_x, r.pixel_y, (int)r.peak_b, r.grid_delta, r.block_idx);
                fflush(stdout);
                send_message(MSG_CAL_DONE, 0, payload, CAL_DONE_PAYLOAD_LEN);

                /* CAL_VISUAL: 15-byte single DATA frame (1 type + 14 body),
                   fits the 16-byte single-frame ceiling. Body =
                   x(2)+y(2)+bright(1)+delta(1)+zoom_4x4(8). Only fire when
                   bidi==0 — sending during bidi==1 (bicall phase 1)
                   collides with the peer's immediate role-swap to scanner.
                   No fragmentation — single ACK round-trip, ~30 sec. */
                if (bidi == 0) {
                    uint8_t visual[15];
                    int delta_clamped = r.grid_delta > 255 ? 255
                                      : (r.grid_delta < 0 ? 0 : r.grid_delta);
                    visual[0] = APP_CAL_VISUAL;
                    visual[1] = (uint8_t)((r.pixel_x >> 8) & 0xFF);
                    visual[2] = (uint8_t)(r.pixel_x & 0xFF);
                    visual[3] = (uint8_t)((r.pixel_y >> 8) & 0xFF);
                    visual[4] = (uint8_t)(r.pixel_y & 0xFF);
                    visual[5] = r.peak_b;
                    visual[6] = (uint8_t)delta_clamped;
                    /* Pack 16 4-bit cells into 8 bytes, high nibble first. */
                    for (int i = 0; i < 8; i++) {
                        uint8_t hi = r.zoom_4x4[i * 2] & 0x0F;
                        uint8_t lo = r.zoom_4x4[i * 2 + 1] & 0x0F;
                        visual[7 + i] = (uint8_t)((hi << 4) | lo);
                    }
                    fprintf(stderr,
                            "PROTO: sending CAL_VISUAL (15B; 4x4 grid-delta "
                            "zoom; fire-and-forget)\n");
                    /* Fire-and-forget. Peer's rx_thread auto-ACKs DATA when
                       alive, but we don't wait — the cal coords already
                       landed via CAL_DONE, so the visual is informational
                       only. Going through do_send_bytes() would retry 3×
                       and wedge this daemon in cal_sending_done for ~230s
                       if the peer quits immediately after BICAL completion
                       (e.g. scripted `printf 'bicall\nquit'`), colliding
                       with any subsequent connection attempt from the
                       same peer. seq=1 mirrors what do_send_bytes would
                       have used on its first DATA — keeps peer logs sane
                       if the visual does land. */
                    send_message(MSG_DATA, 1, visual, 15);
                } else {
                    fprintf(stderr,
                            "PROTO: skipping CAL_VISUAL (bidi=1, phase 1 of "
                            "bicall — peer will swap roles immediately)\n");
                }
                cal_flow_active = 0;
                return 0;
            }

            /* On failure, send empty CAL_DONE so peer doesn't hang. */
            fprintf(stderr, "PROTO: pixel cal failed, sending empty CAL_DONE\n");
            send_message(MSG_CAL_DONE, 0, NULL, 0);
            cal_flow_active = 0;
            return -1;
        }
        fprintf(stderr, "PROTO: CAL_ACK timeout, retry %d/%d\n",
                attempt + 1, MAX_RETRIES);
    }
    cal_flow_active = 0;
    return -1;
}

static int do_cal_request(void) { return do_cal_request_internal(0); }

/* Bidirectional cal: scan peer, then swap roles and let peer scan us. After
   this returns, both /opt/etc/calibration.json files are fresh AND the
   orchestrator has seen both PIXEL: (our scan) and PEER-CAL: (peer's scan)
   lines on stdout. */
static int do_bicall(void)
{
    int rc = do_cal_request_internal(1);
    if (rc != 0) {
        fprintf(stderr, "BICAL: phase 1 (forward scan) failed\n");
        return -1;
    }

    /* AE was unfrozen at the end of do_calibrate_pixel_to. For phase 2 RX
       we need it frozen — without it, AE adapts during the inter-phase
       quiet window, then drops the contrast on incoming CAL_REQ symbols
       (we observed this as cam2 RX delta=14 instead of 200+, decode
       failures for the rest of phase 2). Keep AE frozen through the wait
       and through handle_cal_request's LED-on phase + CAL_DONE wait. */
    set_ae_freeze(1);
    fprintf(stderr, "BICAL: phase 1 done; waiting for peer's reverse CAL_REQ "
                    "(AE frozen for phase 2 RX)...\n");
    rx_message_t msg;
    int wait_ms = ack_timeout_ms * 2;
    if (wait_ms < 60000) wait_ms = 60000;
    if (wait_for_msg(MSG_CAL_REQ, &msg, wait_ms) != 0) {
        fprintf(stderr, "BICAL: timeout waiting for reverse CAL_REQ\n");
        set_ae_freeze(0);
        return -1;
    }
    fprintf(stderr, "BICAL: got reverse CAL_REQ — now LED-holder, peer is scanner\n");
    /* Peer's reverse CAL_REQ should have bidi=0 (no further swap). */
    int peer_bidi = (msg.data_len >= 1 && msg.data[0] == 1) ? 1 : 0;
    handle_cal_request(peer_bidi);
    set_ae_freeze(0);
    fprintf(stderr, "BICAL: bidirectional cal complete\n");
    return 0;
}

/* ---- Handle incoming calibration request ----
   Peer is the LED holder. Timing must align with peer's do_calibrate_pixel:
     0..1.5s   LED OFF — peer's AE freeze settle (0.5s) + baseline grid (1s)
     1.5..14s  LED ON steady — peer's grid active (4s) + ROI sweep (~7-9s
               with possible edge expansion)
     14s+      LED OFF — peer wraps up and sends CAL_DONE
   If `bidi` is set, after the responder phase + receive of peer's CAL_DONE,
   swap roles: WE become the requestor and scan the peer back. */

static void handle_cal_request(int bidi)
{
    fprintf(stderr, "PROTO: peer requested pixel calibration (bidi=%d)\n", bidi);
    cal_flow_active = 1;
    status_cal_phase = (bidi == 1) ? 1 : 2;
    status_cal_bidi = bidi;
    status_set("cal_holding", bidi ? "phase 1: holder, peer scanning us"
                                    : "phase 2: holder, peer scanning us");
    last_valid_frame_ms = now_ms();
    send_message(MSG_CAL_ACK, 0, NULL, 0);

    /* Phase 1: 1.5s LED OFF — peer's baseline phase needs a clean OFF window. */
    fprintf(stderr, "PROTO: holding LED OFF 1.5s for peer baseline...\n");
    usleep(1500000);

    /* Phase 2: 12s LED steady ON — peer's grid active + ROI sweep. */
    fprintf(stderr, "PROTO: holding LED ON steady for 12s for peer scan...\n");
    pthread_mutex_lock(&tx_mutex);
    tx_active = 1;
    gpio_set(1);
    for (int i = 0; i < 120 && running; i++)
        usleep(100000);  /* 120 * 100ms = 12s, polling running */
    gpio_set(0);
    tx_active = 0;
    pthread_mutex_unlock(&tx_mutex);

    /* Phase 3: wait for CAL_DONE with pixel coord payload. Wait window
       sized for: scanner takes ~12s for grid+ROI, +7s zoom (bidi==0 only),
       +CAL_DONE TX (~30s at 160ms). 90s gives slack for slow links. */
    status_set("cal_awaiting_done", "LED hold ended; waiting for peer CAL_DONE");
    rx_message_t msg;
    if (wait_for_msg(MSG_CAL_DONE, &msg, 90000) == 0) {
        if (msg.data_len >= CAL_DONE_PAYLOAD_LEN) {
            int x, y;
            unpack_cal_done(msg.data, &x, &y);
            fprintf(stderr,
                "PROTO: peer cal complete → received payload [%d, %d] → "
                "x=%d y=%d (4-px quantized)\n",
                msg.data[0], msg.data[1], x, y);
            status_peak_x = x;
            status_peak_y = y;
            /* peak_brightness / grid_delta are no longer carried in the
             * CAL_DONE payload (compressed 6→2 B). Leave the status fields
             * untouched so prior values remain visible. */
            char ev[STATUS_EVENT_LEN];
            snprintf(ev, sizeof(ev), "peer cal complete x=%d y=%d", x, y);
            status_set("cal_done_recv", ev);
            /* Stable parseable line for orchestrators (peer's view of US).
             * b/d remain in format for back-compat — emit -1 since we don't
             * have them. */
            printf("PEER-CAL: %d %d -1 -1\n", x, y);
            fflush(stdout);
            unlock_rate_after_cal("handle_cal_request success");
        } else {
            fprintf(stderr, "PROTO: peer cal returned empty CAL_DONE (failed)\n");
            status_set("cal_failed", "empty CAL_DONE from peer");
        }
    } else {
        fprintf(stderr, "PROTO: CAL_DONE timeout\n");
        status_set("cal_failed", "CAL_DONE timeout (90s)");
    }

    /* Phase 4 (bidi only): swap roles. Peer expects us to send CAL_REQ next.
       do_cal_request_internal manages cal_flow_active itself; clear here so
       there's no overlap if it sets+clears, and re-set is fine — both calls
       to set just write 1. */
    if (bidi) {
        fprintf(stderr, "PROTO: bidi swap — letting peer settle 2s, then scanning back\n");
        usleep(2000000);
        do_cal_request_internal(0);
    }
    cal_flow_active = 0;
}

/* ---- Monocal: cam2 holds LEDs, cam1 scans (peer_scans=1 path) ----
   Mirrors do_cal_request_internal but with REQUESTOR=holder, not scanner.
   CAL_REQ payload[0] bit 1 (peer_scans) tells the responder to invert its
   role — instead of holding LEDs while we scan, the responder scans while
   we hold LEDs. We then receive CAL_DONE with the responder's coords and
   write /run/monocal-status.json for the laptop webui to poll.

   Used by the `irlink monocal` subcommand on cam2; cam1 runs the
   responder side via handle_monocal_request() dispatched from
   daemon-listen / interactive_mode. */
static int do_monocal_request(void)
{
    fprintf(stderr, "PROTO: monocal — sending CAL_REQ with peer_scans=1 (we hold, peer scans)\n");
    cal_flow_active = 1;
    monocal_started_ms = now_ms();
    monocal_ok = -1;
    monocal_error[0] = '\0';
    monocal_have_peer_pixel = 0;
    monocal_ack_recv_ms = 0;
    monocal_hold_start_ms = 0;
    monocal_hold_end_ms = 0;
    monocal_ended_ms = 0;
    monocal_set_state("monocal_req", NULL);

    /* Reset split-brain timer (same hygiene as do_cal_request_internal). */
    last_valid_frame_ms = now_ms();

    int success = 0;
    for (int attempt = 0; attempt < MAX_RETRIES && !success; attempt++) {
        uint8_t req_payload[1] = { 0x02 };  /* bit 1 = peer_scans, bit 0 = bidi (clear) */
        send_message(MSG_CAL_REQ, 0, req_payload, 1);
        monocal_set_state("monocal_ack_wait", NULL);

        rx_message_t reply;
        if (wait_for_msg(MSG_CAL_ACK, &reply, ack_timeout_ms) != 0) {
            fprintf(stderr, "PROTO: monocal CAL_ACK timeout, retry %d/%d\n",
                    attempt + 1, MAX_RETRIES);
            continue;
        }
        monocal_ack_recv_ms = now_ms();
        fprintf(stderr, "PROTO: monocal CAL_ACK received; holding LEDs for peer scan\n");

        /* Phase 1: 1.5s LED OFF — peer's baseline window. */
        monocal_hold_start_ms = now_ms();
        monocal_set_state("monocal_holding_off", NULL);
        usleep(1500000);

        /* Phase 2: 12s LED steady ON — peer's grid + ROI sweep. */
        monocal_set_state("monocal_holding_on", NULL);
        fprintf(stderr, "PROTO: monocal — holding LED ON 12s for peer scan...\n");
        pthread_mutex_lock(&tx_mutex);
        tx_active = 1;
        gpio_set(1);
        for (int i = 0; i < 120 && running; i++)
            usleep(100000);  /* 120 * 100ms = 12s, polling running */
        gpio_set(0);
        tx_active = 0;
        pthread_mutex_unlock(&tx_mutex);
        monocal_hold_end_ms = now_ms();

        /* Phase 3: wait for CAL_DONE (peer scan ~12s + CAL_DONE TX ~30s @ 160ms). */
        monocal_set_state("monocal_awaiting_done", NULL);
        rx_message_t done_msg;
        if (wait_for_msg(MSG_CAL_DONE, &done_msg, 90000) != 0) {
            fprintf(stderr, "PROTO: monocal CAL_DONE timeout (90s)\n");
            monocal_ended_ms = now_ms();
            monocal_ok = 0;
            monocal_set_state("monocal_failed", "CAL_DONE timeout (90s)");
            cal_flow_active = 0;
            return -1;
        }

        if (done_msg.data_len >= CAL_DONE_PAYLOAD_LEN) {
            int x, y;
            unpack_cal_done(done_msg.data, &x, &y);
            fprintf(stderr,
                "PROTO: monocal complete → received payload [%d, %d] → "
                "peer x=%d y=%d (4-px quantized)\n",
                done_msg.data[0], done_msg.data[1], x, y);
            monocal_peer_x = x;
            monocal_peer_y = y;
            monocal_peer_b = 0;   /* not carried in 2-byte payload */
            monocal_peer_d = 0;
            monocal_have_peer_pixel = 1;
            /* Reuse status_peak_* so /x/cal-status.cgi reflects monocal too. */
            status_peak_x = x;
            status_peak_y = y;
            /* peak_brightness / grid_delta untouched — see RX-site-1 note. */
            /* Same parseable line as bicall's responder path uses. */
            printf("PEER-CAL: %d %d -1 -1\n", x, y);
            fflush(stdout);
            monocal_ended_ms = now_ms();
            monocal_ok = 1;
            monocal_set_state("monocal_done", NULL);
            success = 1;
        } else {
            fprintf(stderr, "PROTO: monocal — peer returned empty CAL_DONE (scan failed)\n");
            monocal_ended_ms = now_ms();
            monocal_ok = 0;
            monocal_set_state("monocal_failed", "peer scan failed (empty CAL_DONE)");
            cal_flow_active = 0;
            return -1;
        }
    }

    cal_flow_active = 0;
    if (!success) {
        monocal_ended_ms = now_ms();
        monocal_ok = 0;
        monocal_set_state("monocal_failed", "CAL_ACK timeout after retries");
        return -1;
    }
    return 0;
}

/* ---- Reverse monocal: WE scan, peer holds (Phase 2 of bidirectional) ----
   Mirror of do_monocal_request but with REQUESTOR=scanner, not holder.
   CAL_REQ flags=0 (regular cal mode) — the responder routes that to its
   existing handle_cal_request(0) which does 1.5s OFF + 12s ON LED hold,
   then waits for CAL_DONE. No responder-side change needed.

   Timing:
     T0      we send CAL_REQ
     ~T0+13s responder receives, sends CAL_ACK; CAL_ACK travels ~13s @ 160ms
     ~T0+26s we receive CAL_ACK — record cal_ack_recv_ms. Responder starts
             hold immediately: 1.5s OFF + 12s ON = 13.5s
     ~T0+26s..38s   we run do_calibrate_pixel_to() during peer's LED hold
                    window (1s baseline + 4s grid + ~6s ROI sweep = ~12s)
     ~T0+39.5s peer's hold ends
     SYNC_BARRIER: wait until cal_ack_recv_ms + 13500 (peer hold) + 1500
                   (drain) before TXing CAL_DONE. Same discipline as
                   handle_monocal_request, just applied requestor-side.
     CAL_DONE TX  ~30s — peer (responder) is waiting for it.

   On success: writes OUR /opt/etc/calibration.json (via do_calibrate_pixel_to),
   updates monocal-status with peer_pixel = OUR scan result (semantically:
   where the peer's LEDs landed in OUR view). Webui uses this to refresh
   cam2's bullseye after Phase 2 lands. */
static int do_monocal_request_reverse(void)
{
    fprintf(stderr, "PROTO: monocal reverse — sending CAL_REQ flags=0 (we scan, peer holds)\n");
    cal_flow_active = 1;
    monocal_started_ms = now_ms();
    monocal_ok = -1;
    monocal_error[0] = '\0';
    monocal_have_peer_pixel = 0;
    monocal_ack_recv_ms = 0;
    monocal_hold_start_ms = 0;
    monocal_hold_end_ms = 0;
    monocal_ended_ms = 0;
    monocal_set_state("monocal_rev_req", NULL);

    /* Reset split-brain timer (same hygiene as do_monocal_request). */
    last_valid_frame_ms = now_ms();

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        uint8_t req_payload[1] = { 0x00 };  /* flags=0: regular cal — peer holds, we scan */
        send_message(MSG_CAL_REQ, 0, req_payload, 1);
        monocal_set_state("monocal_rev_ack_wait", NULL);

        rx_message_t reply;
        if (wait_for_msg(MSG_CAL_ACK, &reply, ack_timeout_ms) != 0) {
            fprintf(stderr, "PROTO: monocal-rev CAL_ACK timeout, retry %d/%d\n",
                    attempt + 1, MAX_RETRIES);
            continue;
        }
        int64_t cal_ack_recv_at = now_ms();
        monocal_ack_recv_ms = cal_ack_recv_at;
        fprintf(stderr, "PROTO: monocal-rev CAL_ACK received; peer is holding LEDs, "
                        "starting our scan\n");

        /* Peer's hold sequence (handle_cal_request): 1.5s OFF + 12s ON, total
           13.5s starting essentially when peer sent CAL_ACK ≈ our receive
           time. do_calibrate_pixel_to internally handles AE freeze and writes
           /opt/etc/calibration.json on success. ~12s wall. */
        monocal_set_state("monocal_rev_scanning", "scanning peer's LED hold");
        pixel_cal_result_t r;
        int rc = do_calibrate_pixel_to(&r);

        /* SYNC_BARRIER: do not TX CAL_DONE until peer's LED hold has fully
           ended + drain margin. Mirrors handle_monocal_request's barrier on
           the responder side — applied here on the requestor (scanner) side
           because our scan can finish 1.5s before peer's hold does. Without
           this, CAL_DONE preamble starts while peer's tx_active=1 → peer
           misses our preamble → CAL_DONE timeout on peer side. */
        const int64_t PEER_HOLD_MS = 13500;
        const int64_t DRAIN_MARGIN_MS = 1500;
        int64_t wait_until = cal_ack_recv_at + PEER_HOLD_MS + DRAIN_MARGIN_MS;
        while (now_ms() < wait_until && running) {
            int64_t remaining = wait_until - now_ms();
            if (remaining > 200000) remaining = 200000;  /* defensive cap */
            usleep((useconds_t)(remaining * 1000));
        }

        if (rc == 0) {
            uint8_t payload[CAL_DONE_PAYLOAD_LEN];
            pack_cal_done(r.pixel_x, r.pixel_y, payload);
            int rx_x, rx_y;
            unpack_cal_done(payload, &rx_x, &rx_y);
            fprintf(stderr,
                "PROTO: monocal-rev scan OK, sending CAL_DONE "
                "(x=%d y=%d b=%d d=%d → quantized payload [%d, %d] → recovered (%d, %d))\n",
                r.pixel_x, r.pixel_y, r.peak_b, r.grid_delta,
                payload[0], payload[1], rx_x, rx_y);
            /* Update status_peak_* so /x/cal-status.cgi reflects OUR scan. */
            status_peak_x = r.pixel_x;
            status_peak_y = r.pixel_y;
            status_peak_brightness = r.peak_b;
            status_peak_delta = r.grid_delta;
            status_peak_block = r.block_idx;
            /* monocal-status.json: peer_pixel holds OUR scan result (the
               pixel where the peer's LEDs landed in our view — symmetric
               with regular monocal where peer_pixel = cam1's view of cam2). */
            monocal_peer_x = r.pixel_x;
            monocal_peer_y = r.pixel_y;
            monocal_peer_b = r.peak_b;
            monocal_peer_d = r.grid_delta;
            monocal_have_peer_pixel = 1;
            /* Parseable line for orchestrators (this side's scan = peer's
               TX position in OUR view). Same format as do_cal_request_internal. */
            printf("PIXEL: %d %d %d %d %d\n",
                   r.pixel_x, r.pixel_y, (int)r.peak_b, r.grid_delta, r.block_idx);
            fflush(stdout);
            send_message(MSG_CAL_DONE, 0, payload, CAL_DONE_PAYLOAD_LEN);
            monocal_ended_ms = now_ms();
            monocal_ok = 1;
            monocal_set_state("monocal_rev_done", NULL);
            unlock_rate_after_cal("do_monocal_request_reverse success");
            cal_flow_active = 0;
            return 0;
        }

        /* Scan failed — send empty CAL_DONE so peer's wait_for_msg drops out
           rather than timing out at 90s. */
        fprintf(stderr, "PROTO: monocal-rev scan failed, sending empty CAL_DONE\n");
        send_message(MSG_CAL_DONE, 0, NULL, 0);
        monocal_ended_ms = now_ms();
        monocal_ok = 0;
        monocal_set_state("monocal_rev_failed", "pixel scan failed");
        cal_flow_active = 0;
        return -1;
    }

    monocal_ended_ms = now_ms();
    monocal_ok = 0;
    monocal_set_state("monocal_rev_failed", "CAL_ACK timeout after retries");
    cal_flow_active = 0;
    return -1;
}

/* ---- Handle incoming monocal request (peer_scans=1) ----
   Inverse of handle_cal_request: we are the SCANNER, peer is the HOLDER.
   Timing alignment with peer's hold (1.5s OFF + 12s ON, total 13.5s starting
   ~50ms after our CAL_ACK TX completes):

     0..~12s   we run do_calibrate_pixel_to() — its internal AE freeze +
               1s baseline + 4s grid scan + ~6s ROI sweep happens during
               peer's LED hold window
     ~12..15s  SYNC_BARRIER — wait until cal_ack_sent_at + PEER_HOLD_MS
               + DRAIN_MARGIN_MS so our CAL_DONE preamble doesn't collide
               with peer's tx_active=1 LED-on window. This is the explicit
               fix for the bicall phase-2 "scanner finishes before holder
               drops LEDs" pitfall.
     15s+      send CAL_DONE with 6-byte coord payload */
static void handle_monocal_request(void)
{
    fprintf(stderr, "PROTO: peer requested monocal (we scan, peer holds)\n");
    cal_flow_active = 1;
    status_cal_phase = 0;  /* not phase 1 or 2 of bicall */
    status_cal_bidi = 0;
    status_set("monocal_resp_ack", "monocal — sending CAL_ACK as scanner");
    last_valid_frame_ms = now_ms();

    /* Exposure rebalance (scanner-side, pre-CAL_ACK). cam1's daemon-listen
       holds AE frozen continuously at idle (daemon_mode re-asserts
       set_ae_freeze(1) after every handled message), so without this the
       scanner would capture its baseline at whatever exposure the ISP locked
       at BOOT under boot-time ambient — do_grid_calibration's
       `set_ae_freeze(1); usleep(500000)` is a no-op in this path because AE
       is already frozen. If ambient shifted since boot the locked exposure is
       mismatched (too dark → weak peer-LED Δ; too bright → clipped).

       The holder (peer) has sent CAL_REQ and is now blocked waiting for our
       CAL_ACK — it will NOT drive LEDs until it receives it, so right here is
       the one window where the scene is provably peer-LEDs-OFF long enough to
       let AE re-adapt. Unfreeze, let AE free-run to current ambient, refreeze
       at the balanced level, THEN send CAL_ACK.

       SYNC_BARRIER stays correct without re-derivation: cal_ack_sent_at is
       captured AFTER this block and the holder's 13.5s hold clock starts only
       when it receives CAL_ACK — both anchors shift forward by exactly this
       settle delay, so their relationship is unchanged. The added delay is
       far inside the holder's CAL_ACK-wait ack_timeout_ms (~77s @ 160ms/sym),
       so no CAL_REQ retransmit risk. cal_flow_active is already 1, so the
       AE-drift defense (which only re-asserts when !cal_flow_active) won't
       fight our unfreeze. */
    {
        const int64_t AE_REBALANCE_SETTLE_MS = 4000;
        fprintf(stderr,
            "PROTO: monocal — rebalancing exposure (AE free-run %lldms, "
            "peer LEDs OFF) before CAL_ACK\n",
            (long long)AE_REBALANCE_SETTLE_MS);
        status_set("monocal_resp_aebal",
                   "rebalancing exposure before scan");
        set_ae_freeze(0);
        int64_t reb_until = now_ms() + AE_REBALANCE_SETTLE_MS;
        while (now_ms() < reb_until && running) {
            int64_t remaining = reb_until - now_ms();
            if (remaining > 200000) remaining = 200000;  /* defensive cap */
            usleep((useconds_t)(remaining * 1000));
        }
        set_ae_freeze(1);
        fprintf(stderr,
            "PROTO: monocal — exposure rebalanced, AE refrozen\n");
    }

    send_message(MSG_CAL_ACK, 0, NULL, 0);
    int64_t cal_ack_sent_at = now_ms();

    /* Run the pixel scan during peer's LED hold window. do_calibrate_pixel_to
       handles its own AE freeze and writes /opt/etc/calibration.json on
       success. Takes ~12s. */
    status_set("monocal_resp_scan", "scanning peer's LED hold");
    pixel_cal_result_t r;
    int rc = do_calibrate_pixel_to(&r);

    /* SYNC_BARRIER: do not TX CAL_DONE until peer's LED hold has fully ended
       plus a 1.5s drain margin. peer's hold = 1.5s OFF + 12s ON = 13500ms,
       starting ~50ms after our CAL_ACK TX completes. We wait until
       cal_ack_sent_at + 15000ms regardless of when our scan finished. */
    const int64_t PEER_HOLD_MS = 13500;
    const int64_t DRAIN_MARGIN_MS = 1500;
    int64_t wait_until = cal_ack_sent_at + PEER_HOLD_MS + DRAIN_MARGIN_MS;
    while (now_ms() < wait_until && running) {
        int64_t remaining = wait_until - now_ms();
        if (remaining > 200000) remaining = 200000;  /* defensive cap */
        usleep((useconds_t)(remaining * 1000));
    }

    if (rc == 0) {
        uint8_t payload[CAL_DONE_PAYLOAD_LEN];
        pack_cal_done(r.pixel_x, r.pixel_y, payload);
        int rx_x, rx_y;
        unpack_cal_done(payload, &rx_x, &rx_y);
        fprintf(stderr,
            "PROTO: monocal scan OK, sending CAL_DONE "
            "(x=%d y=%d b=%d d=%d → quantized payload [%d, %d] → recovered (%d, %d))\n",
            r.pixel_x, r.pixel_y, r.peak_b, r.grid_delta,
            payload[0], payload[1], rx_x, rx_y);
        status_peak_x = r.pixel_x;
        status_peak_y = r.pixel_y;
        status_peak_brightness = r.peak_b;
        status_peak_delta = r.grid_delta;
        status_peak_block = r.block_idx;
        status_set("monocal_resp_done", "scan complete; sending CAL_DONE");
        printf("PIXEL: %d %d %d %d %d\n",
               r.pixel_x, r.pixel_y, (int)r.peak_b, r.grid_delta, r.block_idx);
        fflush(stdout);
        send_message(MSG_CAL_DONE, 0, payload, CAL_DONE_PAYLOAD_LEN);
        unlock_rate_after_cal("handle_monocal_request success");
    } else {
        fprintf(stderr, "PROTO: monocal scan failed, sending empty CAL_DONE\n");
        status_set("monocal_resp_failed", "pixel scan failed");
        send_message(MSG_CAL_DONE, 0, NULL, 0);
    }

    /* do_calibrate_pixel_to() always unfreezes AE on its way out, but the
       enclosing daemon-listen needs AE frozen for reliable RX of subsequent
       SYNs / DATA / CAL_REQs. handle_cal_request restores it (line ~2187);
       this path was missing the same restore — symptom: daemon left running
       with AE active, peer's next SYN goes undecoded. Mirror that here. */
    set_ae_freeze(1);
    /* Reset split-brain timers so the post-monocal idle window starts
       fresh — without this, last_valid_frame_ms is stale (last decoded
       CAL_REQ might be 30+s old) and split-brain can fire 2*ack_timeout
       later even though the link is healthy. */
    last_valid_frame_ms = now_ms();
    last_substantial_decode_fail_ms = 0;
    status_set("listening", "monocal complete; ready for next request");
    cal_flow_active = 0;
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
    fprintf(stderr, "  cal                — request peer calibration (one-way)\n");
    fprintf(stderr, "  bicall             — bidirectional cal (we scan, then peer scans us)\n");
    fprintf(stderr, "  rate <ms>          — switch symbol rate (sync'd via RATE_CHANGE)\n");
    fprintf(stderr, "  stats              — print link counters + current rate\n");
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
        } else if (strncmp(line, "rate ", 5) == 0) {
            int new_ms = atoi(line + 5);
            if (new_ms < 40 || new_ms > 2000) {
                fprintf(stderr, "rate: ms must be 40-2000\n");
            } else {
                do_rate_change(new_ms, "manual");
            }
        } else if (strcmp(line, "stats") == 0) {
            printf("STATS: tx=%d rx=%d crc=%d rtx=%d dll=%d rate=%d rung=%d\n",
                   tx_count, rx_count, crc_fail_count,
                   retransmit_count, dpll_loss_count,
                   symbol_ms, current_rung);
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
        } else if (strcmp(line, "bicall") == 0) {
            do_bicall();
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
                /* Extract flags before releasing the mutex — rx_thread might
                   overwrite rx_msg.data once we let go. payload[0] flags:
                   bit 0 = bidi, bit 1 = peer_scans (mutually exclusive). */
                int flags = (rx_msg.data_len >= 1) ? rx_msg.data[0] : 0;
                int bidi       = !!(flags & 0x01);
                int peer_scans = !!(flags & 0x02);
                rx_msg.valid = 0;
                pthread_mutex_unlock(&rx_mutex);
                if (peer_scans)
                    handle_monocal_request();
                else
                    handle_cal_request(bidi);
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
    /* Rate floor + lock are applied from main() before we get here, common
       to all communication modes. */
    pthread_t rx_tid;
    if (pthread_create(&rx_tid, NULL, rx_thread, NULL) != 0) {
        perror("pthread_create");
        return 1;
    }

    /* Grid-watchdog DISABLED 2026-05-15 — never armed, never spawned.
       It auto-overwrote /opt/etc/calibration.json from a single LED-ON
       brightness argmax with NO peer ground truth. No brightness / delta /
       distance heuristic can separate a real beacon from a bright ambient
       reflection: field event logs showed it saving garbage pixels with
       trigger deltas of 38 / 31 / 60 — STRONGER than a genuine cam2 beacon
       (only +13..+21) — while no peer was transmitting, random-walking
       cam1's saved pixel ~200px across the frame every ~10 min and leaving
       cam1 booting blind on every reboot. Its only legitimate role was a
       light-only bootstrap LED-hold for a wrong-pixel cam1, but it was
       ITSELF the cause of the wrong pixel, and the protocol/monocal
       handshake — reliable after the 2×45s short-handshake fix — is the
       correct, peer-confirmed way to refresh cal. So cal is now persisted
       ONLY by a confirmed peer monocal/CAL_DONE. watchdog_enabled stays 0,
       so watchdog_should_run() always returns 0 (defense in depth if the
       thread is ever spawned elsewhere). The watchdog code is kept compiled
       and referenced below so re-enabling is a one-block revert. */
    (void)grid_watchdog_thread;   /* intentionally never run; see above */
    (void)watchdog_armed_at_ms;   /* unused while disabled */
    if (is_listener)
        fprintf(stderr,
            "WATCHDOG: disabled — cal is peer-confirmed (monocal) only\n");

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
            case MSG_CAL_REQ: {
                /* payload[0] flags: bit 0 = bidi, bit 1 = peer_scans. */
                int flags = (msg.data_len >= 1) ? msg.data[0] : 0;
                int bidi       = !!(flags & 0x01);
                int peer_scans = !!(flags & 0x02);
                if (peer_scans)
                    handle_monocal_request();
                else
                    handle_cal_request(bidi);
                break;
            }
            case MSG_SYN:
                send_message(MSG_SYN_ACK, 0, NULL, 0);
                break;
            default:
                break;
            }

            /* Belt-and-suspenders: any handler that completes should leave
               us back in a known good steady state — AE frozen, status
               "listening". Idempotent on the happy path (no-op cost), and
               catches the class of bug where a future handler forgets to
               restore one of these (the missing AE refreeze in the original
               handle_monocal_request was exactly this). */
            set_ae_freeze(1);
            if (strcmp(status_state, "listening") != 0)
                status_set("listening", "handler returned; ready");
        }

        /* Auto-revert `data_recv` back to `listening` after a short
           visibility window. rx_thread sets `data_recv` asynchronously
           when it auto-ACKs a peer DATA frame — main thread never sees
           those, so without this revert the webui would show
           "got data from peer" sticky until the next non-DATA event
           (could be hours). 1500ms is long enough for the 2s webui
           poll to catch it at least once. */
        if (strcmp(status_state, "data_recv") == 0
            && status_event_ms > 0
            && now_ms() - status_event_ms > 1500) {
            status_set("listening", "ready for next peer");
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

    status_started_ms = now_ms();
    if (argc >= 2) {
        strncpy(status_mode, argv[1], sizeof(status_mode) - 1);
        status_mode[sizeof(status_mode) - 1] = '\0';
    }
    status_set("starting", "irlink launched");
    atexit(status_on_exit);

    /* Heartbeat: 1 Hz status re-write so ts_ms ticks even in steady states.
       Detached — terminates implicitly when running=0 + process exit. */
    {
        pthread_t hb_tid;
        if (pthread_create(&hb_tid, NULL, heartbeat_thread, NULL) == 0)
            pthread_detach(hb_tid);
    }

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <command> [options]\n", argv[0]);
        fprintf(stderr, "Commands:\n");
        fprintf(stderr, "  listen [--block N] [--speed MS]     — wait for connection\n");
        fprintf(stderr, "  connect [--block N] [--speed MS]    — initiate connection\n");
        fprintf(stderr, "  daemon-listen [--block N] [--speed MS] — listen + auto-respond\n");
        fprintf(stderr, "  daemon-connect [--block N] [--speed MS] — connect + auto-respond\n");
        fprintf(stderr, "  monocal --pixel X,Y [--speed MS] — peer scans, we hold LEDs (writes /run/monocal-status.json)\n");
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
            status_pixel_x = pixel_x;
            status_pixel_y = pixel_y;
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
            current_rung = rung_for_rate_ms(symbol_ms);
            fprintf(stderr, "irlink: symbol speed %d ms (rung %d)\n", symbol_ms, current_rung);
        } else if (strcmp(argv[i], "--reverse") == 0) {
            /* Reverse monocal: WE scan, peer holds. Only valid with the
               `monocal` subcommand; ignored by everything else. */
            monocal_reverse = 1;
            fprintf(stderr, "irlink: monocal reverse — we scan, peer holds\n");
        } else if (strcmp(argv[i], "--both") == 0) {
            /* Fused bidirectional monocal: Phase 1 (default) then Phase 2
               (reverse), single handshake. Only valid with `monocal`. */
            monocal_both = 1;
            fprintf(stderr, "irlink: monocal both — fused Phase 1 + Phase 2 in one session\n");
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

    if (strcmp(cmd, "calibrate-pixel") == 0) {
        return do_calibrate_pixel();
    }

    if (strcmp(cmd, "search") == 0) {
        int wait_ms = 60000;
        for (int i = 2; i < argc; i++) {
            if (strcmp(argv[i], "--wait") == 0 && i + 1 < argc)
                wait_ms = atoi(argv[i + 1]);
        }
        int block, px, py, off_v, on_v, delta_v;
        int rc = do_search(wait_ms, &block, &px, &py, &off_v, &on_v, &delta_v);
        if (rc == 0) {
            /* Stable parseable line. Mirrors do_calibrate_pixel's PIXEL line
               format so existing orchestrators can consume either source.  */
            printf("PIXEL: %d %d %d %d %d\n", px, py, on_v, delta_v, block);
            printf("SEARCH: block=%d pixel=%d,%d baseline=%d active=%d delta=%d\n",
                   block, px, py, off_v, on_v, delta_v);
            fflush(stdout);
            return 0;
        }
        return 1;
    }

    /* All communication modes: freeze AE first, unfreeze on exit */

    /* Force AE to adapt to current ambient before freezing. Without this,
       restarting daemon-listen while a prior run left AE frozen at a stale
       exposure (e.g. peer LEDs were on when AE last settled) keeps the cam
       saturated at the wrong brightness — subsequent peer SYNs at lower
       brightness modulation never cross MIN_BRIGHTNESS_DELTA, every
       handshake fails into bootstrap_fallback. The unfreeze→3s→freeze
       handshake forces AE to converge on whatever the current scene
       actually is, then locks. */
    #define AE_PRE_FREEZE_SETTLE_MS  3000
    #define AE_FREEZE_APPLY_MS        500

    if (strcmp(cmd, "listen") == 0) {
        lock_rate_at_floor("listen");
        set_ae_freeze(0);
        usleep(AE_PRE_FREEZE_SETTLE_MS * 1000);
        set_ae_freeze(1);
        usleep(AE_FREEZE_APPLY_MS * 1000);
        int ret = interactive_mode(1);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "connect") == 0) {
        lock_rate_at_floor("connect");
        set_ae_freeze(0);
        usleep(AE_PRE_FREEZE_SETTLE_MS * 1000);
        set_ae_freeze(1);
        usleep(AE_FREEZE_APPLY_MS * 1000);
        int ret = interactive_mode(0);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "daemon-listen") == 0) {
        lock_rate_at_floor("daemon-listen");
        set_ae_freeze(0);
        usleep(AE_PRE_FREEZE_SETTLE_MS * 1000);
        set_ae_freeze(1);
        usleep(AE_FREEZE_APPLY_MS * 1000);
        int ret = daemon_mode(1);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "daemon-connect") == 0) {
        lock_rate_at_floor("daemon-connect");
        set_ae_freeze(0);
        usleep(AE_PRE_FREEZE_SETTLE_MS * 1000);
        set_ae_freeze(1);
        usleep(AE_FREEZE_APPLY_MS * 1000);
        int ret = daemon_mode(0);
        set_ae_freeze(0);
        return ret;
    }

    if (strcmp(cmd, "monocal") == 0) {
        /* Two-path flow:
           1) Try a short-handshake SYN (one attempt, ~35s budget).
              If it succeeds, the saved cam1 pixel is approximately correct
              → run the existing protocol monocal (do_monocal_request) to
              refresh both cals via SYN/CAL_REQ/hold/CAL_DONE.
           2) If the short handshake times out, cam1's saved pixel can't
              decode our SYN. Fall back to a LIGHT-ONLY bootstrap:
                a) Hold LEDs solid 25s — cam1's grid-watchdog detects this
                   non-Manchester signal and self-engages refine + hold-back.
                b) 2s OFF grace (cam2's own do_search baseline window).
                c) do_search to find cam1's responding hold; write our cal.
              Spawned by /x/monocal-trigger.cgi on cam2. */
        if (pixel_x < 0 || pixel_y < 0) {
            fprintf(stderr, "irlink monocal: --pixel X,Y is required\n");
            return 1;
        }

        monocal_started_ms = now_ms();
        monocal_set_state("starting", NULL);

        lock_rate_at_floor("monocal");
        /* Defensive: ensure our LEDs are OFF before AE settles. If a prior
           monocal run left them stuck ON (observed after `do_search` timeout
           paths), AE would settle to a saturated exposure and the rest of
           the flow runs blind. gpio_set drives both 850 + 940 OFF directly. */
        gpio_set(0);
        /* Same AE-settle handshake as listen/connect/daemon-listen — without
           it, monocal's rx_thread runs with whatever AE state was inherited
           from the caller (typically 0 since /x/monocal-trigger.cgi spawns
           cam2 fresh). With AE active, the ISP compensates for cam1's IR
           pulses and the brightness deltas of incoming SYN_ACK / CAL_ACK /
           CAL_DONE never cross MIN_BRIGHTNESS_DELTA → every decode fails.
           Symptom (observed 2026-05-10): cam1 progresses through SYN→
           SYN_ACK fine, cam2 sits at fail=9 / rx=0, handshake stalls. */
        set_ae_freeze(0);
        usleep(AE_PRE_FREEZE_SETTLE_MS * 1000);
        set_ae_freeze(1);
        usleep(AE_FREEZE_APPLY_MS * 1000);

        pthread_t rx_tid;
        if (pthread_create(&rx_tid, NULL, rx_thread, NULL) != 0) {
            perror("pthread_create");
            monocal_ended_ms = now_ms();
            monocal_ok = 0;
            monocal_set_state("monocal_failed", "rx_thread create failed");
            gpio_set(0);
            set_ae_freeze(0);
            return 1;
        }

        monocal_set_state("handshaking", NULL);
        /* 45s/try comfortably clears one ~35-37s round trip at 160ms/sym
         * (the old 35s single-shot frequently expired on a GOOD decode and
         * gave zero retransmit); 2 tries rescues a lone transient miss on
         * cam1. Worst case 90s before the bootstrap pivot — still well
         * under do_connect's ~230s, so fast-fail-to-bootstrap for a
         * genuinely-wrong cam1 pixel is preserved. */
        const int SHORT_HANDSHAKE_PER_TRY_MS = 45000;
        const int SHORT_HANDSHAKE_TRIES = 2;
        int handshake_ok = (do_connect_short(SHORT_HANDSHAKE_PER_TRY_MS,
                                             SHORT_HANDSHAKE_TRIES) == 0);

        if (handshake_ok) {
            /* HAPPY PATH: cam1 decoded our SYN. Branch on direction —
               default = we hold + peer scans (cam1's JSON refreshes);
               --reverse = we scan + peer holds (OUR JSON refreshes);
               --both   = fused Phase 1 then Phase 2 in this session, so
                          BOTH /opt/etc/calibration.json files refresh
                          without paying a second 55s handshake.

               In-session rate-change to 120ms was profiled 2026-05-14
               and made things ~10s slower, not faster: long CAL_REQ
               (136 syms) and CAL_DONE (168 syms) at 120ms incurred
               decode retries that outweighed the wire-time savings.
               Same root cause as the floor-drop attempt — grid_Δ≈37-52
               is below the DPLL reliability threshold for 120ms frames.
               Path forward to <235s would be (a) boost SNR (re-aim, or
               cleaner saved-cal), or (b) shorten the protocol frames
               (2-way handshake, leaner CAL_REQ payload). Staying at the
               handshake-rate (160ms floor) for now. */
            int rc;
            if (monocal_both) {
                /* Phase 1: cam2 (us) holds LEDs, cam1 scans → cam1.json. */
                rc = do_monocal_request();
                if (rc == 0) {
                    /* Inter-phase settle. After Phase 1's CAL_DONE lands,
                       cam1's handle_monocal_request returns to its
                       daemon-listen event loop. The 2s pause gives both
                       sides quiet time so:
                         - cam1's last_valid_frame_ms is fresh (no split-
                           brain), but its rx state is back to IDLE.
                         - Baselines on both sides re-settle after the
                           Phase 1 LED hold.
                         - Status JSON pollers can observe monocal_done
                           briefly before monocal_rev_req overwrites it. */
                    sleep(2);
                    fprintf(stderr,
                        "PROTO: monocal both — Phase 1 complete, starting Phase 2 "
                        "(reverse) on same session\n");
                    /* Phase 2: cam2 scans, cam1 holds → cam2.json. */
                    rc = do_monocal_request_reverse();
                }
            } else if (monocal_reverse) {
                rc = do_monocal_request_reverse();
            } else {
                rc = do_monocal_request();
            }
            running = 0;
            pthread_join(rx_tid, NULL);
            gpio_set(0);  /* defensive — match the failure-path cleanup */
            set_ae_freeze(0);
            return rc == 0 ? 0 : 1;
        }

        /* Reverse mode is intended for Phase 2 of a bidirectional cal —
           Phase 1 (regular monocal) just succeeded, so the link is known
           good. If the short handshake fails here, something is genuinely
           wrong (cam1 crashed, cam2 moved between phases, etc). Bootstrap
           fallback doesn't fit reverse semantics — we'd be holding LEDs,
           but the whole point of reverse is that we scan. Fail loud and
           tell the operator to re-run Phase 1.
           --both fuses Phase 1 + Phase 2 — bootstrap fallback IS still
           valid for it (Phase 1 part), so don't short-circuit there. */
        if (monocal_reverse) {
            fprintf(stderr,
                "PROTO: monocal-rev short-handshake failed — link not "
                "responsive. Run regular monocal (Phase 1) first to refresh "
                "cam1's pixel, then retry --reverse.\n");
            monocal_ended_ms = now_ms();
            monocal_ok = 0;
            monocal_set_state("monocal_rev_failed",
                "SYN unanswered; run Phase 1 first");
            running = 0;
            pthread_join(rx_tid, NULL);
            gpio_set(0);
            set_ae_freeze(0);
            return 1;
        }

        /* FALLBACK: cam1 didn't decode our SYN — pixel must be wrong.
           Drop the rx_thread (we don't need protocol decode in bootstrap
           mode) and pivot to a light-only sequence. */
        fprintf(stderr,
            "PROTO: short-handshake failed → bootstrap fallback "
            "(cam1's saved pixel is off; using light-only bootstrap)\n");
        log_event("BOOTSTRAP", "fallback engaged after short-handshake fail");
        monocal_set_state("bootstrap_fallback",
            "saved pixel may be wrong; light-only bootstrap");

        running = 0;
        pthread_join(rx_tid, NULL);
        running = 1;  /* re-arm for the search loop in do_search */

        /* Phase 1: hold LEDs solid 25s. The first ~5s wakes cam1's grid-
           watchdog (it requires WATCHDOG_TRIGGER_SAMPLES consecutive
           elevated samples). The remaining ~20s gives cam1 time to refine
           pixel and observe our falling edge. AE OFF on cam2 because we're
           only TXing — exposure freeze on the holder side is unnecessary. */
        set_ae_freeze(0);
        monocal_hold_start_ms = now_ms();
        monocal_set_state("bootstrap_hold",
            "holding LEDs solid 25s for cam1 grid-watchdog");
        hold_leds_on(BOOTSTRAP_HOLD_MS);
        monocal_hold_end_ms = now_ms();

        /* Phase 2: 2s OFF grace — covers cam2's own do_search baseline,
           AND gives cam1 the OFF-grace window before cam1 starts holding
           for our search. */
        usleep(BOOTSTRAP_BASELINE_GRACE_MS * 1000);

        /* Phase 3: cam2's own do_search. cam1 is now holding (per the
           watchdog handler's flow: refine → wait falling edge → OFF
           grace → hold 15s). do_search captures baseline, scans for
           cam1's beacon, refines to pixel, returns. */
        monocal_set_state("bootstrap_search",
            "searching for cam1's responding hold");
        int block, px, py, off_v, on_v, delta_v;
        int rc = do_search(BOOTSTRAP_HOLD_MS, &block, &px, &py,
                           &off_v, &on_v, &delta_v);
        if (rc != 0) {
            fprintf(stderr,
                "PROTO: bootstrap fallback — do_search did not acquire\n");
            monocal_ended_ms = now_ms();
            monocal_ok = 0;
            monocal_set_state("monocal_failed",
                "bootstrap fallback: did not see cam1's responding hold");
            /* Defensive LED-off — without this, a hold_leds_on partial-run
               or any other path that left GPIO HIGH at exit time leaves
               cam2 blasting IR until next manual intervention. Observed
               2026-05-10: webui showed `ir850=1 ir940=1` after a search-
               timeout, requiring `/api/cam2/led value=0` to clear. */
            gpio_set(0);
            set_ae_freeze(0);
            return 1;
        }

        /* Persist cam2's fresh cal. status_peak_brightness was set by
           find_peak_pixel_in_block inside do_search. */
        write_calibration_json(px, py, off_v, on_v, delta_v,
                               (uint8_t)status_peak_brightness);
        fprintf(stderr,
            "PROTO: bootstrap complete — cam2 sees cam1 at (%d,%d) "
            "block=%d delta=%d\n", px, py, block, delta_v);
        log_event("BOOTSTRAP", "complete pixel=(%d,%d)", px, py);
        printf("PIXEL: %d %d %d %d %d\n", px, py, on_v, delta_v, block);
        fflush(stdout);

        /* Surface the result through the same monocal-status fields the
           webui already polls (peer_pixel + monocal_done). */
        monocal_peer_x = px;
        monocal_peer_y = py;
        monocal_peer_b = (uint8_t)status_peak_brightness;
        monocal_peer_d = delta_v;
        monocal_have_peer_pixel = 1;
        monocal_ended_ms = now_ms();
        monocal_ok = 1;
        /* The bootstrap path inherently refreshes BOTH cams in one
         * light-only sequence: cam1 via its grid-watchdog refine during
         * our 25s hold, cam2 via the do_search above. For `--both` that
         * IS the complete bidirectional result, so emit the terminal
         * state the both-flow waits on (monocal_rev_done) rather than
         * monocal_done — otherwise the webui times out waiting for a
         * protocol Phase 2 that the bootstrap path legitimately subsumes.
         * Plain `monocal` (CAM1-only) still ends at monocal_done. */
        monocal_set_state(
            monocal_both ? "monocal_rev_done" : "monocal_done",
            monocal_both
              ? "bootstrap fallback: both cams refreshed (light-only path)"
              : "bootstrap fallback: cam1 refreshed via light-only path");
        gpio_set(0);  /* defensive — match the failure-path cleanup */
        set_ae_freeze(0);
        return 0;
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

        prof_mark("send: ae_freeze=1 + 1s settle");
        set_ae_freeze(1);
        usleep(1000000); /* 1s AE settle */
        prof_mark("send: settle done; spawning rx_thread");

        /* Start RX, connect, send, done */
        pthread_t rx_tid;
        pthread_create(&rx_tid, NULL, rx_thread, NULL);

        prof_mark("send: before do_connect (handshake start)");
        int connect_rc = do_connect();
        prof_mark(connect_rc == 0 ? "send: handshake complete"
                                  : "send: handshake FAILED");
        if (connect_rc == 0) {
            prof_mark("send: before do_send (DATA TX + ACK wait)");
            do_send(text);
            prof_mark("send: do_send returned");
        }

        running = 0;
        prof_mark("send: running=0; joining rx_thread");
        pthread_join(rx_tid, NULL);
        prof_mark("send: rx_thread joined; ae_freeze=0");
        set_ae_freeze(0);
        prof_mark("send: ae thawed; returning from main");
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

    if (strcmp(cmd, "tx-symbols") == 0) {
        /* Raw symbol-stream TX: hex-encoded bits, 8 symbols per byte (MSB first).
           Bypasses all framing — caller supplies the full on-wire symbol sequence.
           Used for resync experiments driven from host Python (protocol/frame.py). */
        if (argc < 3) {
            fprintf(stderr, "Usage: irlink tx-symbols <hex> [--speed N]\n");
            return 1;
        }
        const char *hex = NULL;
        for (int i = 2; i < argc; i++) {
            if ((strcmp(argv[i], "--speed") == 0 ||
                 strcmp(argv[i], "--block") == 0 ||
                 strcmp(argv[i], "--pixel") == 0 ||
                 strcmp(argv[i], "--roi-size") == 0) && i + 1 < argc) {
                i++;
                continue;
            }
            hex = argv[i];
            break;
        }
        if (!hex) {
            fprintf(stderr, "No symbol hex provided\n");
            return 1;
        }

        uint8_t bytes[MAX_SYMBOLS / 8];
        int n_bytes = hex_decode(hex, bytes, sizeof(bytes));
        if (n_bytes < 0) {
            fprintf(stderr, "Bad hex\n");
            return 1;
        }

        static uint8_t syms[MAX_SYMBOLS];
        int n_syms = 0;
        for (int b = 0; b < n_bytes; b++) {
            for (int i = 7; i >= 0 && n_syms < MAX_SYMBOLS; i--) {
                syms[n_syms++] = (bytes[b] >> i) & 1;
            }
        }

        set_ae_freeze(1);
        usleep(1000000);

        fprintf(stderr, "TX: %d raw symbols, speed=%dms (~%.1fs)\n",
                n_syms, symbol_ms, (n_syms * symbol_ms) / 1000.0);
        transmit_symbols(syms, n_syms);
        fprintf(stderr, "TX: done\n");

        set_ae_freeze(0);
        return 0;
    }

    fprintf(stderr, "Unknown command: %s\n", cmd);
    return 1;
}
