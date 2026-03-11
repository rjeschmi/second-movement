/*
 * Standalone test harness for IR time decoder.
 * Extracts the decoder state machine and feeds it synthetic ADC values
 * generated from an .irsig file.
 *
 * Build:  cc -o test_ir_decode test_ir_decode.c
 * Usage:  ./test_ir_decode path/to/file.irsig
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// ── Protocol constants (from ir_time_face.h) ────────────────────────
#define IR_TIME_CAL_TICKS 32
#define IR_TIME_DATA_BITS 56
#define IR_TIME_MIN_TRANSITIONS 4
#define IR_TIME_DELAY_TICKS 192  // 3 seconds at 64 Hz

#define RTC_HZ 128
#define FACE_HZ 64
#define ADC_DARK 6500
#define ADC_BRIGHT 4200

// ── Decoder state (from ir_time_face.h) ─────────────────────────────
typedef enum {
    IR_TIME_CALIBRATE,
    IR_TIME_LISTENING,
    IR_TIME_PREAMBLE,
    IR_TIME_DATA,
    IR_TIME_SUCCESS,
    IR_TIME_ERROR,
} ir_time_decode_state_t;

static const char *state_names[] = {
    "CALIBRATE", "LISTENING", "PREAMBLE", "DATA", "SUCCESS", "ERROR"
};

typedef struct {
    ir_time_decode_state_t decode_state;
    uint32_t cal_sum;
    uint16_t cal_count;
    uint16_t cal_min;
    uint16_t cal_max;
    uint16_t baseline;
    uint16_t threshold;
    uint16_t noise_margin;
    uint16_t tick_count;
    bool last_high;
    uint32_t last_transition_counter;
    uint32_t interval_sum;
    uint8_t interval_count;
    uint32_t bit_period;
    uint32_t next_sample_counter;
    uint8_t bits_received;
    uint8_t data[7];
    uint8_t year, month, day, hour, minute, second;
} ir_time_state_t;

// ── Signal file ─────────────────────────────────────────────────────
typedef struct {
    int bit_period_ms;
    int frame[256];
    int frame_len;
    int data_bytes[6];
    int checksum;
} ir_signal_t;

// Minimal JSON parser — just enough for .irsig files
static int parse_irsig(const char *path, ir_signal_t *sig) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc(len + 1);
    fread(buf, 1, len, f);
    buf[len] = 0;
    fclose(f);

    // Parse bitPeriodMs
    char *p = strstr(buf, "\"bitPeriodMs\"");
    if (!p) { free(buf); return -1; }
    p = strchr(p, ':');
    sig->bit_period_ms = (int)strtol(p + 1, NULL, 10);

    // Parse frame array
    p = strstr(buf, "\"frame\"");
    if (!p) { free(buf); return -1; }
    p = strchr(p, '[');
    sig->frame_len = 0;
    p++; // skip '['
    while (*p && *p != ']') {
        if (*p >= '0' && *p <= '9') {
            sig->frame[sig->frame_len++] = (int)strtol(p, &p, 10);
        } else {
            p++;
        }
    }

    // Parse dataBytes
    p = strstr(buf, "\"dataBytes\"");
    if (p) {
        p = strchr(p, '[');
        p++;
        for (int i = 0; i < 6 && *p && *p != ']'; ) {
            if (*p >= '0' && *p <= '9') {
                sig->data_bytes[i++] = (int)strtol(p, &p, 10);
            } else {
                p++;
            }
        }
    }

    // Parse checksum
    p = strstr(buf, "\"checksum\"");
    if (p) {
        p = strchr(p, ':');
        sig->checksum = (int)strtol(p + 1, NULL, 10);
    }

    free(buf);
    return 0;
}

// ── Decoder logic (extracted from ir_time_face.c) ───────────────────

static void reset_decoder(ir_time_state_t *s) {
    s->decode_state = IR_TIME_CALIBRATE;
    s->cal_sum = 0;
    s->cal_count = 0;
    s->cal_min = 65535;
    s->cal_max = 0;
    s->baseline = 0;
    s->threshold = 0;
    s->noise_margin = 0;
    s->tick_count = 0;
    s->last_high = false;
    s->last_transition_counter = 0;
    s->interval_sum = 0;
    s->interval_count = 0;
    s->bit_period = 0;
    s->next_sample_counter = 0;
    s->bits_received = 0;
    memset(s->data, 0, sizeof(s->data));
}

static bool validate_and_decode(ir_time_state_t *s) {
    s->year   = s->data[0];
    s->month  = s->data[1];
    s->day    = s->data[2];
    s->hour   = s->data[3];
    s->minute = s->data[4];
    s->second = s->data[5];

    if (s->year > 63) return false;
    if (s->month < 1 || s->month > 12) return false;
    if (s->day < 1 || s->day > 31) return false;
    if (s->hour > 23) return false;
    if (s->minute > 59) return false;
    if (s->second > 59) return false;

    uint8_t checksum = 0;
    for (int i = 0; i < 6; i++)
        checksum = (checksum + s->data[i]) & 0xFF;
    if (checksum != s->data[6]) {
        printf("  !! checksum mismatch: computed=%d stored=%d\n", checksum, s->data[6]);
    }
    return true;
}

static void process_tick(ir_time_state_t *s, uint16_t reading, uint32_t rtc_counter) {
    s->tick_count++;

    switch (s->decode_state) {
        case IR_TIME_CALIBRATE:
            if (s->tick_count <= IR_TIME_DELAY_TICKS) break;
            s->cal_sum += reading;
            s->cal_count++;
            if (reading < s->cal_min) s->cal_min = reading;
            if (reading > s->cal_max) s->cal_max = reading;

            if (s->cal_count >= IR_TIME_CAL_TICKS) {
                s->baseline = s->cal_sum / s->cal_count;
                uint16_t noise_range = s->cal_max - s->cal_min;
                s->noise_margin = noise_range + 10;
                if (s->baseline > s->noise_margin)
                    s->threshold = s->baseline - s->noise_margin;
                else
                    s->threshold = 0;
                printf("IR cal: baseline=%d min=%d max=%d threshold=%d\n",
                       s->baseline, s->cal_min, s->cal_max, s->threshold);
                s->last_high = false;
                s->decode_state = IR_TIME_LISTENING;
            }
            break;

        case IR_TIME_LISTENING: {
            bool is_bright = reading < s->threshold;
            if (is_bright && !s->last_high) {
                s->last_transition_counter = rtc_counter;
                s->decode_state = IR_TIME_PREAMBLE;
                printf("IR listen: first bright counter=%u adc=%d tick=%d\n",
                       rtc_counter, reading, s->tick_count);
            }
            s->last_high = is_bright;
            break;
        }

        case IR_TIME_PREAMBLE: {
            bool is_bright = reading < s->threshold;
            if (is_bright != s->last_high) {
                uint32_t interval = rtc_counter - s->last_transition_counter;
                printf("IR preamble: transition %d interval=%u counter=%u adc=%d%s\n",
                       s->interval_count + 1, interval, rtc_counter, reading,
                       s->interval_count == 0 ? " (skipped - partial)" : "");
                // Skip first interval (partial — first bright detected mid-bit)
                if (s->interval_count > 0)
                    s->interval_sum += interval;
                s->interval_count++;
                s->last_transition_counter = rtc_counter;

                uint8_t good = s->interval_count > 0 ? s->interval_count - 1 : 0;
                if (good >= IR_TIME_MIN_TRANSITIONS)
                    s->bit_period = s->interval_sum / good;
            } else if (s->bit_period > 0) {
                uint32_t since_last = rtc_counter - s->last_transition_counter;
                if (since_last > s->bit_period + s->bit_period / 2) {
                    s->decode_state = IR_TIME_DATA;
                    s->bits_received = 0;
                    s->next_sample_counter = s->last_transition_counter
                                             + 2 * s->bit_period
                                             + s->bit_period / 2;
                    printf("IR sync: bit_period=%u intervals=%d next_sample=%u\n",
                           s->bit_period, s->interval_count, s->next_sample_counter);
                }
            }
            s->last_high = is_bright;
            break;
        }

        case IR_TIME_DATA:
            if (rtc_counter >= s->next_sample_counter) {
                bool is_bright = reading < s->threshold;
                uint8_t byte_idx = s->bits_received / 8;
                uint8_t bit_idx = s->bits_received % 8;

                if (is_bright)
                    s->data[byte_idx] |= (1 << bit_idx);

                printf("IR data: bit %2d = %d  counter=%u  byte[%d].%d  adc=%d\n",
                       s->bits_received, is_bright ? 1 : 0,
                       rtc_counter, byte_idx, bit_idx, reading);

                s->bits_received++;
                s->next_sample_counter += s->bit_period;

                if (s->bits_received >= IR_TIME_DATA_BITS) {
                    if (validate_and_decode(s)) {
                        s->decode_state = IR_TIME_SUCCESS;
                    } else {
                        s->decode_state = IR_TIME_ERROR;
                    }
                }
            }
            break;

        case IR_TIME_SUCCESS:
        case IR_TIME_ERROR:
            break;
    }
}

// ── Main: simulate RTC + face ticks + IR playback ───────────────────

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <file.irsig>\n", argv[0]);
        return 1;
    }

    ir_signal_t sig;
    if (parse_irsig(argv[1], &sig) < 0) return 1;

    printf("Loaded: %d bits @ %dms bit period\n", sig.frame_len, sig.bit_period_ms);
    printf("Expected data: ");
    for (int i = 0; i < 6; i++) printf("%d ", sig.data_bytes[i]);
    printf("checksum=%d\n", sig.checksum);
    printf("Frame: ");
    for (int i = 0; i < sig.frame_len; i++) printf("%d", sig.frame[i]);
    printf("\n\n");

    // ticks per bit at 128 Hz RTC
    int tpb = (sig.bit_period_ms * RTC_HZ + 500) / 1000;  // round
    printf("RTC ticks per bit (tpb): %d\n\n", tpb);

    ir_time_state_t state;
    memset(&state, 0, sizeof(state));
    reset_decoder(&state);

    // Simulate:
    // - RTC counter increments every tick (128 Hz)
    // - Face gets called every 2nd tick (64 Hz) — on odd counter values
    // - IR playback starts after calibration is done
    //   (we start it a few ticks after cal finishes, simulating user clicking Play)

    uint32_t playback_start = 0;  // RTC counter when playback begins
    bool playback_started = false;
    int playback_delay_ticks = 10; // face ticks after LISTENING before signal starts

    // Run enough ticks: calibration + signal + margin
    int total_rtc_ticks = (IR_TIME_DELAY_TICKS + IR_TIME_CAL_TICKS) * 2  // cal phase
                          + playback_delay_ticks * 2                       // gap
                          + sig.frame_len * tpb                            // signal
                          + 200;                                           // margin

    int face_ticks_in_listening = 0;

    for (uint32_t counter = 1; counter <= (uint32_t)total_rtc_ticks; counter++) {
        // Face fires at 64 Hz = odd counter values
        bool face_tick = (counter & 1) == 1;
        if (!face_tick) continue;

        // Determine ADC reading
        uint16_t adc = ADC_DARK;  // default: dark (no signal)

        if (playback_started) {
            int elapsed = (int)(counter - playback_start);
            int bi = elapsed / tpb;
            if (bi >= 0 && bi < sig.frame_len) {
                adc = sig.frame[bi] == 1 ? ADC_BRIGHT : ADC_DARK;
            }
        }

        // Track when to start playback
        if (state.decode_state == IR_TIME_LISTENING && !playback_started) {
            face_ticks_in_listening++;
            if (face_ticks_in_listening >= playback_delay_ticks) {
                playback_start = counter;
                playback_started = true;
                printf(">> Playback started at counter=%u\n\n", counter);
            }
        }

        process_tick(&state, adc, counter);

        if (state.decode_state == IR_TIME_SUCCESS) {
            printf("\n=== SUCCESS ===\n");
            printf("Decoded: 20%02d-%02d-%02d %02d:%02d:%02d\n",
                   state.year, state.month, state.day,
                   state.hour, state.minute, state.second);
            printf("Raw bytes: ");
            for (int i = 0; i < 7; i++) printf("%02X ", state.data[i]);
            printf("\n");
            return 0;
        }
        if (state.decode_state == IR_TIME_ERROR) {
            printf("\n=== ERROR ===\n");
            printf("Raw bytes: ");
            for (int i = 0; i < 7; i++) printf("%02X ", state.data[i]);
            printf("\nExpected:  ");
            for (int i = 0; i < 6; i++) printf("%02X ", sig.data_bytes[i]);
            printf("%02X\n", sig.checksum);
            return 1;
        }
    }

    printf("\n=== TIMEOUT — state=%s bits_received=%d ===\n",
           state_names[state.decode_state], state.bits_received);
    return 1;
}
