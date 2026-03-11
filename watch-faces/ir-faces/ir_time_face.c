/*
 * MIT License
 *
 * Copyright (c) 2026 Rob
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ir_time_face.h"
#include "adc.h"
#include "watch_deepsleep.h"
#include "filesystem.h"
#include "zones.h"

#ifdef HAS_IR_SENSOR

// At 64 Hz, 192 ticks = 3 seconds
#define IR_TIME_DELAY_TICKS 192

static void _ir_time_reset_decoder(ir_time_state_t *state) {
    state->decode_state = IR_TIME_CALIBRATE;
    state->cal_sum = 0;
    state->cal_count = 0;
    state->cal_min = 65535;
    state->cal_max = 0;
    state->baseline = 0;
    state->threshold = 0;
    state->noise_margin = 0;
    state->tick_count = 0;
    state->last_high = false;
    state->last_transition_counter = 0;
    state->interval_sum = 0;
    state->interval_count = 0;
    state->bit_period = 0;
    state->next_sample_counter = 0;
    state->bits_received = 0;
    state->current_byte = 0;
    state->protocol_version = 0;
    state->pkt_type = 0;
    state->pkt_length = 0;
    state->pkt_data_idx = 0;
    state->pkt_bytes_expected = 0;
    state->header_idx = 0;
    state->packets_received = 0;
    state->got_time = false;
    state->got_timezone = false;
    state->got_location = false;
    memset(state->pkt_data, 0, sizeof(state->pkt_data));
    memset(state->header_bytes, 0, sizeof(state->header_bytes));
}

static void _ir_time_display_status(ir_time_state_t *state) {
    watch_display_text(WATCH_POSITION_TOP, "IR");

    switch (state->decode_state) {
        case IR_TIME_CALIBRATE:
        {
            if (state->tick_count < IR_TIME_DELAY_TICKS) {
                char buf[8];
                uint8_t secs_left = (IR_TIME_DELAY_TICKS - state->tick_count) / 64 + 1;
                snprintf(buf, sizeof(buf), "CAL %1d ", secs_left);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
            } else {
                watch_display_text(WATCH_POSITION_BOTTOM, "CAL   ");
            }
            break;
        }
        case IR_TIME_LISTENING:
            watch_display_text(WATCH_POSITION_BOTTOM, " WAIT ");
            break;
        case IR_TIME_PREAMBLE:
            watch_display_text(WATCH_POSITION_BOTTOM, " SYnC ");
            watch_set_indicator(WATCH_INDICATOR_SIGNAL);
            break;
        case IR_TIME_VERSION:
            watch_display_text(WATCH_POSITION_BOTTOM, " VER  ");
            break;
        case IR_TIME_PKT_HEADER:
        case IR_TIME_PKT_DATA:
        {
            char buf[8];
            snprintf(buf, sizeof(buf), "RC  %1d", state->packets_received);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        }
        case IR_TIME_SUCCESS:
        {
            char buf[8];
            snprintf(buf, sizeof(buf), "OK  %1d", state->packets_received);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            watch_set_indicator(WATCH_INDICATOR_BELL);
            break;
        }
        case IR_TIME_ERROR:
            watch_display_text(WATCH_POSITION_BOTTOM, "  ERR ");
            break;
    }
}

static void _ir_time_apply_packet(ir_time_state_t *state) {
    state->packets_received++;
    printf("IR pkt: type=0x%02X len=%d applied (#%d)\n",
           state->pkt_type, state->pkt_length, state->packets_received);

    switch (state->pkt_type) {
        case IR_PKT_TIME: {
            if (state->pkt_length != 6) break;
            uint8_t year   = state->pkt_data[0];
            uint8_t month  = state->pkt_data[1];
            uint8_t day    = state->pkt_data[2];
            uint8_t hour   = state->pkt_data[3];
            uint8_t minute = state->pkt_data[4];
            uint8_t second = state->pkt_data[5];
            // Sanity checks
            if (year > 63 || month < 1 || month > 12 || day < 1 || day > 31 ||
                hour > 23 || minute > 59 || second > 59) break;
            watch_date_time_t dt;
            dt.reg = 0;
            dt.unit.year = year;
            dt.unit.month = month;
            dt.unit.day = day;
            dt.unit.hour = hour;
            dt.unit.minute = minute;
            dt.unit.second = second;
            printf("IR apply time: 20%02d-%02d-%02d %02d:%02d:%02d\n",
                   year, month, day, hour, minute, second);
            movement_set_local_date_time(dt);
            state->got_time = true;
            movement_play_note(BUZZER_NOTE_C6, 80);
            break;
        }
        case IR_PKT_TIMEZONE: {
            if (state->pkt_length != 1) break;
            int8_t half_hours = (int8_t)state->pkt_data[0];
            int32_t offset_seconds = (int32_t)half_hours * 1800;
            printf("IR apply tz: half_hours=%d offset=%ld\n",
                   half_hours, (long)offset_seconds);
            for (uint8_t i = 0; i < NUM_ZONE_NAMES; i++) {
                if (movement_get_current_timezone_offset_for_zone(i) == offset_seconds) {
                    movement_set_timezone_index(i);
                    state->got_timezone = true;
                    movement_play_note(BUZZER_NOTE_E6, 80);
                    break;
                }
            }
            break;
        }
        case IR_PKT_LOCATION: {
            if (state->pkt_length != 4) break;
            movement_location_t loc;
            loc.bit.latitude  = (int16_t)((state->pkt_data[1] << 8) | state->pkt_data[0]);
            loc.bit.longitude = (int16_t)((state->pkt_data[3] << 8) | state->pkt_data[2]);
            printf("IR apply loc: lat=%d lon=%d\n", loc.bit.latitude, loc.bit.longitude);
            watch_store_backup_data(loc.reg, 1);
            filesystem_write_file("location.u32", (char *)&loc.reg, sizeof(movement_location_t));
            state->got_location = true;
            movement_play_note(BUZZER_NOTE_G6, 80);
            break;
        }
        case IR_PKT_TOTP: {
            if (state->pkt_length == 0 || state->pkt_length > IR_MAX_PACKET_DATA) break;
            // pkt_data contains a UTF-8 otpauth:// URI (not null-terminated)
            // Append it as a line to totp_uris.txt
            char line[IR_MAX_PACKET_DATA + 2];
            memcpy(line, state->pkt_data, state->pkt_length);
            line[state->pkt_length] = '\n';
            line[state->pkt_length + 1] = '\0';
            printf("IR apply totp: %.*s\n", state->pkt_length, (char *)state->pkt_data);
            if (filesystem_file_exists("totp_uris.txt")) {
                filesystem_append_file("totp_uris.txt", line, state->pkt_length + 1);
            } else {
                filesystem_write_file("totp_uris.txt", line, state->pkt_length + 1);
            }
            movement_play_note(BUZZER_NOTE_A6, 80);
            break;
        }
        case IR_PKT_TOTP_CLR: {
            printf("IR apply totp_clr: deleting totp_uris.txt\n");
            if (filesystem_file_exists("totp_uris.txt")) {
                filesystem_rm("totp_uris.txt");
            }
            movement_play_note(BUZZER_NOTE_A4, 150);
            break;
        }
    }
}

// Start receiving the next byte (reset bit counter within byte)
static void _ir_time_start_byte(ir_time_state_t *state) {
    state->bits_received = 0;
    state->current_byte = 0;
}

// Try to sample a bit at the current RTC counter. Returns true if a full byte is ready.
static bool _ir_time_sample_bit(ir_time_state_t *state, uint16_t reading, uint32_t rtc_counter) {
    if (rtc_counter < state->next_sample_counter) return false;

    bool is_bright = reading < state->threshold;
    if (is_bright) {
        state->current_byte |= (1 << state->bits_received);
    }

    state->bits_received++;
    state->next_sample_counter += state->bit_period;

    return (state->bits_received >= 8);
}

static void _ir_time_process_tick(ir_time_state_t *state) {
    uint16_t reading = adc_get_analog_value(HAL_GPIO_IRSENSE_pin());
    uint32_t rtc_counter = watch_rtc_get_counter();
    state->tick_count++;

    switch (state->decode_state) {
        case IR_TIME_CALIBRATE:
        {
            if (state->tick_count <= IR_TIME_DELAY_TICKS) break;
            state->cal_sum += reading;
            state->cal_count++;
            if (reading < state->cal_min) state->cal_min = reading;
            if (reading > state->cal_max) state->cal_max = reading;

            if (state->cal_count >= IR_TIME_CAL_TICKS) {
                state->baseline = state->cal_sum / state->cal_count;
                uint16_t noise_range = state->cal_max - state->cal_min;
                state->noise_margin = noise_range + 10;
                if (state->baseline > state->noise_margin) {
                    state->threshold = state->baseline - state->noise_margin;
                } else {
                    state->threshold = 0;
                }
                printf("IR cal: baseline=%d min=%d max=%d threshold=%d\n",
                       state->baseline, state->cal_min, state->cal_max, state->threshold);
                state->last_high = false;
                state->decode_state = IR_TIME_LISTENING;
            }
            break;
        }

        case IR_TIME_LISTENING:
        {
            bool is_bright = reading < state->threshold;
            if (is_bright && !state->last_high) {
                state->last_transition_counter = rtc_counter;
                state->decode_state = IR_TIME_PREAMBLE;
                printf("IR listen: first bright counter=%lu adc=%d tick=%d\n",
                       (unsigned long)rtc_counter, reading, state->tick_count);
            }
            state->last_high = is_bright;
            break;
        }

        case IR_TIME_PREAMBLE:
        {
            bool is_bright = reading < state->threshold;
            if (is_bright != state->last_high) {
                uint32_t interval = rtc_counter - state->last_transition_counter;
                printf("IR preamble: transition %d interval=%lu counter=%lu adc=%d\n",
                       state->interval_count + 1, (unsigned long)interval,
                       (unsigned long)rtc_counter, reading);
                if (state->interval_count > 0) {
                    state->interval_sum += interval;
                }
                state->interval_count++;
                state->last_transition_counter = rtc_counter;

                uint8_t good = state->interval_count > 0 ? state->interval_count - 1 : 0;
                if (good >= IR_TIME_MIN_TRANSITIONS) {
                    state->bit_period = state->interval_sum / good;
                }
            } else if (state->bit_period > 0) {
                uint32_t since_last = rtc_counter - state->last_transition_counter;
                if (since_last > state->bit_period + state->bit_period / 2) {
                    // Sync found — start reading version byte
                    state->decode_state = IR_TIME_VERSION;
                    state->next_sample_counter = state->last_transition_counter
                                                 + 2 * state->bit_period
                                                 + state->bit_period / 2;
                    _ir_time_start_byte(state);
                    printf("IR sync: bit_period=%lu intervals=%d next_sample=%lu\n",
                           (unsigned long)state->bit_period, state->interval_count,
                           (unsigned long)state->next_sample_counter);
                }
            }
            state->last_high = is_bright;
            break;
        }

        case IR_TIME_VERSION:
        {
            if (_ir_time_sample_bit(state, reading, rtc_counter)) {
                state->protocol_version = state->current_byte;
                printf("IR version: %d\n", state->protocol_version);
                if (state->protocol_version != IR_PROTOCOL_VERSION) {
                    state->decode_state = IR_TIME_ERROR;
                    movement_play_note(BUZZER_NOTE_A3, 200);
                    break;
                }
                // Start reading first packet header
                state->decode_state = IR_TIME_PKT_HEADER;
                state->header_idx = 0;
                _ir_time_start_byte(state);
            }
            break;
        }

        case IR_TIME_PKT_HEADER:
        {
            if (_ir_time_sample_bit(state, reading, rtc_counter)) {
                state->header_bytes[state->header_idx] = state->current_byte;
                state->header_idx++;
                _ir_time_start_byte(state);

                if (state->header_idx >= 2) {
                    state->pkt_type = state->header_bytes[0];
                    state->pkt_length = state->header_bytes[1];

                    printf("IR pkt header: type=0x%02X len=%d\n",
                           state->pkt_type, state->pkt_length);

                    // End sentinel
                    if (state->pkt_type == IR_PKT_END && state->pkt_length == 0) {
                        // Still need to read the checksum byte (should be 0x00)
                        state->pkt_data_idx = 0;
                        state->pkt_bytes_expected = 1; // just checksum
                        state->decode_state = IR_TIME_PKT_DATA;
                        break;
                    }

                    if (state->pkt_length > IR_MAX_PACKET_DATA) {
                        printf("IR pkt: length %d too large\n", state->pkt_length);
                        state->decode_state = IR_TIME_ERROR;
                        movement_play_note(BUZZER_NOTE_A3, 200);
                        break;
                    }

                    state->pkt_data_idx = 0;
                    state->pkt_bytes_expected = state->pkt_length + 1; // data + checksum
                    memset(state->pkt_data, 0, sizeof(state->pkt_data));
                    state->decode_state = IR_TIME_PKT_DATA;
                }
            }
            break;
        }

        case IR_TIME_PKT_DATA:
        {
            if (_ir_time_sample_bit(state, reading, rtc_counter)) {
                uint8_t byte = state->current_byte;
                uint8_t total_data_bytes = state->pkt_length; // 0 for end sentinel

                if (state->pkt_data_idx < total_data_bytes) {
                    // Data byte
                    state->pkt_data[state->pkt_data_idx] = byte;
                    state->pkt_data_idx++;
                    _ir_time_start_byte(state);
                } else {
                    // Checksum byte
                    uint8_t expected = (state->pkt_type + state->pkt_length) & 0xFF;
                    for (uint8_t i = 0; i < total_data_bytes; i++) {
                        expected = (expected + state->pkt_data[i]) & 0xFF;
                    }

                    if (byte != expected) {
                        printf("IR pkt: checksum fail got=0x%02X expected=0x%02X\n",
                               byte, expected);
                        state->decode_state = IR_TIME_ERROR;
                        movement_play_note(BUZZER_NOTE_A3, 200);
                        break;
                    }

                    // End sentinel with valid checksum
                    if (state->pkt_type == IR_PKT_END) {
                        state->decode_state = IR_TIME_SUCCESS;
                        printf("IR stream complete: %d packets\n", state->packets_received);
                        movement_play_note(BUZZER_NOTE_C6, 80);
                        movement_play_note(BUZZER_NOTE_E6, 80);
                        movement_play_note(BUZZER_NOTE_G6, 150);
                        break;
                    }

                    // Apply this packet
                    _ir_time_apply_packet(state);

                    // Ready for next packet header
                    state->decode_state = IR_TIME_PKT_HEADER;
                    state->header_idx = 0;
                    _ir_time_start_byte(state);
                }
            }
            break;
        }

        case IR_TIME_SUCCESS:
        case IR_TIME_ERROR:
            break;
    }
}

void ir_time_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(ir_time_state_t));
        memset(*context_ptr, 0, sizeof(ir_time_state_t));
    }
}

void ir_time_face_activate(void *context) {
    ir_time_state_t *state = (ir_time_state_t *)context;

    HAL_GPIO_IR_ENABLE_out();
    HAL_GPIO_IR_ENABLE_clr();
    HAL_GPIO_IRSENSE_pmuxen(HAL_GPIO_PMUX_ADC);
    adc_init();
    adc_enable();

    movement_request_tick_frequency(64);
    _ir_time_reset_decoder(state);

    watch_clear_display();
    watch_display_text(WATCH_POSITION_TOP, "IR");
    watch_display_text(WATCH_POSITION_BOTTOM, "CAL  3");
}

bool ir_time_face_loop(movement_event_t event, void *context) {
    ir_time_state_t *state = (ir_time_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            _ir_time_display_status(state);
            break;

        case EVENT_TICK:
            _ir_time_process_tick(state);
            if (state->tick_count % 8 == 0 ||
                state->decode_state == IR_TIME_SUCCESS ||
                state->decode_state == IR_TIME_ERROR) {
                _ir_time_display_status(state);
            }
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            // Suppress LED — it interferes with IR sensor
            break;

        case EVENT_LIGHT_BUTTON_UP:
            // Reset and re-listen
            watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
            watch_clear_indicator(WATCH_INDICATOR_BELL);
            watch_clear_display();
            _ir_time_reset_decoder(state);
            _ir_time_display_status(state);
            break;

        case EVENT_TIMEOUT:
            if (state->decode_state == IR_TIME_PKT_DATA ||
                state->decode_state == IR_TIME_PKT_HEADER ||
                state->decode_state == IR_TIME_VERSION ||
                state->decode_state == IR_TIME_PREAMBLE) {
                break;
            }
            movement_move_to_face(0);
            break;

        default:
            return movement_default_loop_handler(event);
    }

    return false;
}

void ir_time_face_resign(void *context) {
    (void) context;

    adc_disable();
    HAL_GPIO_IRSENSE_pmuxdis();
    HAL_GPIO_IRSENSE_off();
    HAL_GPIO_IR_ENABLE_off();

    watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
    watch_clear_indicator(WATCH_INDICATOR_BELL);
    movement_request_tick_frequency(1);
}

#endif // HAS_IR_SENSOR
