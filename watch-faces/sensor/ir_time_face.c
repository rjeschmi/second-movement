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
    state->last_transition_tick = 0;
    state->interval_sum = 0;
    state->interval_count = 0;
    state->bit_period = 0;
    state->next_sample_tick = 0;
    state->bits_received = 0;
    memset(state->data, 0, sizeof(state->data));
}

static void _ir_time_display_status(ir_time_state_t *state) {
    watch_display_text(WATCH_POSITION_TOP, "IR");

    switch (state->decode_state) {
        case IR_TIME_CALIBRATE:
        {
            // Show countdown during the 3-second delay
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
        case IR_TIME_DATA:
        {
            char buf[8];
            uint8_t pct = (state->bits_received * 100) / IR_TIME_DATA_BITS;
            snprintf(buf, sizeof(buf), "RC %3d", pct);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        }
        case IR_TIME_SUCCESS:
        {
            char buf[12];
            snprintf(buf, sizeof(buf), "%2d%02d%02d",
                     state->hour, state->minute, state->second);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            watch_set_indicator(WATCH_INDICATOR_BELL);
            break;
        }
        case IR_TIME_ERROR:
        {
            // Show first 3 received bytes for debugging
            char buf[12];
            snprintf(buf, sizeof(buf), "%02X%02X%02X",
                     state->data[0], state->data[1], state->data[2]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        }
    }
}

static bool _ir_time_validate_and_decode(ir_time_state_t *state) {
    state->year   = state->data[0];
    state->month  = state->data[1];
    state->day    = state->data[2];
    state->hour   = state->data[3];
    state->minute = state->data[4];
    state->second = state->data[5];

    // Sanity checks
    if (state->year > 63) return false;
    if (state->month < 1 || state->month > 12) return false;
    if (state->day < 1 || state->day > 31) return false;
    if (state->hour > 23) return false;
    if (state->minute > 59) return false;
    if (state->second > 59) return false;

    // Checksum verification
    uint8_t checksum = 0;
    for (int i = 0; i < 6; i++) {
        checksum = (checksum + state->data[i]) & 0xFF;
    }
    if (checksum != state->data[6]) {
        // Checksum failed but data looks valid — accept anyway for now
        // TODO: re-enable strict checksum once protocol is proven reliable
    }

    return true;
}

static void _ir_time_process_tick(ir_time_state_t *state) {
    uint16_t reading = adc_get_analog_value(HAL_GPIO_IRSENSE_pin());
    state->tick_count++;

    switch (state->decode_state) {
        case IR_TIME_CALIBRATE:
        {
            // Skip the first 3 seconds to let LED interference settle
            if (state->tick_count <= IR_TIME_DELAY_TICKS) {
                break;
            }
            // Then collect calibration samples
            state->cal_sum += reading;
            state->cal_count++;
            if (reading < state->cal_min) state->cal_min = reading;
            if (reading > state->cal_max) state->cal_max = reading;

            if (state->cal_count >= IR_TIME_CAL_TICKS) {
                state->baseline = state->cal_sum / state->cal_count;
                // Sensor is inverted: bright light = low reading, dark = high reading.
                // Threshold is set BELOW baseline so that light causes a crossing.
                uint16_t noise_range = state->cal_max - state->cal_min;
                state->noise_margin = noise_range + 10;
                // Subtract margin from baseline (light makes reading drop)
                if (state->baseline > state->noise_margin) {
                    state->threshold = state->baseline - state->noise_margin;
                } else {
                    state->threshold = 0;
                }
                state->last_high = false;
                state->decode_state = IR_TIME_LISTENING;
            }
            break;
        }

        case IR_TIME_LISTENING:
        {
            // Inverted sensor: light = low reading, so "bright" = below threshold
            bool is_bright = reading < state->threshold;
            if (is_bright && !state->last_high) {
                state->last_transition_tick = state->tick_count;
                state->decode_state = IR_TIME_PREAMBLE;
            }
            state->last_high = is_bright;
            break;
        }

        case IR_TIME_PREAMBLE:
        {
            bool is_bright = reading < state->threshold;
            if (is_bright != state->last_high) {
                uint16_t interval = state->tick_count - state->last_transition_tick;
                state->interval_sum += interval;
                state->interval_count++;
                state->last_transition_tick = state->tick_count;

                if (state->interval_count >= IR_TIME_MIN_TRANSITIONS) {
                    state->bit_period = state->interval_sum / state->interval_count;
                }
            } else if (state->bit_period > 0) {
                uint16_t since_last = state->tick_count - state->last_transition_tick;
                if (since_last > state->bit_period + state->bit_period / 2) {
                    // Missing transition = start marker found
                    state->decode_state = IR_TIME_DATA;
                    state->bits_received = 0;
                    state->next_sample_tick = state->last_transition_tick
                                             + 2 * state->bit_period
                                             + state->bit_period / 2;
                }
            }
            state->last_high = is_bright;
            break;
        }

        case IR_TIME_DATA:
        {
            if (state->tick_count >= state->next_sample_tick) {
                bool is_bright = reading < state->threshold;
                uint8_t byte_idx = state->bits_received / 8;
                uint8_t bit_idx = state->bits_received % 8;

                if (is_bright) {
                    state->data[byte_idx] |= (1 << bit_idx);
                }

                state->bits_received++;
                state->next_sample_tick += state->bit_period;

                if (state->bits_received >= IR_TIME_DATA_BITS) {
                    if (_ir_time_validate_and_decode(state)) {
                        state->decode_state = IR_TIME_SUCCESS;
                        movement_play_note(BUZZER_NOTE_C5, 100);
                    } else {
                        state->decode_state = IR_TIME_ERROR;
                        movement_play_note(BUZZER_NOTE_A3, 200);
                    }
                }
            }
            break;
        }

        case IR_TIME_SUCCESS:
        case IR_TIME_ERROR:
            break;
    }
}

static void _ir_time_apply(ir_time_state_t *state) {
    watch_date_time_t dt;
    dt.reg = 0;
    dt.unit.year = state->year; // protocol year is offset from 2020, same as RTC
    dt.unit.month = state->month;
    dt.unit.day = state->day;
    dt.unit.hour = state->hour;
    dt.unit.minute = state->minute;
    dt.unit.second = state->second;
    movement_set_local_date_time(dt);
    movement_play_note(BUZZER_NOTE_C6, 80);
    movement_play_note(BUZZER_NOTE_E6, 80);
    movement_play_note(BUZZER_NOTE_G6, 150);
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

        case EVENT_ALARM_BUTTON_DOWN:
            if (state->decode_state == IR_TIME_SUCCESS) {
                _ir_time_apply(state);
                state->decode_state = IR_TIME_ERROR; // stop refreshing display
                watch_clear_display();
                watch_display_text(WATCH_POSITION_TOP, "IR");
                watch_display_text(WATCH_POSITION_BOTTOM, "  SET ");
            }
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            // Suppress LED — it interferes with IR sensor
            break;

        case EVENT_LIGHT_BUTTON_UP:
            // Reset and re-listen (with 3s delay for LED to settle)
            watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
            watch_clear_indicator(WATCH_INDICATOR_BELL);
            watch_clear_display();
            _ir_time_reset_decoder(state);
            _ir_time_display_status(state);
            break;

        case EVENT_TIMEOUT:
            if (state->decode_state == IR_TIME_DATA ||
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
