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
#include "ir_test_face.h"
#include "adc.h"

#ifdef HAS_IR_SENSOR

static const uint8_t tick_frequencies[] = {8, 16, 32, 64};
#define NUM_TICK_FREQS 4

static uint16_t _ir_test_read_sensor(void) {
    return adc_get_analog_value(HAL_GPIO_IRSENSE_pin());
}

static void _ir_test_reset_range(ir_test_state_t *state) {
    state->min_reading = 65535;
    state->max_reading = 0;
}

static void _ir_test_update_range(ir_test_state_t *state, uint16_t reading) {
    if (reading < state->min_reading) state->min_reading = reading;
    if (reading > state->max_reading) state->max_reading = reading;
}

static void _ir_test_check_transition(ir_test_state_t *state, uint16_t reading) {
    // Sensor is inverted: light = low reading. "Bright" = below threshold.
    bool bright = reading < state->threshold;
    if (bright != state->last_above_threshold) {
        state->transition_count++;
        state->last_above_threshold = bright;
    }
    if (bright) {
        watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    } else {
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
    }
}

static void _ir_test_display(ir_test_state_t *state) {
    char buf[7];

    switch (state->mode) {
        case IR_TEST_MODE_LIVE:
            watch_display_text(WATCH_POSITION_TOP, "IR");
            snprintf(buf, 7, "%-6d", state->current_reading);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_TEST_MODE_RANGE:
            if (state->show_max) {
                watch_display_text(WATCH_POSITION_TOP, "HI");
                snprintf(buf, 7, "%-6d", state->max_reading);
            } else {
                watch_display_text(WATCH_POSITION_TOP, "LO");
                snprintf(buf, 7, "%-6d", state->min_reading);
            }
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_TEST_MODE_COUNT:
            watch_display_text(WATCH_POSITION_TOP, "Cn");
            snprintf(buf, 7, "%-6d", state->transition_count);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        default:
            break;
    }
}

void ir_test_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(ir_test_state_t));
        memset(*context_ptr, 0, sizeof(ir_test_state_t));
        ir_test_state_t *state = (ir_test_state_t *)*context_ptr;
        _ir_test_reset_range(state);
        state->threshold = 32768; // midpoint default
    }
}

void ir_test_face_activate(void *context) {
    ir_test_state_t *state = (ir_test_state_t *)context;

    HAL_GPIO_IR_ENABLE_out();
    HAL_GPIO_IR_ENABLE_clr();
    HAL_GPIO_IRSENSE_pmuxen(HAL_GPIO_PMUX_ADC);
    adc_init();
    adc_enable();

    movement_request_tick_frequency(tick_frequencies[state->tick_freq_index]);
}

bool ir_test_face_loop(movement_event_t event, void *context) {
    ir_test_state_t *state = (ir_test_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            // Take an initial reading to seed threshold/range
            state->current_reading = _ir_test_read_sensor();
            _ir_test_update_range(state, state->current_reading);
            _ir_test_check_transition(state, state->current_reading);
            _ir_test_display(state);
            break;

        case EVENT_TICK:
        {
            state->current_reading = _ir_test_read_sensor();
            _ir_test_update_range(state, state->current_reading);
            _ir_test_check_transition(state, state->current_reading);

            // In RANGE mode, alternate between min/max every ~0.5 sec
            if (state->mode == IR_TEST_MODE_RANGE) {
                uint8_t freq = tick_frequencies[state->tick_freq_index];
                if (event.subsecond % (freq / 2) == 0) {
                    state->show_max = !state->show_max;
                }
            }

            _ir_test_display(state);
            break;
        }

        case EVENT_ALARM_BUTTON_UP:
            // Cycle display modes
            state->mode = (state->mode + 1) % IR_TEST_MODE_NUM_MODES;
            watch_clear_display();
            _ir_test_display(state);
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            // Suppress LED — it interferes with IR sensor
            break;

        case EVENT_LIGHT_BUTTON_UP:
            if (state->mode == IR_TEST_MODE_COUNT) {
                // Reset counter and set threshold halfway between min and max
                // Use RANGE mode first to measure the spread, then switch to COUNT
                state->transition_count = 0;
                if (state->max_reading > state->min_reading) {
                    state->threshold = state->min_reading + (state->max_reading - state->min_reading) / 2;
                } else {
                    state->threshold = state->current_reading;
                }
                state->last_above_threshold = state->current_reading > state->threshold;
            } else {
                // Reset min/max range
                _ir_test_reset_range(state);
            }
            _ir_test_display(state);
            break;

        case EVENT_LIGHT_REALLY_LONG_PRESS:
            // Cycle sample rate
            state->tick_freq_index = (state->tick_freq_index + 1) % NUM_TICK_FREQS;
            movement_request_tick_frequency(tick_frequencies[state->tick_freq_index]);
            // Briefly show the new frequency
            {
                char buf[8];
                watch_display_text(WATCH_POSITION_TOP, "Hz");
                snprintf(buf, sizeof(buf), "    %2d", tick_frequencies[state->tick_freq_index]);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
            }
            break;

        case EVENT_TIMEOUT:
            movement_move_to_face(0);
            break;

        case EVENT_LOW_ENERGY_UPDATE:
            watch_display_text(WATCH_POSITION_TOP, "IR");
            watch_display_text(WATCH_POSITION_BOTTOM, "  SLEp");
            watch_start_sleep_animation(1000);
            break;

        default:
            return movement_default_loop_handler(event);
    }

    return false; // keep awake for continuous sampling
}

void ir_test_face_resign(void *context) {
    (void) context;

    adc_disable();
    HAL_GPIO_IRSENSE_pmuxdis();
    HAL_GPIO_IRSENSE_off();
    HAL_GPIO_IR_ENABLE_off();

    movement_request_tick_frequency(1);
}

#endif // HAS_IR_SENSOR
