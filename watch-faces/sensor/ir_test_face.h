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

#pragma once

#include "movement.h"

#ifdef HAS_IR_SENSOR

/*
 * IR TEST FACE
 *
 * A watch face for testing the IR sensor on the Sensor Watch Pro.
 * Designed to evaluate the sensor's ability to detect light changes
 * for use as a one-way communication channel.
 *
 * Modes (cycle with ALARM button):
 *   LIVE  - Shows real-time ADC reading (0-65535)
 *   RANGE - Shows min and max readings (alternates)
 *   COUNT - Counts detected transitions (light/dark crossings)
 *
 * Controls:
 *   ALARM  - Cycle through display modes
 *   LIGHT  - In LIVE/RANGE: reset min/max. In COUNT: reset counter and set new threshold.
 *   LIGHT really long press - Cycle sample rate (8, 16, 32, 64 Hz)
 *
 * The SIGNAL indicator lights when reading is above the threshold.
 */

typedef enum {
    IR_TEST_MODE_LIVE,
    IR_TEST_MODE_RANGE,
    IR_TEST_MODE_COUNT,
    IR_TEST_MODE_NUM_MODES,
} ir_test_mode_t;

typedef struct {
    ir_test_mode_t mode;
    uint16_t current_reading;
    uint16_t min_reading;
    uint16_t max_reading;
    uint16_t threshold;
    uint16_t transition_count;
    bool last_above_threshold;
    uint8_t tick_freq_index;    // index into frequency table: 0=8, 1=16, 2=32, 3=64
    bool show_max;              // for RANGE mode alternation
} ir_test_state_t;

void ir_test_face_setup(uint8_t watch_face_index, void **context_ptr);
void ir_test_face_activate(void *context);
bool ir_test_face_loop(movement_event_t event, void *context);
void ir_test_face_resign(void *context);

#define ir_test_face ((const watch_face_t){ \
    ir_test_face_setup, \
    ir_test_face_activate, \
    ir_test_face_loop, \
    ir_test_face_resign, \
    NULL, \
})

#endif // HAS_IR_SENSOR
