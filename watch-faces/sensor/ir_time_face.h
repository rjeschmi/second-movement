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
 * IR TIME SYNC
 *
 * Receives a time value encoded as light pulses from a phone screen
 * and sets the watch RTC.
 *
 * Protocol: OOK with fixed bit period
 *   - 8-bit alternating preamble (1,0,1,0,...) for sync
 *   - 2-bit start marker (1,1) detected as missing transition
 *   - 7 data bytes (LSB first): year-2020, month, day, hour, min, sec, checksum
 *   - Bit period auto-measured from preamble transitions
 *
 * Usage:
 *   1. Navigate to this face — it auto-calibrates ambient light (~0.5s)
 *   2. Shows "WAIT" — hold phone screen against sensor
 *   3. Start the sender app — face detects preamble ("SYNC") then receives ("RECV")
 *   4. On success: shows decoded time, press ALARM to apply
 *   5. Press LIGHT to reset and listen again
 *   6. On error: shows "ERR", press LIGHT to retry
 */

#define IR_TIME_CAL_TICKS 32        // calibration samples (~0.5s at 64 Hz)
#define IR_TIME_DATA_BITS 56        // 7 bytes * 8 bits
#define IR_TIME_MIN_TRANSITIONS 4   // minimum preamble transitions before sync
#define IR_TIME_YEAR_OFFSET 2020

typedef enum {
    IR_TIME_CALIBRATE,
    IR_TIME_LISTENING,
    IR_TIME_PREAMBLE,
    IR_TIME_DATA,
    IR_TIME_SUCCESS,
    IR_TIME_ERROR,
} ir_time_decode_state_t;

typedef struct {
    ir_time_decode_state_t decode_state;

    // Calibration
    uint32_t cal_sum;
    uint16_t cal_count;
    uint16_t cal_min;
    uint16_t cal_max;
    uint16_t baseline;      // ambient average
    uint16_t threshold;     // baseline + margin
    uint16_t noise_margin;  // half the observed noise range during cal

    // Tick tracking
    uint16_t tick_count;
    bool last_high;

    // Preamble measurement
    uint16_t last_transition_tick;
    uint32_t interval_sum;
    uint8_t interval_count;
    uint16_t bit_period;    // measured bit period in ticks

    // Data reception
    uint16_t next_sample_tick;
    uint8_t bits_received;
    uint8_t data[7];        // 6 data bytes + 1 checksum

    // Decoded result
    uint8_t year;           // offset from 2020
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} ir_time_state_t;

void ir_time_face_setup(uint8_t watch_face_index, void **context_ptr);
void ir_time_face_activate(void *context);
bool ir_time_face_loop(movement_event_t event, void *context);
void ir_time_face_resign(void *context);

#define ir_time_face ((const watch_face_t){ \
    ir_time_face_setup, \
    ir_time_face_activate, \
    ir_time_face_loop, \
    ir_time_face_resign, \
    NULL, \
})

#endif // HAS_IR_SENSOR
