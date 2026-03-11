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
 * IR CONFIG SYNC (Protocol v2)
 *
 * Receives configuration packets encoded as light pulses from a phone
 * screen or flashlight and applies them to the watch.
 *
 * Protocol v2: OOK with fixed bit period
 *   - 8-bit alternating preamble (1,0,1,0,...) for sync
 *   - 2-bit start marker (1,1) detected as missing transition
 *   - 1 version byte (0x02)
 *   - TLV packets: [type: 8b] [length: 8b] [data: length bytes] [checksum: 8b]
 *   - End sentinel: type=0x00, length=0x00, checksum=0x00
 *   - Bit period auto-measured from preamble transitions
 *
 * Packet types:
 *   0x01 - Time (6 bytes: year-2020, month, day, hour, min, sec)
 *   0x02 - Timezone (1 byte: signed half-hours from UTC)
 *   0x03 - Location (4 bytes: lat int16 + lon int16, hundredths of degree)
 *   0x00 - End of stream
 *
 * Usage:
 *   1. Navigate to this face — it auto-calibrates ambient light (~3s)
 *   2. Shows "WAIT" — hold phone against sensor
 *   3. Start the sender app — face detects preamble ("SYNC") then receives
 *   4. Each packet is applied immediately on valid checksum
 *   5. Press LIGHT to reset and listen again
 */

#define IR_TIME_CAL_TICKS 32        // calibration samples (~0.5s at 64 Hz)
#define IR_TIME_MIN_TRANSITIONS 4   // minimum preamble transitions before sync
#define IR_TIME_YEAR_OFFSET 2020
#define IR_MAX_PACKET_DATA 150      // max data bytes in a single TLV packet

// Protocol version
#define IR_PROTOCOL_VERSION 2

// Packet types
#define IR_PKT_END      0x00
#define IR_PKT_TIME     0x01
#define IR_PKT_TIMEZONE 0x02
#define IR_PKT_LOCATION 0x03
#define IR_PKT_TOTP     0x04
#define IR_PKT_TOTP_CLR 0x05

typedef enum {
    IR_TIME_CALIBRATE,
    IR_TIME_LISTENING,
    IR_TIME_PREAMBLE,
    IR_TIME_VERSION,        // reading 8-bit version byte
    IR_TIME_PKT_HEADER,     // reading type (8b) + length (8b)
    IR_TIME_PKT_DATA,       // reading data (length bytes) + checksum (1 byte)
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
    uint16_t baseline;
    uint16_t threshold;
    uint16_t noise_margin;

    // Tick tracking
    uint16_t tick_count;
    bool last_high;

    // Preamble measurement (RTC counter units, 128 Hz)
    uint32_t last_transition_counter;
    uint32_t interval_sum;
    uint8_t interval_count;
    uint32_t bit_period;

    // Bit-level reception
    uint32_t next_sample_counter;
    uint8_t bits_received;
    uint8_t current_byte;       // byte being assembled

    // Protocol v2 state
    uint8_t protocol_version;
    uint8_t pkt_type;
    uint8_t pkt_length;
    uint8_t pkt_data[IR_MAX_PACKET_DATA];
    uint8_t pkt_data_idx;
    uint8_t pkt_bytes_expected; // data + checksum bytes remaining
    uint8_t header_bytes[2];    // type and length staging
    uint8_t header_idx;

    // Results
    uint8_t packets_received;
    bool got_time;
    bool got_timezone;
    bool got_location;
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
