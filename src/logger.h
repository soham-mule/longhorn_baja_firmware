#pragma once

// SD logging module (spec section 8) -- the highest risk part of the
// project. SD cards stall unpredictably for tens to hundreds of ms while
// their controller does housekeeping, so samples land in a RAM ring
// buffer and drain to the card only when it will accept data. The sample
// path never waits on the card.

#include <stdint.h>

#include "record.h"

// Mount the card, pick the next free LOGnnnn.BIN, pre-allocate it and
// write the file header (including the IMU full-scale settings the
// parser needs). Blocking; call from setup(). false = no usable card
// (the firmware halts on that -- a run that logs nothing is worse than
// a run that does not start).
bool logger_begin(uint32_t boot_id, uint8_t accel_fs_g, uint16_t gyro_fs_dps);

// Queue one record. Returns false (and counts a drop) if the ring buffer
// is full, which means the card stalled longer than the buffer covers.
bool logger_push(const sample_t& s);

// Housekeeping: drain full sectors to the card when it is not busy, sync
// on a schedule, patch the header after the first GPS fix. Call every
// loop() iteration; never blocks on a busy card.
void logger_poll();

// Record the GPS-UTC / micros() pairing of the first fix; the file
// header is patched in place at the next quiet moment.
void logger_note_first_fix(uint32_t gps_utc, uint32_t t_us);

uint32_t logger_dropped();   // samples lost to a full ring buffer
uint32_t logger_written();   // records handed to the card
bool logger_ok();            // false after an unrecoverable write error
