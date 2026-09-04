#pragma once

// GPS module (spec section 7): UART1, TX on GP12, RX on GP13, NMEA via
// TinyGPSPlus. The firmware never reconciles the 1-10 Hz GPS rate with
// the 100 Hz IMU rate -- it caches the latest values, flags the sample
// where a fresh fix landed, and leaves fusion to offline analysis.

#include <Arduino.h>
#include <stdint.h>

struct GpsState {
    bool fix_valid = false;       // location valid and recent
    bool new_this_sample = false; // fresh fix since the last consume
    int32_t lat_e7 = 0;           // degrees x 1e7
    int32_t lon_e7 = 0;
    uint16_t speed_cms = 0;
    uint16_t course_cdeg = 0;
    uint8_t sats = 0;
    uint8_t hdop_tenths = 0;
    bool have_utc = false;
    uint32_t utc_seconds = 0;     // Unix time of the latest fix
};

void gps_begin();

// Drain waiting UART bytes into the parser and refresh the cached state.
// Call every loop() iteration -- it never blocks.
void gps_poll();

// Latest cached state; clears the new_this_sample flag on read so each
// fresh fix is attributed to exactly one log record. Only the sample
// path may call this.
GpsState gps_take();

// Read-only view for status output; never clears the flag.
const GpsState& gps_peek();
