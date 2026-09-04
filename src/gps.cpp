#include "gps.h"

#include <TinyGPSPlus.h>

static const uint8_t kPinGpsTx = 12;  // UART1 TX -> GPS RX
static const uint8_t kPinGpsRx = 13;  // UART1 RX <- GPS TX
static const uint32_t kGpsBaud = 9600;

// A fix older than this is stale: the receiver has stopped producing
// location sentences (tunnel, antenna fault, cold start).
static const uint32_t kFixMaxAgeMs = 2000;

static TinyGPSPlus parser;
static GpsState state;

// Days-from-civil (Hinnant's algorithm), integer-only, so GPS date+time
// becomes Unix seconds without floating point or a time library.
static uint32_t to_unix(uint16_t y, uint8_t m, uint8_t d, uint8_t hh,
                        uint8_t mm, uint8_t ss)
{
    int32_t yy = (int32_t)y - (m <= 2 ? 1 : 0);
    const int32_t era = (yy >= 0 ? yy : yy - 399) / 400;
    const uint32_t yoe = (uint32_t)(yy - era * 400);
    const uint32_t doy =
        (uint32_t)((153 * ((int32_t)m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
    const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const int32_t days = era * 146097 + (int32_t)doe - 719468;
    return (uint32_t)days * 86400u + hh * 3600u + mm * 60u + (uint32_t)ss;
}

// RawDegrees -> degrees x 1e7 using integer math; going through the
// library's double-precision degrees would round the last digits.
static int32_t raw_to_e7(const RawDegrees& r)
{
    const int32_t v = (int32_t)(r.deg * 10000000UL + r.billionths / 100);
    return r.negative ? -v : v;
}

void gps_begin()
{
    // Serial2 is UART1 on the earlephilhower core. Bigger FIFO so a full
    // NMEA burst survives between polls.
    Serial2.setTX(kPinGpsTx);
    Serial2.setRX(kPinGpsRx);
    Serial2.setFIFOSize(256);
    Serial2.begin(kGpsBaud);
}

void gps_poll()
{
    while (Serial2.available() > 0) {
        parser.encode((char)Serial2.read());
    }

    if (parser.location.isUpdated()) {
        state.new_this_sample = true;
        state.lat_e7 = raw_to_e7(parser.location.rawLat());
        state.lon_e7 = raw_to_e7(parser.location.rawLng());
    }
    state.fix_valid =
        parser.location.isValid() && parser.location.age() < kFixMaxAgeMs;

    if (parser.speed.isValid()) {
        // TinyGPSPlus stores hundredths of knots; 1 knot = 51.4444 cm/s.
        const uint32_t cms = (uint32_t)(parser.speed.value() * 0.514444f);
        state.speed_cms = cms > 0xFFFF ? 0xFFFF : (uint16_t)cms;
    }
    if (parser.course.isValid()) {
        state.course_cdeg = (uint16_t)(parser.course.value() % 36000);
    }
    if (parser.satellites.isValid()) {
        const uint32_t n = parser.satellites.value();
        state.sats = n > 255 ? 255 : (uint8_t)n;
    }
    if (parser.hdop.isValid()) {
        const uint32_t t = parser.hdop.value() / 10;  // hundredths -> tenths
        state.hdop_tenths = t > 255 ? 255 : (uint8_t)t;
    }
    if (parser.date.isValid() && parser.time.isValid() &&
        parser.date.year() >= 2024) {
        state.have_utc = true;
        state.utc_seconds =
            to_unix(parser.date.year(), parser.date.month(),
                    parser.date.day(), parser.time.hour(),
                    parser.time.minute(), parser.time.second());
    }
}

GpsState gps_take()
{
    const GpsState out = state;
    state.new_this_sample = false;
    return out;
}

const GpsState& gps_peek() { return state; }
