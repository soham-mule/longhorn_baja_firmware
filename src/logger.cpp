#include "logger.h"

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <RingBuf.h>

// SPI0 pin map from the ECU schematic (Pico.SchDoc): SCK=GP2, MOSI=GP3,
// MISO=GP4, CS=GP5. All native SPI0 functions.
static const uint8_t kPinSck = 2;
static const uint8_t kPinMosi = 3;
static const uint8_t kPinMiso = 4;
static const uint8_t kPinCs = 5;

// 32 KiB buffer = 8 s of headroom at 4 kB/s -- covers any sane stall.
// The RP2040 has 264 KiB of RAM; be generous here (spec 8.2).
static const size_t kRingBytes = 32768;

// One SD sector; writes in other sizes force read-modify-write cycles.
static const size_t kSectorBytes = 512;

// Pre-allocate contiguous space so logging never grows the file (FAT
// updates are another stall source). 64 MiB > a four-hour run at 4 kB/s.
static const uint64_t kPreallocBytes = 64ull * 1024 * 1024;

static const uint32_t kSyncPeriodUs = 4000000;  // bound loss on power cut

// SdFat over the stock SD library: faster, and reads the exFAT that
// 64 GB SDXC cards ship formatted as (which the stock library cannot).
static SdFs sd;
static FsFile file;
static RingBuf<FsFile, kRingBytes> ring;

static file_header_t header;
static bool ok = false;
static bool header_patch_pending = false;
static uint32_t dropped = 0;
static uint32_t written = 0;
static uint32_t last_sync_us = 0;

bool logger_begin(uint32_t boot_id, uint8_t accel_fs_g, uint16_t gyro_fs_dps)
{
    SPI.setSCK(kPinSck);
    SPI.setTX(kPinMosi);
    SPI.setRX(kPinMiso);

    if (!sd.begin(SdSpiConfig(kPinCs, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI))) {
        return false;
    }

    // Incrementing index, never timestamps: there is no wall clock until
    // the GPS fixes, which may be minutes after logging starts.
    char name[16];
    uint16_t idx = 0;
    do {
        idx++;
        snprintf(name, sizeof(name), "LOG%04u.BIN", idx);
    } while (sd.exists(name) && idx < 9999);
    if (sd.exists(name)) return false;  // 9999 files: card needs clearing

    if (!file.open(name, O_RDWR | O_CREAT | O_EXCL)) return false;
    if (!file.preAllocate(kPreallocBytes)) {
        // Non-fatal: logging still works, just with FAT-growth stalls.
        // Worth a message during bring-up, not worth refusing to log.
    }

    header.magic = LOG_MAGIC;
    header.version = LOG_VERSION;
    header.record_bytes = sizeof(sample_t);
    header.boot_id = boot_id;
    header.gps_utc = 0;      // patched in place at first fix
    header.t_us_at_fix = 0;
    header.gyro_fs_dps = gyro_fs_dps;
    header.accel_fs_g = accel_fs_g;
    header.reserved = 0;
    if (file.write(&header, sizeof(header)) != (int)sizeof(header)) {
        return false;
    }

    ring.begin(&file);
    last_sync_us = micros();
    ok = true;
    return true;
}

bool logger_push(const sample_t& s)
{
    if (!ok) return false;
    if (ring.bytesFree() < sizeof(s)) {
        dropped++;
        return false;
    }
    if (ring.write(&s, sizeof(s)) != sizeof(s)) {
        dropped++;
        return false;
    }
    written++;
    return true;
}

void logger_note_first_fix(uint32_t gps_utc, uint32_t t_us)
{
    if (header.gps_utc != 0) return;  // only the first fix matters
    header.gps_utc = gps_utc;
    header.t_us_at_fix = t_us;
    header_patch_pending = true;
}

void logger_poll()
{
    if (!ok) return;

    // Drain one sector per call when the card will take it. isBusy()
    // is the non-blocking guard: during a card stall we simply come back
    // next loop and the ring buffer absorbs the samples.
    if (ring.bytesUsed() >= kSectorBytes && !file.isBusy()) {
        if (ring.writeOut(kSectorBytes) != kSectorBytes) {
            ok = false;
            return;
        }
    }

    // Patch the header opportunistically: only when the ring is empty,
    // so the seek cannot interleave with buffered record bytes.
    if (header_patch_pending && ring.bytesUsed() == 0 && !file.isBusy()) {
        const uint64_t pos = file.curPosition();
        if (file.seekSet(0) &&
            file.write(&header, sizeof(header)) == (int)sizeof(header) &&
            file.seekSet(pos)) {
            header_patch_pending = false;
            file.sync();
        } else {
            ok = false;
            return;
        }
    }

    // Sync every few seconds: a power cut then costs the last few
    // seconds, not the whole file (metadata is flushed).
    const uint32_t now = micros();
    if ((int32_t)(now - last_sync_us) >= (int32_t)kSyncPeriodUs) {
        last_sync_us += kSyncPeriodUs;
        if (ring.bytesUsed() == 0 && !file.isBusy()) {
            if (!file.sync()) ok = false;
        }
    }
}

uint32_t logger_dropped() { return dropped; }
uint32_t logger_written() { return written; }
bool logger_ok() { return ok; }
