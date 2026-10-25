/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * MT7961 ICAP setup and capture/firmware lifecycle
 *
 * The receive data path is:
 *
 *   RF/PHY chain 0 -> ICAP tap 4 -> circular SRAM -> MCU DMA -> USB events
 *
 * This module selects the capture tap and clocks; mt7921_rf.c makes that tap
 * receive an external signal. Tap 4 is selected in 0x830ad448[11:8] and uses
 * logical chain 0 (physical RF SPI 1). This is not an antenna-switch control.
 * The backend uses the channel-6 anchor at 40 MSps complex and exposes an
 * low-band retuning interval without changing the capture clocks.
 *
 * ICAP control is 0x80021090, its start/end offsets are 0x80021098/0x8002109c,
 * and its producer pointer is 0x800210b4. The 256 KiB ring at byte address
 * 0xe0200000 holds 65,536 complex samples: only 1.6384 ms at 40 MSps. Samples
 * occupy four bytes each, little-endian Q then I in 16-bit slots with 14 valid
 * bits per component. Four consecutive complex samples fill 16 bytes without
 * inter-sample padding. Address units must not be confused with byte offsets.
 *
 * mt_configure_capture() leaves ICAP stopped (control 0x4f1). The original
 * MCU program waits for GO, starts ICAP with 0x24f3, watches the producer and
 * submits 8,192-byte payloads directly from SRAM: no MCU sample-copy loop.
 * Descriptors are serial because earlier multiple-outstanding SRAM transfers
 * corrupted data. Host USB reads may still be queued and aggregated; that is
 * separate from the device descriptor lifetime.
 *
 * Mailbox offsets below are bytes relative to MT_MAILBOX (0x02038360):
 *   +28 state; +32 packet limit (zero means until STOP); +36 STOP;
 *   +40 submitted packets; +44 completed packets; +48 ticks; +52 DMA errors;
 *   +60 GO. The first three words carry boot magic, nonce and nonce response.
 *
 * Queue host reads BEFORE GO so setup does not consume the short SRAM window.
 * STOP asks the MCU to stop capture, drain DMA and park. It publishes terminal
 * state last, after counters/errors; mt_status() reads state first so terminal
 * values make the remaining words stable. State 1 is ready/running, 2 finite
 * completion and 3 host stop. Fault states include DMA timeout, ambiguous
 * producer wrap and source overrun; src/stream.c must propagate those failures
 * even when a valid prefix has already reached application callbacks.
 *
 * Ring size, producer units, payload size and mailbox layout form a contract
 * with firmware/iq_stream.S, mt7921_frame.c and src/stream.c. Change and test
 * them together; a larger host read buffer alone cannot enlarge hardware SRAM
 * or fix producer wrap. See docs/mt7921.md for the transport overview.
 */
#include "mt7921.h"

/* ICAP writes consecutive raw14 Q,I pairs into a 256 KiB circular SRAM.
 * The original MCU program later watches its producer pointer and submits
 * serial DMA descriptors referencing that SRAM directly. This host setup
 * selects receive-only MAC clocks, the chain-0 tap and the dense 40 MSps mode;
 * it leaves capture stopped until bulk-IN requests have been queued.
 */
int mt_configure_capture(birriasdr_dev_t *dev)
{
    uint32_t value;
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 76, &value));
    if (value)
        return BIRRIASDR_ERROR_STATE;
    BS_TRY(mt_rf_read(dev, MT_TOP_PORT, 0x2c, &value));
    if (value != 0x79728a10)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    BS_TRY(bs_update32(dev, BS_MMIO, 0x820e3000, 0x200, 0x100));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x83080004, 0, 0x3840));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x83080008, 0, 0x3840));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x830a3008, 3u << 21, 0));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x830a1000, 0x04090100));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x830a1004, 0xff, 0x10));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x830a1008, 0));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x80021090, 0xf1));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x80021098, 0));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x8002109c, 0xfffc));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x800210b8, 0xf0000, 0x9c000000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x830ad440, 0xff, 0x80000013));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x830ad448, 0x0fffffff, 0x008c0400));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x88009004, 0, 0x0f000000));
    return bs_write32(dev, BS_MMIO, 0x80021090, 0x4f1);
}

/* Release the custom program's GO gate only after the host has queued USB reads.
 * Starting ICAP earlier would consume the ~1.6 ms SRAM capacity during setup.
 */
int mt_start(birriasdr_dev_t *dev)
{
    return bs_write32(dev, BS_MMIO, MT_MAILBOX + 60, 1);
}

/* The stop mailbox asks the MCU to stop producing, drain DMA, then park. */
int mt_stop(birriasdr_dev_t *dev)
{
    return bs_write32(dev, BS_MMIO, MT_MAILBOX + 36, 1);
}

/* Firmware publishes terminal state LAST, after packet counts and DMA errors.
 * Reading state first is safe: a terminal value makes the remaining words stable.
 */
int mt_status(birriasdr_dev_t *dev, uint32_t words[16])
{
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 28, &words[7]));
    if (words[7] == 1)
        return 0;
    for (unsigned i = 0; i < 16; ++i)
        if (i != 7)
            BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + i * 4, &words[i]));
    return 0;
}

/* Keep validated ranges in the chipset backend; adding another chipset must
 * not require teaching the public API about its register or frequency limits.
 */
static int validate(uint64_t frequency, uint32_t rate, const struct birriasdr_gain *gain)
{
    if (gain->lna > 3 || gain->lpf > 8 || gain->digital > 12)
        return BIRRIASDR_ERROR_INVALID;
    if (mt_band_of(frequency) == MT_BAND_NONE || rate != 40000000)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    return 0;
}

/* The band profile is applied once per preparation, so a live change must stay
 * in the prepared band. Rejecting here leaves the running capture untouched.
 */
static int validate_change(const birriasdr_dev_t *dev, uint64_t frequency)
{
    return mt_band_of(frequency) == mt_band_of(dev->frequency) ? 0 : BIRRIASDR_ERROR_UNSUPPORTED;
}

/* USB IDs identify a product, not necessarily its die revision. Read the digital
 * identity before any boot/reset write; the analog die is checked after power-on.
 */
static int identify(birriasdr_dev_t *dev)
{
    uint32_t chip;
    BS_TRY(bs_read32(dev, BS_MMIO, 0x70010200, &chip));
    return (chip & 0xffff) == 0x7961 ? 0 : BIRRIASDR_ERROR_UNSUPPORTED;
}

/* The command intervals of mt_band_of(), in its band order. */
static const struct bs_band bands[] = {
    {MT_RF_MIN_HZ, MT_RF_MAX_HZ},
    {MT_RF_5G_MIN_HZ, MT_RF_6G_MIN_HZ - 1},
    {MT_RF_6G_MIN_HZ, MT_RF_6G_MAX_HZ}
};

const struct bs_backend bs_mt7921_backend = {
    .chipset = BIRRIASDR_CHIP_MT7921,
    .name = "MT7961/MT7972",
    .capabilities = BIRRIASDR_CAP_RX | BIRRIASDR_CAP_RAW_GAIN | BIRRIASDR_CAP_LIVE_GAIN |
                    BIRRIASDR_CAP_TUNING | BIRRIASDR_CAP_LIVE_TUNING,
    .default_frequency = MT_RF_ANCHOR_HZ,
    .bands = bands, .band_count = sizeof(bands) / sizeof(bands[0]),
    .default_rate = 40000000,
    .validate = validate,
    .identify = identify,
    .prepare = mt_prepare, .start = mt_start, .stop = mt_stop,
    .status = mt_status, .reset = mt_reset, .change_gain = mt_change_gain,
    .change_frequency = mt_change_frequency,
    .validate_change = validate_change
};

_Static_assert(sizeof(bands) / sizeof(bands[0]) <= BS_MAX_BANDS, "too many MT7921 bands");
