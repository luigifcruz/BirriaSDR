/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * MT7961/MT7972 matched receive bandwidth at 10, 20, 40 or 80 MSps
 *
 * At 80 MSps the profile follows the vendor's 80 MHz channel setup (also used
 * by Mtk79ICAP): analog code 2, per-chain 0xb00/0xb04 fields, digital class 2
 * latched by a pulse of 0x8300c008/0x8301c008[31], and on 2.4 GHz
 * 0x830a0080[7:4] = 6. Without them the 80 MSps tap repeats 40 MSps samples.
 * At 20 MSps it is the vendor's 20 MHz setup (codes 0, 0x83080000[12] set);
 * the capture packer then keeps one 80 MSps tap sample in four, so this filter
 * is what keeps the wider span from aliasing into the 20 MHz output.
 * The description below is the 40 MSps profile.
 *
 * The analog filter and the digital PHY bandwidth are independent controls.
 * RF 0x160[31:30] selects the analog profile in each physical SPI bank;
 * 0x83081000[27:24] and 0x83091000[27:24] select the digital bandwidth class.
 * The original channel-6 bootstrap leaves both at code 0. Merely widening the
 * analog filter retains the digital rolloff: noise is about 22 dB down at
 * +/-18 MHz with analog code 1 and digital code 0.
 *
 * Set both fields to code 1 after the established RF/gain bootstrap, while
 * capture is stopped and the original MCU program waits for GO. This is the
 * matched 40 MHz profile. The capture tap and divider remain unchanged at
 * nominal 40 MSps complex; no host equalizer, averaging or format conversion
 * is introduced. The paired configuration reduced the measured noise rolloff
 * to about 2.8 dB at +/-18 MHz on the tested board.
 *
 * This is a nominal channel profile, not a brick-wall anti-alias filter. The
 * receive-only Wi-Fi comparison measured roughly 19-20 dB suppression at
 * +/-30 MHz and still decoded some aliased headers from strong signals there.
 * The wider analog codes admit more out-of-span energy and are research-only.
 * Calibrated passband, stopband guarantees and arbitrary filter widths remain
 * unsupported; see docs/mt7921.md for the configuration and its limitations.
 *
 * Save all four baseline fields before the first write. Preserve unrelated
 * bits, verify each application, and restore every field on partial failure.
 * Analog state can survive WFSYS reset, so normal reset restores this baseline
 * too. A failed restoration keeps the saved state for retry. Every automatic
 * restart follows the same bootstrap and reapplies this profile before GO.
 */
#include "mt7921.h"
#include <string.h>

/* Per-rate values from the vendor firmware's channel setup (FUN_0093dbd4,
 * FUN_0093ca3e; tabulated in upstream issue #1). Fields 0-3 alone are the
 * 40 MHz profile. 20 and 80 MHz add the per-chain analog words 0xb00/0xb04
 * and 0x830a0080[7:4]: the 2.4 GHz channel number at 20 MHz, 6 at 80 MHz.
 * 0x83080000[12] is set only at 20 MHz. RF fields name their physical SPI
 * bank; the others are MMIO.
 */
enum { FIELD_CHANNEL = 8, FIELD_BW20 = 9, FIELD_NARROW0 = 10, FIELD_NARROW1 = 11,
       FIELD_COUNT = 12 };
static const struct filter_field {
    bool rf;
    unsigned port;
    uint32_t address, mask, value20, value40, value80;
} fields[FIELD_COUNT] = {
    {true, 0, 0x160, 0xc0000000, 0, 0x40000000, 0x80000000},
    {true, 1, 0x160, 0xc0000000, 0, 0x40000000, 0x80000000},
    {false, 0, 0x83081000, 0x0f000000, 0, 0x01000000, 0x02000000},
    {false, 0, 0x83091000, 0x0f000000, 0, 0x01000000, 0x02000000},
    {true, 0, 0xb00, 0x3f000000, 0x10000000, 0, 0x20000000},
    {true, 1, 0xb00, 0x3f000000, 0x10000000, 0, 0x20000000},
    {true, 0, 0xb04, 0x1f0000c0, 0x0f000040, 0, 0x1f000080},
    {true, 1, 0xb04, 0x1f0000c0, 0x0f000040, 0, 0x1f000080},
    {false, 0, 0x830a0080, 0x000000f0, 0, 0, 0x00000060},
    {false, 0, 0x83080000, 0x00001000, 0x00001000, 0, 0},
    {false, 0, 0x83081000, 0x70000000, 0x70000000, 0, 0},
    {false, 0, 0x83091000, 0x70000000, 0x70000000, 0, 0}
};

/* The fields one rate owns, as a bit mask over fields[]. 10 MSps is the 20 MHz
 * profile plus 0x83081000/0x83091000[30:28] = 7, one of two extra codes in the
 * vendor's bandwidth setup (FUN_0093dbd4). Measured, it narrows the digital
 * filter to about 10 MHz: energy from just outside the 10 MHz span drops
 * about 20 dB instead of folding in at full strength.
 */
static uint32_t rate_fields(const birriasdr_dev_t *dev)
{
    bool narrow = dev->rate == 10000000;
    uint32_t mask = dev->rate == 40000000 ? 0x00f : 0x0ff;
    if (dev->rate != 40000000)
        mask |= 1u << FIELD_CHANNEL;
    if (dev->rate == 20000000 || narrow)
        mask |= 1u << FIELD_BW20;
    if (narrow)
        mask |= 1u << FIELD_NARROW0 | 1u << FIELD_NARROW1;
    return mask;
}

/* The vendor writes the 2.4 GHz channel number for a 20 MHz channel; use the
 * nearest one to the requested center.
 */
static uint32_t field_value(const birriasdr_dev_t *dev, unsigned field)
{
    bool narrow = dev->rate == 10000000 || dev->rate == 20000000;
    if (narrow && field == FIELD_CHANNEL) {
        uint64_t mhz = dev->frequency / 1000000;
        uint32_t channel;
        if (mhz >= 2482)
            channel = 14;
        else if (mhz <= 2412)
            channel = 1;
        else if ((channel = (uint32_t)((mhz - 2407 + 2) / 5)) > 13)
            channel = 13;
        return channel << 4;
    }
    if (narrow)
        return fields[field].value20;
    return dev->rate == 80000000 ? fields[field].value80 : fields[field].value40;
}

static int filter_read(birriasdr_dev_t *dev, unsigned field, uint32_t *value)
{
    const struct filter_field *f = &fields[field];
    if (f->rf)
        return mt_rf_read(dev, f->port, f->address, value);
    return bs_read32(dev, BS_MMIO, f->address, value);
}

/* Change only the owned field, retaining band selection and other modes.
 * Masked verification accepts independent changes to unrelated register bits.
 */
static int filter_write(birriasdr_dev_t *dev, unsigned field, uint32_t value)
{
    const struct filter_field *f = &fields[field];
    uint32_t actual;
    if (f->rf)
        BS_TRY(mt_rf_update(dev, f->port, f->address, f->mask, value & f->mask));
    else
        BS_TRY(bs_update32(dev, BS_MMIO, f->address, f->mask, value & f->mask));
    BS_TRY(filter_read(dev, field, &actual));
    return ((actual ^ value) & f->mask) ? BIRRIASDR_ERROR_STATE : 0;
}

/* The digital rate class takes effect on a pulse of 0x8300c008/0x8301c008[31],
 * as the vendor firmware and Mtk79ICAP do. The 40 MHz profile never needed it.
 */
static int latch_rate_class(birriasdr_dev_t *dev)
{
    BS_TRY(bs_update32(dev, BS_MMIO, 0x8300c008, 0, 0x80000000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x8301c008, 0, 0x80000000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x8300c008, 0x80000000, 0));
    return bs_update32(dev, BS_MMIO, 0x8301c008, 0x80000000, 0);
}

/* Restore digital fields first, then analog width, before the subsystem reset.
 * Attempt every field even if one fails; retain the baseline until all pass.
 */
int mt_restore_filter(birriasdr_dev_t *dev)
{
    static const unsigned order[FIELD_COUNT] = {FIELD_NARROW0, FIELD_NARROW1, 2, 3, FIELD_BW20,
                                                FIELD_CHANNEL, 0, 1, 4, 5, 6, 7};
    uint32_t mask = dev->mt7921.filter_fields;
    int first_error = 0;
    if (!dev->mt7921.filter_saved)
        return 0;
    for (unsigned i = 0; i < FIELD_COUNT; ++i) {
        unsigned field = order[i];
        int result;
        if (!(mask & (1u << field)))
            continue;
        result = filter_write(dev, field, dev->mt7921.filter_baseline[field]);
        if (result < 0 && !first_error)
            first_error = result;
    }
    if (mask != 0x00f) {
        int result = latch_rate_class(dev);
        if (result < 0 && !first_error)
            first_error = result;
    }
    if (!first_error)
        dev->mt7921.filter_saved = false;
    return first_error;
}

/* Apply the fixed profile once per preparation; this is not a live control.
 * The prepare owner handles any remaining cleanup and whole-startup retry.
 * The 40 MHz profile keeps its original four writes and no latch pulse.
 */
int mt_configure_filter(birriasdr_dev_t *dev)
{
    uint32_t before[FIELD_COUNT], mask;
    if (!mt_rate_supported(dev->rate))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    if (dev->mt7921.filter_saved)
        return BIRRIASDR_ERROR_STATE;
    mask = rate_fields(dev);
    for (unsigned i = 0; i < FIELD_COUNT; ++i)
        if (mask & (1u << i))
            BS_TRY(filter_read(dev, i, &before[i]));
    memcpy(dev->mt7921.filter_baseline, before, sizeof(before));
    dev->mt7921.filter_fields = mask;
    dev->mt7921.filter_saved = true;
    for (unsigned i = 0; i < FIELD_COUNT; ++i) {
        int result;
        if (!(mask & (1u << i)))
            continue;
        result = filter_write(dev, i, field_value(dev, i));
        if (result < 0)
            return mt_restore_filter(dev) < 0 ? BIRRIASDR_ERROR_STATE : result;
    }
    if (mask != 0x00f) {
        int result = latch_rate_class(dev);
        if (result < 0)
            return mt_restore_filter(dev) < 0 ? BIRRIASDR_ERROR_STATE : result;
    }
    bs_sleep_us(20000);
    return 0;
}
