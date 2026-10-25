/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * MT7961/MT7972 matched receive bandwidth at 40 or 80 MSps
 *
 * At 80 MSps the profile follows the vendor's 80 MHz channel setup (also used
 * by Mtk79ICAP): analog code 2, per-chain 0xb00/0xb04 fields, digital class 2
 * latched by a pulse of 0x8300c008/0x8301c008[31], and on 2.4 GHz
 * 0x830a0080[7:4] = 6. Without them the 80 MSps tap repeats 40 MSps samples.
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

/* Fields 0-3 are the 40 MHz profile. The 80 MHz profile adds the per-chain
 * analog words 0xb00/0xb04 and 0x830a0080[7:4] (vendor FUN_0093ca3e writes 6
 * there for an 80 MHz 2.4 GHz channel). RF fields name their physical SPI
 * bank; the others are MMIO.
 */
static const struct filter_field {
    bool rf;
    unsigned port;
    uint32_t address, mask, value40, value80;
} fields[] = {
    {true, 0, 0x160, 0xc0000000, 0x40000000, 0x80000000},
    {true, 1, 0x160, 0xc0000000, 0x40000000, 0x80000000},
    {false, 0, 0x83081000, 0x0f000000, 0x01000000, 0x02000000},
    {false, 0, 0x83091000, 0x0f000000, 0x01000000, 0x02000000},
    {true, 0, 0xb00, 0x3f000000, 0, 0x20000000},
    {true, 1, 0xb00, 0x3f000000, 0, 0x20000000},
    {true, 0, 0xb04, 0x1f0000c0, 0, 0x1f000080},
    {true, 1, 0xb04, 0x1f0000c0, 0, 0x1f000080},
    {false, 0, 0x830a0080, 0x000000f0, 0, 0x00000060}
};

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

/* Restore digital class first, then analog width, before the subsystem reset.
 * Attempt every field even if one fails; retain the baseline until all pass.
 */
int mt_restore_filter(birriasdr_dev_t *dev)
{
    static const unsigned order[] = {2, 3, 8, 0, 1, 4, 5, 6, 7};
    unsigned count = dev->mt7921.filter_fields;
    int first_error = 0;
    if (!dev->mt7921.filter_saved)
        return 0;
    for (unsigned i = 0; i < sizeof(order) / sizeof(order[0]); ++i) {
        unsigned field = order[i];
        int result;
        if (field >= count)
            continue;
        result = filter_write(dev, field, dev->mt7921.filter_baseline[field]);
        if (result < 0 && !first_error)
            first_error = result;
    }
    if (count > 4) {
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
 */
int mt_configure_filter(birriasdr_dev_t *dev)
{
    uint32_t before[9];
    bool wide = dev->rate == 80000000;
    unsigned count = wide ? 9 : 4;
    if (dev->rate != 40000000 && !wide)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    if (dev->mt7921.filter_saved)
        return BIRRIASDR_ERROR_STATE;
    for (unsigned i = 0; i < count; ++i)
        BS_TRY(filter_read(dev, i, &before[i]));
    memcpy(dev->mt7921.filter_baseline, before, count * sizeof(before[0]));
    dev->mt7921.filter_fields = count;
    dev->mt7921.filter_saved = true;
    for (unsigned i = 0; i < count; ++i) {
        int result = filter_write(dev, i, wide ? fields[i].value80 : fields[i].value40);
        if (result < 0)
            return mt_restore_filter(dev) < 0 ? BIRRIASDR_ERROR_STATE : result;
    }
    if (wide) {
        int result = latch_rate_class(dev);
        if (result < 0)
            return mt_restore_filter(dev) < 0 ? BIRRIASDR_ERROR_STATE : result;
    }
    bs_sleep_us(20000);
    return 0;
}
