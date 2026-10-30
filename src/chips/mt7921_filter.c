/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * MT7961/MT7972 matched receive bandwidth at 10, 20, 40 or 80 MSps
 *
 * At 80 MSps the profile follows the vendor firmware's 80 MHz channel setup:
 * analog code 2, per-chain 0xb00/0xb04 fields, digital class 2
 * latched by a pulse of 0x8300c008/0x8301c008[31], and on 2.4 GHz
 * 0x830a0080[7:4] = 6. Without them the 80 MSps tap repeats 40 MSps samples.
 * At 20 MSps it is the vendor's 20 MHz setup (codes 0, 0x83080000[12] set);
 * the capture packer then keeps one 80 MSps tap sample in four, so this filter
 * is what keeps the wider span from aliasing into the 20 MHz output.
 * At 10 MSps it is the 20 MHz setup plus a narrower digital filter.
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
 * Each rate uses only the analog code that matches its span. Calibrated
 * passband, stopband guarantees and arbitrary filter widths remain
 * unsupported; see docs/mt7921.md for the configuration and its limitations.
 *
 * Save every field the rate owns before the first write. Preserve unrelated
 * bits, verify each application, and restore every field on partial failure.
 * Analog state can survive WFSYS reset, so normal reset restores this baseline
 * too. A failed restoration keeps the saved state for retry. Every automatic
 * restart follows the same bootstrap and reapplies this profile before GO.
 */
#include "mt7921.h"
#include <string.h>

/* Per-rate values from the vendor firmware's channel setup, tabulated in this
 * project's issue #1. Fields 0-3 alone are the
 * 40 MHz profile. 20 and 80 MHz add the per-chain analog words 0xb00/0xb04
 * and 0x830a0080[7:4]: the 2.4 GHz channel number at 20 MHz, 6 at 80 MHz.
 * 0x83080000[12] is set only at 20 MHz. RF fields name their physical SPI
 * bank; the others are MMIO.
 */
enum {
    FIELD_ANALOG0,      /* RF port 0 0x160[31:30], analog profile code */
    FIELD_ANALOG1,      /* RF port 1 0x160[31:30] */
    FIELD_DIGITAL0,     /* 0x83081000[27:24], digital bandwidth class */
    FIELD_DIGITAL1,     /* 0x83091000[27:24] */
    FIELD_CHAIN_WIDTH0, /* RF port 0 0xb00[29:24], per-chain analog word */
    FIELD_CHAIN_WIDTH1, /* RF port 1 0xb00[29:24] */
    FIELD_CHAIN_SHAPE0, /* RF port 0 0xb04[28:24] and [7:6] */
    FIELD_CHAIN_SHAPE1, /* RF port 1 0xb04[28:24] and [7:6] */
    FIELD_CHANNEL,      /* 0x830a0080[7:4], channel number, not decoded */
    FIELD_MODE20,       /* 0x83080000[12], set by the vendor at 20 MHz */
    FIELD_NARROW0,      /* 0x83081000[30:28], narrow digital filter */
    FIELD_NARROW1,      /* 0x83091000[30:28] */
    FIELD_COUNT
};
_Static_assert(FIELD_COUNT == sizeof(((birriasdr_dev_t *)0)->mt7921.filter_baseline) /
                                  sizeof(uint32_t),
               "the saved baseline must hold every filter field");

static const struct filter_field {
    bool rf;
    unsigned port;
    uint32_t address, mask, value10, value20, value40, value80;
} fields[FIELD_COUNT] = {
    [FIELD_ANALOG0] = {true, 0, 0x160, 0xc0000000, 0, 0, 0x40000000, 0x80000000},
    [FIELD_ANALOG1] = {true, 1, 0x160, 0xc0000000, 0, 0, 0x40000000, 0x80000000},
    [FIELD_DIGITAL0] = {false, 0, 0x83081000, 0x0f000000, 0, 0, 0x01000000, 0x02000000},
    [FIELD_DIGITAL1] = {false, 0, 0x83091000, 0x0f000000, 0, 0, 0x01000000, 0x02000000},
    [FIELD_CHAIN_WIDTH0] = {true, 0, 0xb00, 0x3f000000, 0x10000000, 0x10000000, 0, 0x20000000},
    [FIELD_CHAIN_WIDTH1] = {true, 1, 0xb00, 0x3f000000, 0x10000000, 0x10000000, 0, 0x20000000},
    [FIELD_CHAIN_SHAPE0] = {true, 0, 0xb04, 0x1f0000c0, 0x0f000040, 0x0f000040, 0, 0x1f000080},
    [FIELD_CHAIN_SHAPE1] = {true, 1, 0xb04, 0x1f0000c0, 0x0f000040, 0x0f000040, 0, 0x1f000080},
    [FIELD_CHANNEL] = {false, 0, 0x830a0080, 0x000000f0, 0, 0, 0, 0x00000060},
    [FIELD_MODE20] = {false, 0, 0x83080000, 0x00001000, 0x00001000, 0x00001000, 0, 0},
    [FIELD_NARROW0] = {false, 0, 0x83081000, 0x70000000, 0x70000000, 0, 0, 0},
    [FIELD_NARROW1] = {false, 0, 0x83091000, 0x70000000, 0x70000000, 0, 0, 0}
};

#define FIELD_BIT(field) (1u << (field))
/* The original matched 40 MHz profile: analog code and digital class only. */
static const uint32_t profile40_fields = FIELD_BIT(FIELD_ANALOG0) | FIELD_BIT(FIELD_ANALOG1) |
                                         FIELD_BIT(FIELD_DIGITAL0) | FIELD_BIT(FIELD_DIGITAL1);

/* The fields one rate owns, as a bit mask over fields[]. The 40 MSps profile
 * keeps its original four fields. The other rates own all twelve, writing zero
 * to the ones they do not use, so a field left set by an earlier run cannot
 * change their profile. 10 MSps is the 20 MHz profile plus
 * 0x83081000/0x83091000[30:28] = 7, one of two extra codes in the vendor
 * firmware's bandwidth setup. Measured, it narrows the digital filter to about
 * 10 MHz: energy from just outside the 10 MHz span drops about 20 dB instead of
 * folding in at full strength.
 */
static uint32_t rate_fields(const birriasdr_dev_t *dev)
{
    if (dev->rate == 40000000)
        return profile40_fields;
    return (1u << FIELD_COUNT) - 1u;
}

/* The 2.4 GHz channel whose center is nearest, compared in Hz. Channels 1 to 13
 * are 5 MHz apart from 2412 MHz, and channel 14 is at 2484 MHz. A tie selects
 * the lower channel.
 */
static uint32_t nearest_channel(uint64_t frequency)
{
    uint32_t best = 1;
    uint64_t best_distance = UINT64_MAX;
    for (uint32_t channel = 1; channel <= 14; ++channel) {
        uint64_t center = channel == 14 ? UINT64_C(2484000000)
                                        : UINT64_C(2407000000) + UINT64_C(5000000) * channel;
        uint64_t distance = frequency > center ? frequency - center : center - frequency;
        if (distance < best_distance) {
            best = channel;
            best_distance = distance;
        }
    }
    return best;
}

/* The value a rate writes to one field. At 10 and 20 MSps the channel field
 * gets the 2.4 GHz channel number nearest the requested center, as the vendor
 * writes for a 20 MHz channel. The field is set once per startup and not
 * changed by live retunes, because its effect is not decoded.
 */
static uint32_t field_value(const birriasdr_dev_t *dev, unsigned field)
{
    const struct filter_field *f = &fields[field];
    if (mt_rate_packed(dev->rate) && field == FIELD_CHANNEL)
        return nearest_channel(dev->frequency) << 4;
    switch (dev->rate) {
    case 10000000:
        return f->value10;
    case 20000000:
        return f->value20;
    case 80000000:
        return f->value80;
    default:
        return f->value40;
    }
}

/* Read the whole register that holds one field. */
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
 * as the vendor firmware does. The 40 MHz profile never needed it. Clear both
 * bits first, so the pulse starts low even if a bit was left set, then set and
 * clear them, and check that both read back low. The high state is not read
 * back, because the hardware may clear the bit itself.
 */
static int latch_rate_class(birriasdr_dev_t *dev)
{
    static const uint32_t latches[] = {0x8300c008, 0x8301c008};
    uint32_t value;
    for (unsigned i = 0; i < 2; ++i)
        BS_TRY(bs_update32(dev, BS_MMIO, latches[i], 0x80000000, 0));
    for (unsigned i = 0; i < 2; ++i)
        BS_TRY(bs_update32(dev, BS_MMIO, latches[i], 0, 0x80000000));
    for (unsigned i = 0; i < 2; ++i)
        BS_TRY(bs_update32(dev, BS_MMIO, latches[i], 0x80000000, 0));
    for (unsigned i = 0; i < 2; ++i) {
        BS_TRY(bs_read32(dev, BS_MMIO, latches[i], &value));
        if (value & 0x80000000)
            return BIRRIASDR_ERROR_STATE;
    }
    return 0;
}

/* Restore digital fields first, then analog width, before the subsystem reset.
 * Attempt every field even if one fails; retain the baseline until all pass.
 */
int mt_restore_filter(birriasdr_dev_t *dev)
{
    /* Digital fields first, then analog width. */
    static const unsigned order[FIELD_COUNT] = {
        FIELD_NARROW0,      FIELD_NARROW1,      FIELD_DIGITAL0,     FIELD_DIGITAL1,
        FIELD_MODE20,       FIELD_CHANNEL,      FIELD_ANALOG0,      FIELD_ANALOG1,
        FIELD_CHAIN_WIDTH0, FIELD_CHAIN_WIDTH1, FIELD_CHAIN_SHAPE0, FIELD_CHAIN_SHAPE1};
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
    if (mask != profile40_fields) {
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
    if (mask != profile40_fields) {
        int result = latch_rate_class(dev);
        if (result < 0)
            return mt_restore_filter(dev) < 0 ? BIRRIASDR_ERROR_STATE : result;
    }
    bs_sleep_us(20000);
    return 0;
}
