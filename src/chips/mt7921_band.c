/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 BirriaSDR contributors
 */

/*
 * MT7972 5 GHz and 6 GHz receive band profile
 *
 * The high bands reuse the channel-6 bring-up unchanged and then switch the
 * analog and PHY band fields listed here. The values come from register
 * snapshots of the stock WM firmware on the EDUP AX3000M (0e8d:7961, A-die
 * 0x79728a10) tuned to 24 centers from 2412 to 7115 MHz, read over the RF
 * bridge while the firmware was idle in monitor mode. A snapshot records final
 * state, not the firmware's write order, so the order here is our own.
 *
 * Only fields that change between the stock 2.4 GHz and high-band snapshots
 * are touched. Where a changed nibble has more than one changed bit, the whole
 * nibble is copied from the stock high-band value; single changed bits are
 * treated as flags. BirriaSDR's own 2.4 GHz values differ from the stock ones
 * in many unrelated fields, and a bit-wise merge could otherwise produce a
 * field value neither profile ever used. Trim-like bytes (0x5f0[7:0],
 * 0xbf8[7:0]) are copied whole per physical bank.
 *
 * Left out on purpose: the per-gain tables at 0xa80-0xac0 and 0xe90-0xecc,
 * the firmware's calibration results and the PHY AGC tables. The 2.4 GHz
 * receiver works with all of them at reset defaults. Their effect on high-band
 * sensitivity is unmeasured.
 *
 * Every owned field is saved before the first write and restored by
 * mt_reset(): analog state survives WFSYS reset, and the 2.4 GHz bring-up does
 * not write most of these fields, so a later 2.4 GHz acquisition would
 * otherwise inherit the high-band front end. A process that dies before reset
 * leaves the high-band fields set until the dongle loses power.
 *
 * The per-frequency fields (0x0bc[31:24], the PHY channel group) step at
 * sub-band boundaries placed midway between measured centers; the exact
 * firmware boundaries are unknown. The channel-group word in 0x830ad40c is an
 * exact function of the group index for all 19 observed words.
 */
#include "mt7921.h"

#define MT_MMIO_SPACE 0xffu

/* One owned field. set_5g/set_6g are the static high-band values; fields
 * written per frequency (static == false) are only saved and restored here.
 */
struct band_field {
    uint8_t port;
    bool fixed;
    uint32_t address, mask, set_5g, set_6g;
};

static const struct band_field band_fields[] = {
    /* Band select and front-end switches, both receive banks. */
    {0, true, 0x160, 0x20000000, 0x20000000, 0x20000000},
    {1, true, 0x160, 0x20000000, 0x20000000, 0x20000000},
    {0, true, 0x5ec, 0x8000f000, 0x8000b000, 0x8000b000},
    {1, true, 0x5ec, 0x8000f000, 0x8000b000, 0x8000b000},
    {0, true, 0x5f0, 0x00ff00ff, 0x0081001b, 0x0081001b},
    {1, true, 0x5f0, 0x00ff00ff, 0x0081001b, 0x0081001b},
    {0, true, 0x5fc, 0xf0100000, 0x40100000, 0x40100000},
    {1, true, 0x5fc, 0xf0100000, 0x40100000, 0x40100000},
    {0, true, 0x940, 0x0f000000, 0, 0},
    {1, true, 0x940, 0x0f000000, 0, 0},
    {0, true, 0xbbc, 0xf0020010, 0x10000000, 0x10000000},
    {1, true, 0xbbc, 0xf0020010, 0x10000000, 0x10000000},
    {1, true, 0xbc0, 0x08000200, 0, 0},
    {0, true, 0xbc8, 0x00200000, 0, 0},
    {1, true, 0xbc8, 0x00200000, 0, 0},
    {0, true, 0xbd8, 0x00020000, 0, 0},
    {1, true, 0xbd8, 0x00020000, 0, 0},
    {0, true, 0xbf8, 0x000000ff, 0x000000a3, 0x000000a3},
    {1, true, 0xbf8, 0x000000ff, 0x0000009f, 0x0000009f},
    {0, true, 0xfa8, 0x000f0000, 0x00040000, 0x00040000},
    {1, true, 0xfa8, 0x000f0000, 0x00040000, 0x00040000},
    {0, true, 0xfc4, 0xf0400000, 0x70400000, 0x70400000},
    {1, true, 0xfc4, 0xf0400000, 0x70400000, 0x70400000},
    /* 6 GHz only. */
    {0, true, 0x0e4, 0x0000f000, 0x00007000, 0x00001000},
    {0, true, 0x0ec, 0x10000000, 0x10000000, 0},
    {1, true, 0x0ec, 0x10000000, 0x10000000, 0},
    /* PHY band fields. 0x83081000[27:24] belongs to mt7921_filter.c. */
    {MT_MMIO_SPACE, true, 0x830000d4, 0x01f00000, 0x00c00000, 0x00c00000},
    {MT_MMIO_SPACE, true, 0x8300016c, 0x00000200, 0x00000200, 0x00000200},
    {MT_MMIO_SPACE, true, 0x83081000, 0x10000000, 0x10000000, 0x10000000},
    {MT_MMIO_SPACE, true, 0x830ad464, 0x00010000, 0x00010000, 0x00010000},
    /* Per-frequency fields: mt_band_frequency() and mt_tune_high(). */
    {0, false, 0x0bc, 0xff000000, 0, 0},
    {1, false, 0x0bc, 0xff000000, 0, 0},
    {1, false, 0x184, 0xffffffff, 0, 0},
    {MT_MMIO_SPACE, false, 0x830ad40c, 0xffffffff, 0, 0},
    {0, false, 0xc00, 0x00100010, 0, 0},
    {0, false, 0xc94, 0x000000ff, 0, 0},
    {0, false, 0xcb0, 0xffffffff, 0, 0},
    {0, false, 0xcb8, 0x000ffff0, 0, 0},
    {0, false, 0xcc8, 0x000000fe, 0, 0},
    {0, false, 0xd6c, 0x00ff00ff, 0, 0}
};

#define BAND_FIELDS (sizeof(band_fields) / sizeof(band_fields[0]))
_Static_assert(BAND_FIELDS <= sizeof(((birriasdr_dev_t *)0)->mt7921.band_baseline) /
               sizeof(uint32_t), "band baseline too small");

/* The public command intervals. The 2.4 GHz interval keeps its own limits. */
enum mt_band mt_band_of(uint64_t frequency_hz)
{
    if (frequency_hz >= MT_RF_MIN_HZ && frequency_hz <= MT_RF_MAX_HZ)
        return MT_BAND_2G;
    if (frequency_hz >= MT_RF_5G_MIN_HZ && frequency_hz < MT_RF_6G_MIN_HZ)
        return MT_BAND_5G;
    if (frequency_hz >= MT_RF_6G_MIN_HZ && frequency_hz <= MT_RF_6G_MAX_HZ)
        return MT_BAND_6G;
    return MT_BAND_NONE;
}

static int field_read(birriasdr_dev_t *dev, const struct band_field *field, uint32_t *value)
{
    if (field->port == MT_MMIO_SPACE)
        return bs_read32(dev, BS_MMIO, field->address, value);
    return mt_rf_read(dev, field->port, field->address, value);
}

static int field_update(birriasdr_dev_t *dev, const struct band_field *field, uint32_t value)
{
    if (field->port == MT_MMIO_SPACE)
        return bs_update32(dev, BS_MMIO, field->address, field->mask, value & field->mask);
    return mt_rf_update(dev, field->port, field->address, field->mask, value & field->mask);
}

/* Restore every saved field, attempting all of them even after a failure.
 * The baseline is kept for a retry until the whole restoration succeeds.
 */
int mt_restore_band(birriasdr_dev_t *dev)
{
    int first_error = 0;
    if (!dev->mt7921.band_saved)
        return 0;
    for (size_t i = BAND_FIELDS; i-- > 0;) {
        int result = field_update(dev, &band_fields[i], dev->mt7921.band_baseline[i]);
        if (result < 0 && !first_error)
            first_error = result;
    }
    if (!first_error)
        dev->mt7921.band_saved = false;
    return first_error;
}

/* Save every owned field, then apply the static profile of the requested band.
 * Called once per preparation after the channel-6 bring-up and before tuning.
 */
int mt_configure_band(birriasdr_dev_t *dev)
{
    enum mt_band band = mt_band_of(dev->frequency);
    if (band != MT_BAND_5G && band != MT_BAND_6G)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    if (dev->mt7921.band_saved)
        return BIRRIASDR_ERROR_STATE;
    for (size_t i = 0; i < BAND_FIELDS; ++i)
        BS_TRY(field_read(dev, &band_fields[i], &dev->mt7921.band_baseline[i]));
    dev->mt7921.band_saved = true;
    for (size_t i = 0; i < BAND_FIELDS; ++i) {
        const struct band_field *field = &band_fields[i];
        int result;
        if (!field->fixed)
            continue;
        result = field_update(dev, field, band == MT_BAND_6G ? field->set_6g : field->set_5g);
        if (result < 0)
            return mt_restore_band(dev) < 0 ? BIRRIASDR_ERROR_STATE : result;
    }
    return 0;
}

/* Sub-band steps, each value applying below the given MHz bound. */
struct subband { uint32_t below_mhz; uint8_t value; };

static uint8_t subband_value(const struct subband *table, size_t count, uint64_t frequency_hz)
{
    for (size_t i = 0; i + 1 < count; ++i)
        if (frequency_hz < (uint64_t)table[i].below_mhz * 1000000u)
            return table[i].value;
    return table[count - 1].value;
}

uint8_t mt_band_step(enum mt_band_table which, uint64_t frequency_hz)
{
    /* Port 0 0x0bc[31:24]: measured 57 at 5180-5320, 56 at 5500-5885, 55 at 6 GHz. */
    static const struct subband frontend[] = {{5410, 0x57}, {5925, 0x56}, {0, 0x55}};
    /* PHY channel group: measured at 2..8 over 5 GHz and 9..18 over 6 GHz. */
    static const struct subband group[] = {
        {5250, 2}, {5350, 3}, {5490, 4}, {5550, 5}, {5650, 6}, {5725, 7}, {5925, 8},
        {6075, 9}, {6195, 10}, {6335, 11}, {6415, 12}, {6495, 13}, {6615, 14},
        {6775, 15}, {6895, 16}, {6985, 17}, {0, 18}
    };
    /* Synthesizer presets: 0xc94[7:0], 0xcb0[7:0] (6 GHz) and 0xd6c[23:16]. */
    static const struct subband preset[] = {{6195, 0x15}, {0, 0x16}};
    static const struct subband loop[] = {{6195, 4}, {6365, 5}, {6615, 6}, {6895, 7}, {0, 8}};
    static const struct subband pump[] = {{6365, 0xff}, {6615, 0xee}, {6895, 0xdd}, {0, 0xcc}};
    switch (which) {
    case MT_STEP_FRONTEND: return subband_value(frontend, sizeof(frontend) / sizeof(frontend[0]), frequency_hz);
    case MT_STEP_GROUP: return subband_value(group, sizeof(group) / sizeof(group[0]), frequency_hz);
    case MT_STEP_PRESET: return subband_value(preset, sizeof(preset) / sizeof(preset[0]), frequency_hz);
    case MT_STEP_LOOP: return subband_value(loop, sizeof(loop) / sizeof(loop[0]), frequency_hz);
    case MT_STEP_PUMP: return subband_value(pump, sizeof(pump) / sizeof(pump[0]), frequency_hz);
    }
    return 0;
}

static uint32_t swap_nibbles(uint32_t byte)
{
    return ((byte & 0xf) << 4) | ((byte >> 4) & 0xf);
}

/* Per-frequency fields outside the synthesizer, after a successful high-band
 * tune. The group word's four bytes all derive from the group index g.
 */
int mt_band_frequency(birriasdr_dev_t *dev, uint64_t frequency_hz)
{
    uint32_t g = mt_band_step(MT_STEP_GROUP, frequency_hz);
    uint32_t group_word = swap_nibbles(0x14 + g) << 24 | (0x13 + g) << 16 |
                          swap_nibbles(g + 1) << 8 | g;
    BS_TRY(mt_rf_update(dev, 0, 0x0bc, 0xff000000,
                        (uint32_t)mt_band_step(MT_STEP_FRONTEND, frequency_hz) << 24));
    BS_TRY(mt_rf_update(dev, 1, 0x0bc, 0xff000000, 0x55000000));
    BS_TRY(mt_rf_write(dev, 1, 0x184, (uint32_t)(frequency_hz / 1000)));
    return bs_write32(dev, BS_MMIO, 0x830ad40c, group_word);
}
