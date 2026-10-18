/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * MT7961/MT7972 receive gain overrides and restoration
 *
 * Manual gain is a coordinated analog/PHY override, not one amplifier setting.
 * The tested chain-0 path uses three digital MMIO words and one analog word
 * reached through physical RF SPI port 1. Only the fields in gain_masks belong
 * to this module; read-modify-write preserves every other field.
 *
 *   PHY 0x830000c4: A/B/C force bits [2:0]; LNA codes [10:8], [18:16],
 *                   [26:24]; LPF codes [15:12], [23:20], [31:28].
 *   PHY 0x83000108: digital A [29:24] and C [21:16].
 *   PHY 0x83000150: force D [31], LNA [22:20]; clear owned [15:12].
 *   RF  0x768:      analog force [31], LNA [30:27], LPF gain [26:23].
 *
 * A/B/C/D name the decoded PHY field groups, not four receive antennas. Their
 * individual contributions are not all characterized. The validated control
 * is the combined recipe below, using LNA 0..5, LPF 0..8 and digital 0..12.
 * These are raw indices, not dB. LPF gain changes the filter-stage gain, not
 * its bandwidth; writing a wider numeric code is not a wider RF passband.
 *
 * Initialize this recipe after RF initialization, before capture starts. Snapshot
 * the baseline and reject pre-existing forces, including coarse force
 * 0x830000d0[27], rather than saving a forced state as an automatic baseline.
 * Clamp the analog path first, then program digital and PHY overrides, checking
 * masked readback after every write. On failure, restore digital/PHY fields
 * before releasing the analog clamp. Attempt every restore even if one fails.
 *
 * WFSYS reset does not reliably clear the analog overrides. mt_reset() must
 * call mt_restore_gain() BEFORE resetting; otherwise the next capture can
 * inherit forced gain and fail baseline validation. Keep the saved baseline
 * after a restore failure so later cleanup can retry it.
 *
 * Automatic/released mode means leaving the initialized baseline unforced.
 * It does not prove full AGC operation or that manual mode freezes every gain
 * loop. Calibrated dB, AGC target, attack/decay and complete AGC disable remain
 * unsupported. The private update helper can reuse the saved baseline while
 * capturing, provided the reader owns all RF/register access. The four writes
 * are not simultaneous and buffered samples precede their acknowledgment;
 * callers must not present a host callback offset as an exact RF boundary.
 * See docs/mt7921.md for the scope of these controls.
 */
#include "mt7921.h"
#include <string.h>

static const uint32_t gain_addresses[] = {0x830000c4, 0x83000108, 0x83000150, 0x768};
static const uint32_t gain_masks[] = {0xf7f7f707, 0x3f3f0000, 0x8070f000, 0xff800000};

/* Three PHY words and one analog override together control the measured path.
 * Reading them as a group gives rollback state without copying unknown fields.
 */
static int gain_snapshot(birriasdr_dev_t *dev, uint32_t values[4])
{
    for (unsigned i = 0; i < 3; ++i)
        BS_TRY(bs_read32(dev, BS_MMIO, gain_addresses[i], &values[i]));
    return mt_rf_read(dev, MT_RX_PORT, gain_addresses[3], &values[3]);
}

/* Restore only owned fields and verify them. The analog override uses SPI 1,
 * while the first three words are digital PHY MMIO for receive chain 0.
 */
static int gain_write(birriasdr_dev_t *dev, unsigned field, uint32_t value)
{
    uint32_t actual;
    uint32_t mask = gain_masks[field], address = gain_addresses[field];
    if (field == 3) {
        BS_TRY(mt_rf_update(dev, MT_RX_PORT, address, mask, value & mask));
        BS_TRY(mt_rf_read(dev, MT_RX_PORT, address, &actual));
    } else {
        BS_TRY(bs_update32(dev, BS_MMIO, address, mask, value & mask));
        BS_TRY(bs_read32(dev, BS_MMIO, address, &actual));
    }
    return ((actual ^ value) & mask) ? BIRRIASDR_ERROR_STATE : 0;
}

/* Apply one four-field transaction, preserving unrelated fields. A failed
 * update restores the immediately preceding settings, not the boot baseline.
 * The baseline remains saved separately for release and final reset cleanup.
 */
static int gain_transaction(birriasdr_dev_t *dev, const uint32_t before[4],
                            const uint32_t values[4], bool manual)
{
    static const unsigned order[] = {3, 1, 2, 0};
    static const unsigned restore[] = {1, 2, 0, 3};
    const unsigned *sequence = manual ? order : restore;
    int result = 0;
    for (unsigned i = 0; i < 4; ++i) {
        result = gain_write(dev, sequence[i], values[sequence[i]]);
        if (result < 0)
            break;
    }
    if (result < 0) {
        bool rollback_failed = false;
        for (unsigned i = 0; i < 4; ++i)
            if (gain_write(dev, restore[i], before[restore[i]]) < 0)
                rollback_failed = true;
        if (rollback_failed)
            return BIRRIASDR_ERROR_STATE;
    }
    return result;
}

/* Build raw forced indices; these masks never imply calibrated gain in dB. */
static int gain_values(const struct birriasdr_gain *gain, uint32_t values[4])
{
    uint32_t lna = gain->lna, lpf = gain->lpf, digital = gain->digital;
    if (lna > 5 || lpf > 8 || digital > 12)
        return BIRRIASDR_ERROR_INVALID;
    values[0] = 7 | lna << 8 | lpf << 12 | lna << 16 | lpf << 20 | lna << 24 | lpf << 28;
    values[1] = digital << 24 | digital << 16;
    values[2] = 0x80000000u | lna << 20;
    values[3] = 0x80000000u | lna << 27 | lpf << 23;
    return 0;
}

/* Each read boots into an unforced baseline. Save it even in released mode so
 * a later manual update cannot mistake its own force bits for the baseline.
 * Only the capture owner may call this initialization, before releasing GO.
 */
int mt_apply_gain(birriasdr_dev_t *dev)
{
    uint32_t before[4], coarse, values[4];
    BS_TRY(gain_values(&dev->gain, values));
    BS_TRY(gain_snapshot(dev, before));
    BS_TRY(bs_read32(dev, BS_MMIO, 0x830000d0, &coarse));
    if ((before[0] & 7) || (before[2] & 0x80000000) ||
        (before[3] & 0x80000000) || (coarse & (1u << 27)))
        return BIRRIASDR_ERROR_STATE;
    memcpy(dev->mt7921.gain_baseline, before, sizeof(before));
    dev->mt7921.gain_saved = true;
    return dev->manual ? gain_transaction(dev, before, values, true) : 0;
}

/* Change gain on an initialized receiver without stopping ICAP or rebooting.
 * The caller must serialize the complete operation with other register access.
 * No firmware command is needed: the streaming MCU never touches these fields.
 * Readback confirms register application, not analog settling or a sample index.
 * Leave requested API settings to the caller, which commits only on success.
 */
int mt_change_gain(birriasdr_dev_t *dev, bool manual, const struct birriasdr_gain *gain)
{
    uint32_t before[4], values[4];
    if (!gain)
        return BIRRIASDR_ERROR_INVALID;
    BS_TRY(gain_values(gain, values));
    if (!dev->mt7921.gain_saved)
        return BIRRIASDR_ERROR_STATE;
    if (!manual)
        memcpy(values, dev->mt7921.gain_baseline, sizeof(values));
    BS_TRY(gain_snapshot(dev, before));
    return gain_transaction(dev, before, values, manual);
}

/* WFSYS reset does not reliably erase analog gain overrides. Restore this
 * capture's saved baseline BEFORE reset, otherwise the next read finds forced
 * gain and cannot establish an honest automatic baseline. Always attempt all
 * fields; retain the baseline after a failure so close can retry cleanup.
 */
int mt_restore_gain(birriasdr_dev_t *dev)
{
    static const unsigned order[] = {1, 2, 0, 3};
    int first_error = 0;
    if (!dev->mt7921.gain_saved)
        return 0;
    for (unsigned i = 0; i < 4; ++i) {
        unsigned field = order[i];
        int result = gain_write(dev, field, dev->mt7921.gain_baseline[field]);
        if (result < 0 && !first_error)
            first_error = result;
    }
    if (!first_error)
        dev->mt7921.gain_saved = false;
    return first_error;
}
