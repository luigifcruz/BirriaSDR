/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * Frequency programming for the MT7972 2.4 and 5 GHz receive paths
 *
 * The selected oscillator is the secondary synthesizer on physical SPI 0,
 * while receive chain 0 is on SPI 1. The 2.4 GHz path divides the oscillator
 * by two. With the existing 60 MHz reference profile, the nominal center is
 *
 *     60 MHz * (integer + fraction / 2^20) / 2.
 *
 * The 5 GHz path runs the oscillator at 1.2 times the center instead. Three
 * band fields select it: SPI 0 0x160[29] picks the high VCO range, SPI 0
 * 0xc00[5:2] the 5 GHz LO path, and SPI 1 0x160[29] the 5 GHz front end. These
 * values match vendor firmware readbacks; the 2.4 GHz front end divides the
 * VCO by two regardless of 0xc00. Bring-up restores all three to 2.4 GHz, and
 * band writes happen only when a retune crosses bands.
 *
 * The frequency word contains LOAD at bit 31, a decoded ten-bit integer at
 * bits 29:20, and the fraction at bits 19:0. Normal tuning rounds to all
 * 20 fractional bits, giving nominal 28.6102294921875 Hz steps. The original
 * initialization recipe used 18 bits shifted left by two (114.44091796875 Hz).
 * Both low bits survive hardware readback; their physical RF effect still
 * needs a fine-step measurement. Neither number includes crystal error or drift.
 *
 * There is no Wi-Fi channel number in this programming sequence. Calculate the
 * complete fixed-point divider before packing it so rounding carries into the
 * integer correctly. Keep the input in Hz: truncating it to VCO kHz imposes an
 * unnecessary 500 Hz grid at the receiver. Channel 6 retains its exact old word.
 *
 * Calibration uses a separate cycle count, floor(VCO_hz / 250000) - 2,
 * and the same receive-only handshake as the qualified profile. Completion and
 * frequency readback do not prove oscillator lock or antenna sensitivity. The
 * arithmetic bounds below only prevent field overflow; they are not physical
 * tuning limits. Bias, output dividers, front-end filters and calibration must
 * also work at a requested center. The public command interval follows
 * the observed calibration range; it is not a qualified sensitivity claim.
 *
 * Normal initialization first establishes the channel-6 anchor and then changes
 * the selected oscillator. A live retune uses the same sequence while ICAP and
 * DMA keep running. The host's control mutex owns the RF bridge for the entire
 * transaction; our MCU never touches that bridge after GO. Calibration/routing
 * changes disturb RF samples and phase even if packet sequence stays continuous.
 * There is no exact sample timestamp or guaranteed settling interval here.
 *
 * Every retune restores the temporary routing override, including on failure.
 * A failed live retune attempts to restore the preceding center, then the common
 * layer cancels capture and resets. Only successful updates commit API metadata.
 * The separate research entry remains parked-only and permits wider diagnostic
 * sweeps. All paths change volatile receive state, never transmit or eFuse state.
 */
#include "mt7921.h"

/* Encode a divider and exact nominal frequency without floating-point rounding.
 * Reject unsupported field sizes and unrepresentable cycle/integer values before
 * multiplying. A successful plan says nothing about the physical tuning range.
 */
int mt_plan_2g(uint64_t frequency_hz, unsigned fraction_bits, struct mt_2g_tuning *plan)
{
    const uint64_t reference_hz = 60000000;
    uint64_t cycles = frequency_hz / 125000;
    uint64_t scale, divider;
    if (!plan || (fraction_bits != 18 && fraction_bits != 20))
        return BIRRIASDR_ERROR_INVALID;
    if (cycles < 2 || cycles > UINT16_MAX + UINT64_C(2))
        return BIRRIASDR_ERROR_INVALID;
    scale = UINT64_C(1) << fraction_bits;
    divider = (frequency_hz * 2 * scale + reference_hz / 2) / reference_hz;
    if (divider < scale || divider >= 1024 * scale)
        return BIRRIASDR_ERROR_INVALID;
    *plan = (struct mt_2g_tuning){
        .word = 0x80000000u | (uint32_t)(divider << (20 - fraction_bits)),
        .calibration_cycles = (uint32_t)(cycles - 2),
        .nominal_hz_num = divider * reference_hz,
        .nominal_hz_den = (uint32_t)(2 * scale)
    };
    return 0;
}

/* The 5 GHz plan: VCO = 1.2 x center, so the divider is center / 50 MHz. The
 * vendor firmware writes 18-bit fractions; normal tuning uses all 20.
 */
int mt_plan_5g(uint64_t frequency_hz, unsigned fraction_bits, struct mt_2g_tuning *plan)
{
    const uint64_t vco_reference_hz = 50000000;
    uint64_t cycles = frequency_hz * 6 / 1250000;
    uint64_t scale, divider;
    if (!plan || (fraction_bits != 18 && fraction_bits != 20))
        return BIRRIASDR_ERROR_INVALID;
    if (frequency_hz > UINT64_MAX / (UINT64_C(1) << 20) || cycles < 2 || cycles > UINT16_MAX + UINT64_C(2))
        return BIRRIASDR_ERROR_INVALID;
    scale = UINT64_C(1) << fraction_bits;
    divider = (frequency_hz * scale + vco_reference_hz / 2) / vco_reference_hz;
    if (divider < scale || divider >= 1024 * scale)
        return BIRRIASDR_ERROR_INVALID;
    *plan = (struct mt_2g_tuning){
        .word = 0x80000000u | (uint32_t)(divider << (20 - fraction_bits)),
        .calibration_cycles = (uint32_t)(cycles - 2),
        .nominal_hz_num = divider * vco_reference_hz,
        .nominal_hz_den = (uint32_t)scale
    };
    return 0;
}

bool mt_frequency_supported(uint64_t frequency_hz)
{
    return (frequency_hz >= MT_RF_MIN_HZ && frequency_hz <= MT_RF_MAX_HZ) ||
           (frequency_hz >= MT_RF_5G_MIN_HZ && frequency_hz <= MT_RF_5G_MAX_HZ);
}

/* Stop only the synthesizer calibration machine, attempting every cleanup write
 * even after an I/O failure. No transmit/loopback calibration is invoked.
 */
static int finish_calibration(birriasdr_dev_t *dev, unsigned port)
{
    int results[4];
    results[0] = mt_rf_update(dev, port, 0xcc4, (1u << 29) | (1u << 30), 0);
    results[1] = mt_rf_update(dev, port, 0xcc8, 1, 0);
    results[2] = mt_rf_update(dev, port, 0xd20, 0, 4);
    results[3] = mt_rf_write(dev, port, 0xce0, 0);
    for (size_t i = 0; i < 4; ++i)
        if (results[i] < 0)
            return results[i];
    return 0;
}

/* Start oscillator calibration and latch the result after its bounded DONE poll.
 * The status at 0xd70 is not yet a qualified lock detector and is not tested here.
 */
static int run_calibration(birriasdr_dev_t *dev, unsigned port)
{
    uint32_t value;
    uint64_t deadline;
    BS_TRY(mt_rf_update(dev, port, 0xd20, 4, 0));
    BS_TRY(mt_rf_update(dev, port, 0xcc4, 0, 1u << 30));
    BS_TRY(mt_rf_write(dev, port, 0xce0, 0x42956473));
    BS_TRY(mt_rf_update(dev, port, 0xcc8, 0, 1));
    BS_TRY(mt_rf_update(dev, port, 0xcc4, 0, 1u << 29));
    deadline = bs_now_ns() + 100000000;
    for (;;) {
        BS_TRY(mt_rf_read(dev, port, 0xd7c, &value));
        if (value & (1u << 21))
            break;
        if (bs_now_ns() >= deadline)
            return BIRRIASDR_ERROR_TIMEOUT;
        bs_sleep_us(100);
    }
    BS_TRY(mt_rf_update(dev, port, 0xcc4, 1u << 29, 0));
    BS_TRY(mt_rf_update(dev, port, 0xcc4, 1u << 30, 0));
    BS_TRY(mt_rf_read(dev, port, 0xd78, &value));
    return mt_rf_update(dev, port, 0xc0c, 7u << 22, (value & 7) << 22);
}

/* Load one synthesizer while its caller owns the complete RF transaction.
 * The initial bring-up intentionally retains its primary/secondary
 * ordering. The caller must restore the temporary 0x240 override or reset.
 */
static int tune_plan(birriasdr_dev_t *dev, unsigned port, bool secondary,
                     const struct mt_2g_tuning *planned, bool high_vco)
{
    const struct mt_2g_tuning plan = *planned;
    uint32_t reg = secondary ? 0xca8 : 0xc68;
    uint32_t selector = secondary ? 3 : 2;
    uint32_t readback;
    int result, cleanup;
    if (port != MT_RX_PORT && port != MT_SYNTH_PORT)
        return BIRRIASDR_ERROR_INVALID;
    BS_TRY(mt_rf_update(dev, port, 0x240, 0, 0xf0000000));
    BS_TRY(mt_rf_update(dev, port, 0x204, 0x00c00000, selector << 22));
    BS_TRY(mt_rf_update(dev, port, 0x21c, 0x00060000, selector << 17));
    BS_TRY(mt_rf_update(dev, port, 0x160, 1u << 29, high_vco ? 1u << 29 : 0));
    BS_TRY(mt_rf_write(dev, port, 0xcf8, 0x90));
    BS_TRY(mt_rf_write(dev, port, reg, plan.word));
    BS_TRY(mt_rf_write(dev, port, reg + 4, plan.calibration_cycles));
    BS_TRY(mt_rf_update(dev, port, reg + 16, 0x000ffff0, 0x14 * 0x1010));
    result = run_calibration(dev, port);
    cleanup = finish_calibration(dev, port);
    if (result < 0 || cleanup < 0)
        return result < 0 ? result : cleanup;
    BS_TRY(mt_rf_read(dev, port, reg, &readback));
    return readback == (plan.word & 0x7fffffff) ? 0 : BIRRIASDR_ERROR_STATE;
}

int mt_tune_2g(birriasdr_dev_t *dev, unsigned port, bool secondary,
               uint64_t frequency_hz, unsigned fraction_bits)
{
    struct mt_2g_tuning plan;
    BS_TRY(mt_plan_2g(frequency_hz, fraction_bits, &plan));
    return tune_plan(dev, port, secondary, &plan, false);
}

/* Move the LO path and front end between bands. Commit the state only after
 * both writes succeed, so a failed switch is retried by the next retune.
 */
static int select_band(birriasdr_dev_t *dev, bool high)
{
    if (dev->mt7921.high_band == high)
        return 0;
    BS_TRY(mt_rf_update(dev, MT_SYNTH_PORT, 0xc00, 0x3c, high ? 0x38 : 0x28));
    BS_TRY(mt_rf_update(dev, MT_RX_PORT, 0x160, 1u << 29, high ? 1u << 29 : 0));
    dev->mt7921.high_band = high;
    return 0;
}

/* Preserve routing around a complete selected-oscillator calibration. The
 * caller owns register access; capture need not stop for this RF-only change.
 */
static int retune_selected(birriasdr_dev_t *dev, uint64_t frequency_hz, unsigned fraction_bits)
{
    bool high = frequency_hz >= MT_RF_5G_MIN_HZ;
    struct mt_2g_tuning plan;
    uint32_t routing;
    int result, restore;
    BS_TRY(high ? mt_plan_5g(frequency_hz, fraction_bits, &plan)
                : mt_plan_2g(frequency_hz, fraction_bits, &plan));
    BS_TRY(select_band(dev, high));
    BS_TRY(mt_rf_read(dev, MT_SYNTH_PORT, 0x240, &routing));
    result = tune_plan(dev, MT_SYNTH_PORT, true, &plan, high);
    restore = mt_rf_update(dev, MT_SYNTH_PORT, 0x240, 0xf0000000, routing & 0xf0000000);
    return result < 0 ? result : restore;
}

/* Program the public tuning interval using all 20 fractional bits.
 * Reject the known diagnostic rails, but do not call unsaturated status PLL
 * lock: the full status fields and absolute RF accuracy remain uncharacterized.
 * Startup and live control both use this path, without changing gain or clocks.
 */
int mt_set_frequency(birriasdr_dev_t *dev, uint64_t frequency_hz)
{
    uint32_t diagnostic;
    if (!mt_frequency_supported(frequency_hz))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    BS_TRY(retune_selected(dev, frequency_hz, MT_RF_FRACTION_BITS));
    BS_TRY(mt_rf_read(dev, MT_SYNTH_PORT, 0xd74, &diagnostic));
    return diagnostic == 0xfffff400 || diagnostic == 0x00000400 ?
           BIRRIASDR_ERROR_STATE : 0;
}

/* Roll back a failed live update to the preceding requested center. The control
 * mutex keeps gain writes, STOP and reset out of this sequence. Rollback does
 * not make samples during either calibration valid, so the caller still stops
 * capture on failure. The staged center is committed by the common API only.
 */
int mt_change_frequency(birriasdr_dev_t *dev, uint64_t frequency_hz)
{
    int result;
    if (!mt_frequency_supported(frequency_hz))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    result = mt_set_frequency(dev, frequency_hz);
    if (result < 0 && mt_set_frequency(dev, dev->frequency) < 0)
        return BIRRIASDR_ERROR_STATE;
    return result;
}

/* Retune the selected low-band oscillator after mt_prepare(), before GO. This
 * private entry bypasses public frequency qualification for controlled research.
 * Do not call it concurrently with any other operation on the device handle.
 */
int mt_retune_2g_probe(birriasdr_dev_t *dev, uint64_t frequency_hz, unsigned fraction_bits)
{
    struct mt_2g_tuning plan;
    uint32_t state, go, control;
    BS_TRY(mt_plan_2g(frequency_hz, fraction_bits, &plan));
    if (atomic_load(&dev->run_state) != BS_IDLE)
        return BIRRIASDR_ERROR_BUSY;
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 28, &state));
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 60, &go));
    BS_TRY(bs_read32(dev, BS_MMIO, 0x80021090, &control));
    if (state != 1 || go || (control & 2))
        return BIRRIASDR_ERROR_STATE;
    return retune_selected(dev, frequency_hz, fraction_bits);
}
