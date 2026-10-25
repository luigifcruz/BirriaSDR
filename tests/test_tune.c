/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "chips/mt7921.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); exit(1); } } while (0)

/* Check known words, exact integer boundaries and fractional carry. These
 * values expose the old 500 Hz truncation and the danger of OR-ing a rounded
 * fractional overflow into an odd integer divider instead of carrying it.
 */
static void test_reference_words(void)
{
    struct mt_2g_tuning plan, fine;
    CHECK(mt_plan_2g(UINT64_C(2437000000), 18, &plan) == 0);
    CHECK(plan.word == 0x8513bbbcu && plan.calibration_cycles == 19494);
    CHECK(plan.nominal_hz_num == UINT64_C(1277689860000000));
    CHECK(plan.nominal_hz_den == 524288);
    CHECK(mt_plan_2g(UINT64_C(2400000000), 18, &plan) == 0);
    CHECK(plan.word == 0x85000000u);
    CHECK(mt_plan_2g(UINT64_C(2459999999), 18, &plan) == 0);
    CHECK(plan.word == 0x85200000u); /* Divider 81.999... must carry to 82. */
    CHECK(mt_plan_2g(UINT64_C(2460000000), 18, &fine) == 0);
    CHECK(plan.word == fine.word);
    CHECK(mt_plan_2g(UINT64_C(2400000114), 18, &plan) == 0);
    CHECK(plan.word == 0x85000004u); /* A step smaller than the old 500 Hz grid. */
    CHECK(mt_plan_2g(UINT64_C(2400000029), 20, &fine) == 0);
    CHECK(fine.word == 0x85000001u); /* The normal path now uses this low bit. */
    CHECK(mt_plan_2g(UINT64_C(2400000057), MT_RF_FRACTION_BITS, &fine) == 0);
    CHECK(fine.word == 0x85000002u);
    CHECK(mt_plan_2g(UINT64_C(2400000086), MT_RF_FRACTION_BITS, &fine) == 0);
    CHECK(fine.word == 0x85000003u);
    CHECK(mt_plan_2g(UINT64_C(2437000000), MT_RF_FRACTION_BITS, &fine) == 0);
    CHECK(fine.word == 0x8513bbbcu); /* The default center's word is unchanged. */
    CHECK(mt_plan_2g(UINT64_C(2459999999), MT_RF_FRACTION_BITS, &fine) == 0);
    CHECK(fine.word == 0x85200000u); /* Carry also works at the default precision. */
    CHECK(mt_plan_2g(UINT64_C(2300000000), 18, &plan) == 0);
    CHECK(plan.word == 0x84caaaacu); /* Off-channel and below the Wi-Fi band. */
}

/* Independently bound rounding error over a wide numerical interval, including
 * sub-channel steps. Exact cross-products avoid a floating-point tolerance that
 * could hide off-by-one divider errors. This tests arithmetic, not RF coverage.
 */
static void test_quantization(void)
{
    for (unsigned bits = 18; bits <= 20; bits += 2) {
        struct mt_2g_tuning plan;
        uint32_t previous = 0;
        for (uint64_t hz = UINT64_C(500000000); hz < UINT64_C(3500000000); hz += 7919) {
            uint64_t requested, error;
            CHECK(mt_plan_2g(hz, bits, &plan) == 0);
            requested = hz * plan.nominal_hz_den;
            error = requested > plan.nominal_hz_num ? requested - plan.nominal_hz_num :
                    plan.nominal_hz_num - requested;
            CHECK(error <= 30000000); /* At most half a 60 MHz reference count. */
            CHECK(plan.word >= previous);
            CHECK((plan.word & 0x40000000u) == 0);
            if (bits == 18) CHECK((plan.word & 3) == 0);
            previous = plan.word;
        }
    }
}

/* Reject invalid widths and values before overflow; preserve the output on
 * rejection so callers cannot accidentally use a partly computed plan.
 */
static void test_limits(void)
{
    struct mt_2g_tuning plan = {.word = 123};
    CHECK(mt_plan_2g(0, 18, &plan) == BIRRIASDR_ERROR_INVALID);
    CHECK(mt_plan_2g(UINT64_MAX, 18, &plan) == BIRRIASDR_ERROR_INVALID);
    CHECK(mt_plan_2g(1000000, 18, &plan) == BIRRIASDR_ERROR_INVALID);
    CHECK(mt_plan_2g(UINT64_C(2437000000), 19, &plan) == BIRRIASDR_ERROR_INVALID);
    CHECK(mt_plan_2g(UINT64_C(2437000000), 18, NULL) == BIRRIASDR_ERROR_INVALID);
    CHECK(plan.word == 123);
    CHECK(mt_plan_2g(UINT64_C(8192249999), 20, &plan) == 0);
    CHECK(plan.calibration_cycles == UINT16_MAX);
    CHECK(mt_plan_2g(UINT64_C(8192250000), 20, &plan) == BIRRIASDR_ERROR_INVALID);
}

/* Visit every representable 20-bit tuning word in the public interval. Round
 * its exact rational center to integer Hz, then require that request to select
 * the same word. This covers the full encoding grid, not physical reception.
 */
static void test_full_range(void)
{
    struct mt_2g_tuning lower, upper, plan;
    uint32_t first, last;
    CHECK(mt_plan_2g(MT_RF_MIN_HZ, MT_RF_FRACTION_BITS, &lower) == 0);
    CHECK(mt_plan_2g(MT_RF_MAX_HZ, MT_RF_FRACTION_BITS, &upper) == 0);
    first = lower.word & 0x3fffffff;
    last = upper.word & 0x3fffffff;
    for (uint32_t divider = first; divider <= last; ++divider) {
        uint64_t frequency = ((uint64_t)divider * 30000000 + 524288) / 1048576;
        CHECK(mt_plan_2g(frequency, MT_RF_FRACTION_BITS, &plan) == 0);
        CHECK(plan.word == (0x80000000u | divider));
    }
    printf("%u consecutive tuning words round-trip across the public interval\n", last - first + 1);
}

/* The stock firmware's high-band words, read back from the A-die, use 18
 * fractional bits. The plan must reproduce them and their calibration counts.
 * Band membership follows the public command intervals, with gaps rejected.
 */
static void test_high_band(void)
{
    static const struct { uint64_t hz; uint32_t word, cycles; } stock[] = {
        {UINT64_C(5180000000), 0x06799998, 0x611e}, {UINT64_C(5745000000), 0x072e6668, 0x6bb6},
        {UINT64_C(5955000000), 0x07719998, 0x6fa6}, {UINT64_C(6455000000), 0x08119998, 0x7906},
        {UINT64_C(7115000000), 0x08e4cccc, 0x8566}
    };
    struct mt_2g_tuning plan = {.word = 123};
    for (size_t i = 0; i < sizeof(stock) / sizeof(stock[0]); ++i) {
        CHECK(mt_plan_high(stock[i].hz, 18, &plan) == 0);
        CHECK((plan.word & 0x7fffffff) == stock[i].word);
        CHECK(plan.calibration_cycles == stock[i].cycles);
        {
            uint64_t nominal = plan.nominal_hz_num / plan.nominal_hz_den;
            CHECK((nominal > stock[i].hz ? nominal - stock[i].hz : stock[i].hz - nominal) <= 96);
        }
        CHECK(mt_plan_high(stock[i].hz, MT_RF_FRACTION_BITS, &plan) == 0);
        CHECK(((plan.word >> 20) & 0x3ff) == ((stock[i].word >> 20) & 0x3ff));
    }
    CHECK(mt_plan_high(UINT64_C(5180000000), 19, &plan) == BIRRIASDR_ERROR_INVALID);
    CHECK(mt_plan_high(UINT64_MAX, 20, &plan) == BIRRIASDR_ERROR_INVALID);
    CHECK(mt_band_of(MT_RF_MIN_HZ) == MT_BAND_2G && mt_band_of(MT_RF_MAX_HZ) == MT_BAND_2G);
    CHECK(mt_band_of(MT_RF_MAX_HZ + 1) == MT_BAND_NONE);
    CHECK(mt_band_of(MT_RF_5G_MIN_HZ - 1) == MT_BAND_NONE);
    CHECK(mt_band_of(MT_RF_5G_MIN_HZ) == MT_BAND_5G);
    CHECK(mt_band_of(MT_RF_6G_MIN_HZ - 1) == MT_BAND_5G);
    CHECK(mt_band_of(MT_RF_6G_MIN_HZ) == MT_BAND_6G);
    CHECK(mt_band_of(MT_RF_6G_MAX_HZ) == MT_BAND_6G);
    CHECK(mt_band_of(MT_RF_6G_MAX_HZ + 1) == MT_BAND_NONE);
    CHECK(mt_band_step(MT_STEP_GROUP, UINT64_C(6455000000)) == 13);
    CHECK(mt_band_step(MT_STEP_LOOP, UINT64_C(6935000000)) == 8);
}

/* Run without USB access. Physical range and fine-step tests require a signal. */
int main(void)
{
    test_reference_words();
    test_quantization();
    test_limits();
    test_full_range();
    test_high_band();
    return 0;
}
