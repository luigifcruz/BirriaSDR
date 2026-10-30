/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/* Test bandwidth setup against an independent register transport model. The
 * 40 MSps profile may touch only its original four registers. The 10, 20 and
 * 80 MSps profiles also use the per-chain, channel, mode and latch registers.
 * Inject errors at every successful-path I/O boundary, a lost register write,
 * and a failed rollback. No USB discovery or physical access is performed.
 */
#include "chips/mt7921.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(t) do { if (!(t)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #t); exit(1); } } while (0)

/* Registers 0 to 3 are the original 40 MSps profile. The rest are used only
 * by the other rates: per-chain analog words, channel, 20 MHz mode and the two
 * rate class latches.
 */
enum { RF0_160, RF1_160, DIG0, DIG1, RF0_B00, RF1_B00, RF0_B04, RF1_B04, CHANNEL, MODE20,
       LATCH0, LATCH1, REGISTERS };
struct model {
    uint32_t value[REGISTERS];
    unsigned operations, fail_at, fail_again, writes, latch_pulses;
    bool drop_write, wide, stick_latch;
};
static const uint32_t baseline[REGISTERS] = {
    0x2a123456, 0x17654321, 0xa0abcdef, 0xd0765432, 0x3f00a5a5, 0x15005a5a,
    0x1f0000c3, 0x0a00003c, 0x123456f9, 0x00001001, 0x00000011, 0x00000022};

/* Faults fail once at the requested access; all later cleanup is observable. */
static int access_word(struct model *model, unsigned field, uint32_t *value, bool write)
{
    CHECK(field < REGISTERS && (model->wide || field < 4));
    ++model->operations;
    if (model->operations == model->fail_at || model->operations == model->fail_again)
        return BIRRIASDR_ERROR_IO;
    if (write) {
        ++model->writes;
        if ((field == LATCH0 || field == LATCH1) && (*value & 0x80000000u) &&
            !(model->value[field] & 0x80000000u))
            ++model->latch_pulses;
        /* A stuck latch ignores the one write that should clear its high bit. */
        if (model->stick_latch && field == LATCH0 && !(*value & 0x80000000u) &&
            (model->value[field] & 0x80000000u)) {
            model->stick_latch = false;
            return 0;
        }
        if (model->drop_write) model->drop_write = false;
        else model->value[field] = *value;
    } else {
        *value = model->value[field];
    }
    return 0;
}

/* Map each MMIO word the filter may use to its model register. */
static unsigned mmio_register(uint32_t address)
{
    switch (address) {
    case 0x83081000: return DIG0;
    case 0x83091000: return DIG1;
    case 0x830a0080: return CHANNEL;
    case 0x83080000: return MODE20;
    case 0x8300c008: return LATCH0;
    case 0x8301c008: return LATCH1;
    default: CHECK(!"unexpected MMIO address"); return REGISTERS;
    }
}

static int read_word(void *context, enum bs_space space, uint32_t address, uint32_t *value)
{
    CHECK(space == BS_MMIO);
    return access_word(context, mmio_register(address), value, false);
}

static int write_word(void *context, enum bs_space space, uint32_t address, uint32_t value)
{
    CHECK(space == BS_MMIO);
    return access_word(context, mmio_register(address), &value, true);
}

/* Map an analog bank and address to its model register. */
static unsigned rf_register(unsigned port, uint32_t address)
{
    CHECK(port < 2);
    if (address == 0x160) return port ? RF1_160 : RF0_160;
    if (address == 0xb00) return port ? RF1_B00 : RF0_B00;
    CHECK(address == 0xb04);
    return port ? RF1_B04 : RF0_B04;
}

/* Analog access is modeled as a separate register space, retaining bank IDs. */
int mt_rf_read(birriasdr_dev_t *dev, unsigned port, uint32_t address, uint32_t *value)
{
    return access_word(dev->io.context, rf_register(port, address), value, false);
}

int mt_rf_update(birriasdr_dev_t *dev, unsigned port, uint32_t address,
                 uint32_t clear, uint32_t set)
{
    uint32_t value;
    BS_TRY(mt_rf_read(dev, port, address, &value));
    value = (value & ~clear) | set;
    return access_word(dev->io.context, rf_register(port, address), &value, true);
}

/* Seed unrelated bits differently in both banks to catch accidental clobbers. */
static void initialize(struct model *model, birriasdr_dev_t *dev)
{
    memset(model, 0, sizeof(*model));
    memset(dev, 0, sizeof(*dev));
    memcpy(model->value, baseline, sizeof(baseline));
    dev->rate = 40000000;
    dev->io = (struct bs_io){model, read_word, write_word};
}

/* Expected registers after configuration, written out from the documented
 * field layout rather than taken from the implementation's table.
 */
static void expected_profile(uint32_t rate, uint32_t channel, uint32_t out[REGISTERS])
{
    const bool wide = rate == 80000000;
    memcpy(out, baseline, sizeof(baseline));
    out[RF0_160] = (baseline[RF0_160] & ~0xc0000000u) | (wide ? 0x80000000u : 0);
    out[RF1_160] = (baseline[RF1_160] & ~0xc0000000u) | (wide ? 0x80000000u : 0);
    out[DIG0] = (baseline[DIG0] & ~0x7f000000u) | (wide ? 0x02000000u : 0) |
                (rate == 10000000 ? 0x70000000u : 0);
    out[DIG1] = (baseline[DIG1] & ~0x7f000000u) | (wide ? 0x02000000u : 0) |
                (rate == 10000000 ? 0x70000000u : 0);
    out[RF0_B00] = (baseline[RF0_B00] & ~0x3f000000u) | (wide ? 0x20000000u : 0x10000000u);
    out[RF1_B00] = (baseline[RF1_B00] & ~0x3f000000u) | (wide ? 0x20000000u : 0x10000000u);
    out[RF0_B04] = (baseline[RF0_B04] & ~0x1f0000c0u) | (wide ? 0x1f000080u : 0x0f000040u);
    out[RF1_B04] = (baseline[RF1_B04] & ~0x1f0000c0u) | (wide ? 0x1f000080u : 0x0f000040u);
    out[CHANNEL] = (baseline[CHANNEL] & ~0xf0u) | (wide ? 0x60u : channel << 4);
    out[MODE20] = (baseline[MODE20] & ~0x1000u) | (wide ? 0 : 0x1000u);
    out[LATCH0] = baseline[LATCH0] & ~0x80000000u;
    out[LATCH1] = baseline[LATCH1] & ~0x80000000u;
}

/* Field registers back at baseline, with the latch bits left clear. */
static void check_restored(const struct model *model, uint32_t flipped)
{
    for (unsigned i = 0; i < LATCH0; ++i)
        CHECK(model->value[i] == (baseline[i] ^ flipped));
    CHECK(!(model->value[LATCH0] & 0x80000000u) && !(model->value[LATCH1] & 0x80000000u));
}

/* Configure, preserve unrelated bits, restore and survive a failure at every
 * I/O boundary at 10, 20 and 80 MSps. Each profile latches its rate class.
 */
static void test_rate_profile(unsigned index)
{
    static const uint32_t rates[] = {10000000, 20000000, 80000000};
    static const uint64_t centers[] = {2480000000u, 2437000000u, 5180000000u};
    static const uint32_t channels[] = {14, 6, 0};
    struct model model;
    birriasdr_dev_t dev;
    uint32_t expected[REGISTERS];
    initialize(&model, &dev);
    model.wide = true;
    dev.rate = rates[index];
    dev.frequency = centers[index];
    expected_profile(dev.rate, channels[index], expected);
    CHECK(mt_configure_filter(&dev) == 0 && dev.mt7921.filter_saved);
    unsigned successful_operations = model.operations;
    CHECK(!memcmp(model.value, expected, sizeof(expected)));
    CHECK(model.latch_pulses == 2);
    for (unsigned i = 0; i < LATCH0; ++i) model.value[i] ^= 1;
    CHECK(mt_restore_filter(&dev) == 0 && !dev.mt7921.filter_saved);
    CHECK(model.latch_pulses == 4);
    check_restored(&model, 1);

    for (unsigned failure = 1; failure <= successful_operations; ++failure) {
        initialize(&model, &dev);
        model.wide = true;
        dev.rate = rates[index];
        dev.frequency = centers[index];
        model.fail_at = failure;
        CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_IO);
        CHECK(!dev.mt7921.filter_saved);
        check_restored(&model, 0);
        CHECK(mt_configure_filter(&dev) == 0); /* Failed preparation is reusable. */
        CHECK(!memcmp(model.value, expected, sizeof(expected)));
        CHECK(mt_restore_filter(&dev) == 0);
    }
}

/* A latch bit left high still gets a full pulse, and one that will not clear
 * fails the setup and restores the baseline.
 */
static void test_latch(void)
{
    struct model model;
    birriasdr_dev_t dev;
    initialize(&model, &dev);
    model.wide = true;
    model.value[LATCH0] |= 0x80000000u;
    model.value[LATCH1] |= 0x80000000u;
    dev.rate = 80000000;
    dev.frequency = 2437000000u;
    CHECK(mt_configure_filter(&dev) == 0);
    CHECK(model.latch_pulses == 2);
    CHECK(!(model.value[LATCH0] & 0x80000000u) && !(model.value[LATCH1] & 0x80000000u));
    CHECK(mt_restore_filter(&dev) == 0);

    initialize(&model, &dev);
    model.wide = true;
    dev.rate = 80000000;
    dev.frequency = 2437000000u;
    model.stick_latch = true;
    CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_STATE);
    CHECK(!dev.mt7921.filter_saved);
    check_restored(&model, 0);
}

/* At 10 and 20 MSps the channel field holds the nearest 2.4 GHz channel,
 * compared in Hz.
 */
static void test_channels(void)
{
    static const struct { uint64_t hz; uint32_t channel; } cases[] = {
        {2156000000u, 1},  {2412000000u, 1},  {2414000000u, 1},  {2414500000u, 1},
        {2414750000u, 2},  {2415000000u, 2},  {2437000000u, 6},  {2462000000u, 11},
        {2472000000u, 13}, {2478000000u, 13}, {2478000001u, 14}, {2479000000u, 14},
        {2484000000u, 14}, {2610000000u, 14}, {5180000000u, 14}};
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        struct model model;
        birriasdr_dev_t dev;
        initialize(&model, &dev);
        model.wide = true;
        dev.rate = 20000000;
        dev.frequency = cases[i].hz;
        CHECK(mt_configure_filter(&dev) == 0);
        CHECK(((model.value[CHANNEL] >> 4) & 0xf) == cases[i].channel);
        CHECK(mt_restore_filter(&dev) == 0);
    }
}

int main(void)
{
    struct model model;
    birriasdr_dev_t dev;
    initialize(&model, &dev);
    CHECK(mt_restore_filter(&dev) == 0 && model.operations == 0);
    CHECK(mt_configure_filter(&dev) == 0 && dev.mt7921.filter_saved);
    unsigned successful_operations = model.operations;
    CHECK(model.value[0] == 0x6a123456 && model.value[1] == 0x57654321);
    CHECK(model.value[2] == 0xa1abcdef && model.value[3] == 0xd1765432);
    CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_STATE);
    CHECK(model.operations == successful_operations);
    /* Preserve an unrelated change made after configuration during cleanup. */
    for (unsigned i = 0; i < 4; ++i) model.value[i] ^= 1;
    CHECK(mt_restore_filter(&dev) == 0 && !dev.mt7921.filter_saved);
    for (unsigned i = 0; i < 4; ++i) CHECK(model.value[i] == (baseline[i] ^ 1));

    for (unsigned failure = 1; failure <= successful_operations; ++failure) {
        initialize(&model, &dev);
        model.fail_at = failure;
        CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_IO);
        CHECK(!dev.mt7921.filter_saved);
        CHECK(memcmp(model.value, baseline, sizeof(baseline)) == 0);
        CHECK(mt_configure_filter(&dev) == 0); /* Failed preparation is reusable. */
        CHECK(mt_restore_filter(&dev) == 0);
    }
    initialize(&model, &dev);
    model.drop_write = true;
    CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_STATE);
    CHECK(!dev.mt7921.filter_saved && !memcmp(model.value, baseline, sizeof(baseline)));

    initialize(&model, &dev);
    model.fail_at = successful_operations;
    model.fail_again = successful_operations + 1;
    CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_STATE);
    CHECK(dev.mt7921.filter_saved);
    CHECK(mt_restore_filter(&dev) == 0 && !dev.mt7921.filter_saved);
    CHECK(!memcmp(model.value, baseline, sizeof(baseline)));

    initialize(&model, &dev);
    dev.rate = 5000000;
    CHECK(mt_configure_filter(&dev) == BIRRIASDR_ERROR_UNSUPPORTED && !model.operations);
    for (unsigned i = 0; i < 3; ++i)
        test_rate_profile(i);
    test_latch();
    test_channels();
    puts("Filter configuration, preservation, rollback and readback tests passed");
    return 0;
}
