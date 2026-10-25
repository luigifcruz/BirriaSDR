/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/* Test bandwidth setup against an independent four-register transport model.
 * Inject errors at every successful-path I/O boundary, a lost register write,
 * and a failed rollback. No USB discovery or physical access is performed.
 */
#include "chips/mt7921.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(t) do { if (!(t)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #t); exit(1); } } while (0)

struct model {
    uint32_t value[4];
    unsigned operations, fail_at, fail_again, writes;
    bool drop_write;
};
static const uint32_t baseline[] = {0x2a123456, 0x17654321, 0xa0abcdef, 0xd0765432};

/* Faults fail once at the requested access; all later cleanup is observable. */
static int access_word(struct model *model, unsigned field, uint32_t *value, bool write)
{
    CHECK(field < 4);
    ++model->operations;
    if (model->operations == model->fail_at || model->operations == model->fail_again)
        return BIRRIASDR_ERROR_IO;
    if (write) {
        ++model->writes;
        if (model->drop_write) model->drop_write = false;
        else model->value[field] = *value;
    } else {
        *value = model->value[field];
    }
    return 0;
}

/* The model exposes exactly the two PHY words owned by this module. */
static int read_word(void *context, enum bs_space space, uint32_t address, uint32_t *value)
{
    CHECK(space == BS_MMIO && (address == 0x83081000 || address == 0x83091000));
    return access_word(context, address == 0x83081000 ? 2 : 3, value, false);
}

static int write_word(void *context, enum bs_space space, uint32_t address, uint32_t value)
{
    CHECK(space == BS_MMIO && (address == 0x83081000 || address == 0x83091000));
    return access_word(context, address == 0x83081000 ? 2 : 3, &value, true);
}

/* Analog access is modeled as a separate register space, retaining bank IDs. */
int mt_rf_read(birriasdr_dev_t *dev, unsigned port, uint32_t address, uint32_t *value)
{
    CHECK(port < 2 && address == 0x160);
    return access_word(dev->io.context, port, value, false);
}

int mt_rf_update(birriasdr_dev_t *dev, unsigned port, uint32_t address,
                 uint32_t clear, uint32_t set)
{
    uint32_t value;
    BS_TRY(mt_rf_read(dev, port, address, &value));
    value = (value & ~clear) | set;
    return access_word(dev->io.context, port, &value, true);
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
    puts("Filter configuration, preservation, rollback and readback tests passed");
    return 0;
}
