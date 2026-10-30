/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/* Test the birria_rx argument parser without hardware. The example is included
 * directly, with its main renamed, so its static functions can be called. A
 * missing option value or an option-like token must never become the output
 * file, because that would start a capture instead of reporting usage.
 */
#define main birria_rx_program_main
#include "../examples/birria_rx.c"
#undef main

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "check failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

/* Parse a NULL-terminated argument list. */
static int parse(struct arguments *arguments, const char *const *list)
{
    char *argv[16];
    int argc = 0;
    while (list[argc]) {
        argv[argc] = (char *)list[argc];
        ++argc;
    }
    return parse_arguments(argc, argv, arguments);
}

int main(void)
{
    struct arguments a;
    CHECK(!parse(&a, (const char *const[]){"birria_rx", "out.cs16", NULL}));
    CHECK(!strcmp(a.output, "out.cs16") && !a.frequency && !a.rate && !a.limit && !a.manual);
    CHECK(!parse(&a, (const char *const[]){"birria_rx", "--rate", "20000000", "--frequency",
                                           "5180000000", "out.cs16", "4000000", "manual", NULL}));
    CHECK(a.rate == 20000000 && a.frequency == UINT64_C(5180000000));
    CHECK(!strcmp(a.output, "out.cs16") && a.limit == 4000000 && a.manual);

    /* A missing value, an unknown option or an option-like output fails. */
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--rate", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--frequency", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--rate", "20000000", "--frequency", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--rate", "out.cs16", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--help", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--rate", "20000000", "--help", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", NULL}));

    /* Bad numbers, extra arguments and a wrong gain word fail too. */
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--rate", "0", "out.cs16", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "--rate", "4294967296", "out.cs16", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "out.cs16", "10x", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "out.cs16", "10", "auto", NULL}));
    CHECK(parse(&a, (const char *const[]){"birria_rx", "out.cs16", "10", "manual", "x", NULL}));
    puts("birria_rx argument tests passed");
    return 0;
}
