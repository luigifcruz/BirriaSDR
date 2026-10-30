/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/* Test the birria_udp option parsing and packetization without hardware. The
 * example is included directly, with its main renamed, so its static functions
 * can be called.
 */
#include <sys/time.h>

#define main birria_udp_program_main
#include "../examples/birria_udp.c"
#undef main

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "check failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

/* Check defaults, every option and the rejection of out-of-range values. */
static int test_options(void)
{
    struct options options;
    char *defaults[] = {(char *)"birria_udp"};
    char *configured[] = {
        (char *)"birria_udp", (char *)"--host", (char *)"192.0.2.1",
        (char *)"--port", (char *)"4321", (char *)"--frequency",
        (char *)"2437000000", (char *)"--rate", (char *)"10000000",
        (char *)"--device", (char *)"2",
        (char *)"--packet-samples", (char *)"4096", (char *)"--recovery",
        (char *)"0", (char *)"--manual"
    };
    char *bad_port[] = {(char *)"birria_udp", (char *)"--port", (char *)"0"};
    char *bad_packet[] = {
        (char *)"birria_udp", (char *)"--packet-samples", (char *)"16377"
    };

    CHECK(parse_options(1, defaults, &options) == 0);
    CHECK(!strcmp(options.host, DEFAULT_HOST));
    CHECK(!strcmp(options.port, DEFAULT_PORT));
    CHECK(options.device_index == 0);
    CHECK(options.packet_samples == DEFAULT_PACKET_SAMPLES);
    CHECK(options.recovery_restarts == DEFAULT_RECOVERY_RESTARTS);
    CHECK(options.rate == 0);
    CHECK(!options.manual_gain);

    CHECK(parse_options((int)(sizeof(configured) / sizeof(configured[0])),
                        configured, &options) == 0);
    CHECK(!strcmp(options.host, "192.0.2.1"));
    CHECK(!strcmp(options.port, "4321"));
    CHECK(options.frequency == UINT64_C(2437000000));
    CHECK(options.rate == 10000000);
    CHECK(options.device_index == 2);
    CHECK(options.packet_samples == 4096);
    CHECK(options.recovery_restarts == 0);
    CHECK(options.manual_gain);

    CHECK(parse_options(3, bad_port, &options) < 0);
    CHECK(parse_options(3, bad_packet, &options) < 0);
    return 0;
}

/* Check datagram boundaries and little-endian bytes over a local socket pair. */
static int test_datagrams(void)
{
    static const int16_t samples[] = {
        1, -2, 0x1234, (int16_t)0xabcd, -8192, 8191, 0, -1, 42, -42
    };
    static const uint8_t first_expected[] = {
        0x01, 0x00, 0xfe, 0xff, 0x34, 0x12, 0xcd, 0xab,
        0x00, 0xe0, 0xff, 0x1f
    };
    static const uint8_t second_expected[] = {
        0x00, 0x00, 0xff, 0xff, 0x2a, 0x00, 0xd6, 0xff
    };
    struct timeval timeout = {.tv_sec = 1};
    static struct udp_stream stream = {.socket_fd = -1, .packet_samples = 3};
    uint8_t received[64];
    int sockets[2];
    ssize_t length;

    CHECK(!socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets));
    CHECK(!setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)));
    stream.socket_fd = sockets[0];
    atomic_store(&interrupted, false);
    send_samples(samples, 5, 0, &stream);
    CHECK(!stream.error);
    CHECK(stream.samples_seen == 5);
    CHECK(stream.datagrams_sent == 1);
    CHECK(stream.pending_samples == 2);
    CHECK(!send_pending(&stream));
    CHECK(stream.samples_sent == 5);
    CHECK(stream.datagrams_sent == 2);
    CHECK(stream.samples_dropped == 0);
    CHECK(stream.datagrams_dropped == 0);

    length = recv(sockets[1], received, sizeof(received), 0);
    CHECK(length == (ssize_t)sizeof(first_expected));
    CHECK(!memcmp(received, first_expected, sizeof(first_expected)));
    length = recv(sockets[1], received, sizeof(received), 0);
    CHECK(length == (ssize_t)sizeof(second_expected));
    CHECK(!memcmp(received, second_expected, sizeof(second_expected)));

    close(sockets[0]);
    close(sockets[1]);
    return 0;
}

/* Check that a partial datagram carries over to the next callback, so every
 * datagram holds packet_samples until the final flush.
 */
static int test_carry(void)
{
    static const int16_t first[] = {1, 2, 3, 4};
    static const int16_t second[] = {5, 6, 7, 8};
    static const uint8_t full[] = {1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0};
    static const uint8_t rest[] = {7, 0, 8, 0};
    struct timeval timeout = {.tv_sec = 1};
    static struct udp_stream stream;
    uint8_t received[64];
    int sockets[2];

    CHECK(!socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets));
    CHECK(!setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)));
    memset(&stream, 0, sizeof(stream));
    stream.socket_fd = sockets[0];
    stream.packet_samples = 3;
    atomic_store(&interrupted, false);
    send_samples(first, 2, 0, &stream);
    CHECK(stream.datagrams_sent == 0 && stream.pending_samples == 2);
    send_samples(second, 2, 2, &stream);
    CHECK(stream.datagrams_sent == 1 && stream.pending_samples == 1);
    CHECK(recv(sockets[1], received, sizeof(received), 0) == (ssize_t)sizeof(full));
    CHECK(!memcmp(received, full, sizeof(full)));
    CHECK(!send_pending(&stream));
    CHECK(recv(sockets[1], received, sizeof(received), 0) == (ssize_t)sizeof(rest));
    CHECK(!memcmp(received, rest, sizeof(rest)));
    CHECK(stream.samples_sent == 4 && stream.datagrams_sent == 2);

    close(sockets[0]);
    close(sockets[1]);
    return 0;
}

/* Check that a full receiver drops datagrams without failing the stream, and
 * that any other send error fails it and leaves its samples unsent.
 */
static int test_send_failures(void)
{
    static const int16_t samples[] = {1, 2, 3, 4, 5, 6};
    static struct udp_stream stream;
    int sockets[2];
    int small = 2048;

    CHECK(!socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets));
    CHECK(!fcntl(sockets[0], F_SETFL, fcntl(sockets[0], F_GETFL, 0) | O_NONBLOCK));
    (void)setsockopt(sockets[1], SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
    memset(&stream, 0, sizeof(stream));
    stream.socket_fd = sockets[0];
    stream.packet_samples = 3;
    atomic_store(&interrupted, false);
    for (int i = 0; i < 100000 && !stream.datagrams_dropped && !stream.error; ++i)
        send_samples(samples, 3, 0, &stream);
    CHECK(!stream.error);
    CHECK(stream.datagrams_dropped > 0);
    CHECK(stream.samples_dropped == stream.datagrams_dropped * 3);
    close(sockets[0]);
    close(sockets[1]);

    memset(&stream, 0, sizeof(stream));
    stream.socket_fd = -1;
    stream.packet_samples = 3;
    send_samples(samples, 3, 0, &stream);
    CHECK(stream.error);
    CHECK(stream.samples_seen == 3);
    CHECK(stream.samples_sent == 0 && stream.samples_dropped == 0);
    return 0;
}

/* Check that a gap flushes the partial datagram first, so no datagram spans it,
 * and that packing starts fresh after the gap.
 */
static int test_gap_flush(void)
{
    static const int16_t before[] = {1, 2, 3, 4};
    static const int16_t after[] = {5, 6, 7, 8, 9, 10};
    static const uint8_t flushed[] = {1, 0, 2, 0, 3, 0, 4, 0};
    static const uint8_t next[] = {5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0};
    const struct birriasdr_recovery_event event = {.next_sample = 2, .attempt = 1};
    struct timeval timeout = {.tv_sec = 1};
    static struct udp_stream stream;
    uint8_t received[64];
    int sockets[2];

    CHECK(!socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets));
    CHECK(!setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)));
    memset(&stream, 0, sizeof(stream));
    stream.socket_fd = sockets[0];
    stream.packet_samples = 3;
    atomic_store(&interrupted, false);
    send_samples(before, 2, 0, &stream);
    CHECK(stream.datagrams_sent == 0 && stream.pending_samples == 2);
    on_recovery(&event, &stream);
    CHECK(stream.gaps == 1 && stream.pending_samples == 0 && stream.datagrams_sent == 1);
    CHECK(recv(sockets[1], received, sizeof(received), 0) == (ssize_t)sizeof(flushed));
    CHECK(!memcmp(received, flushed, sizeof(flushed)));
    send_samples(after, 3, 2, &stream);
    CHECK(stream.datagrams_sent == 2 && stream.pending_samples == 0);
    CHECK(recv(sockets[1], received, sizeof(received), 0) == (ssize_t)sizeof(next));
    CHECK(!memcmp(received, next, sizeof(next)));

    close(sockets[0]);
    close(sockets[1]);
    return 0;
}

int main(void)
{
    CHECK(!test_options());
    CHECK(!test_datagrams());
    CHECK(!test_carry());
    CHECK(!test_send_failures());
    CHECK(!test_gap_flush());
    puts("UDP option and packetization tests passed");
    return 0;
}
