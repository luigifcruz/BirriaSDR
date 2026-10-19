/* SPDX-License-Identifier: MIT */
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

static int test_options(void)
{
    struct options options;
    char *defaults[] = {(char *)"birria_udp"};
    char *configured[] = {
        (char *)"birria_udp", (char *)"--host", (char *)"192.0.2.1",
        (char *)"--port", (char *)"4321", (char *)"--frequency",
        (char *)"2437000000", (char *)"--device", (char *)"2",
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
    CHECK(!options.manual_gain);

    CHECK(parse_options((int)(sizeof(configured) / sizeof(configured[0])),
                        configured, &options) == 0);
    CHECK(!strcmp(options.host, "192.0.2.1"));
    CHECK(!strcmp(options.port, "4321"));
    CHECK(options.frequency == UINT64_C(2437000000));
    CHECK(options.device_index == 2);
    CHECK(options.packet_samples == 4096);
    CHECK(options.recovery_restarts == 0);
    CHECK(options.manual_gain);

    CHECK(parse_options(3, bad_port, &options) < 0);
    CHECK(parse_options(3, bad_packet, &options) < 0);
    return 0;
}

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
    struct udp_stream stream = {.socket_fd = -1, .packet_samples = 3};
    uint8_t received[64];
    int sockets[2];
    ssize_t length;

    CHECK(!socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets));
    CHECK(!setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)));
    stream.socket_fd = sockets[0];
    interrupted = 0;
    send_samples(samples, 5, 0, &stream);
    CHECK(!stream.error);
    CHECK(stream.samples_seen == 5);
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

int main(void)
{
    CHECK(!test_options());
    CHECK(!test_datagrams());
    puts("UDP option and packetization tests passed");
    return 0;
}
