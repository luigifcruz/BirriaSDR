/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 * Stream raw little-endian CS16 I/Q datagrams to SDR++ Network Source.
 */
#include "birriasdr.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netdb.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define DEFAULT_HOST "127.0.0.1"
#define DEFAULT_PORT "1234"
#define DEFAULT_PACKET_SAMPLES 2048u
#define MAX_PACKET_SAMPLES 16376u
#define DEFAULT_RECOVERY_RESTARTS 3u

static volatile sig_atomic_t interrupted;

struct udp_stream {
    birriasdr_dev_t *dev;
    int socket_fd;
    uint32_t packet_samples;
    uint64_t samples_seen;
    uint64_t samples_sent;
    uint64_t datagrams_sent;
    uint64_t samples_dropped;
    uint64_t datagrams_dropped;
    uint32_t gaps;
    int error;
};

struct options {
    const char *host;
    const char *port;
    uint64_t frequency;
    uint32_t device_index;
    uint32_t packet_samples;
    uint32_t recovery_restarts;
    bool manual_gain;
};

/* Print the option summary. Help goes to stdout and usage errors to stderr. */
static void usage(FILE *file, const char *program)
{
    fprintf(file,
            "Usage: %s [options]\n"
            "  --host HOST             UDP destination (default " DEFAULT_HOST ")\n"
            "  --port PORT             UDP destination port (default " DEFAULT_PORT ")\n"
            "  --frequency HZ          Center frequency for the receiver\n"
            "  --device INDEX          Device index (default 0)\n"
            "  --packet-samples COUNT  Complex samples per datagram (default 2048, max 16376)\n"
            "  --recovery COUNT        Full stream restart budget (default 3, 0 disables)\n"
            "  --manual                Use gain codes LNA 2, filter 8, digital 0\n"
            "  --help                  Show this help\n",
            program);
}

/* Parse an unsigned decimal without accepting signs, wrap or ignored suffixes. */
static int parse_u64(const char *text, uint64_t maximum, bool allow_zero, uint64_t *value)
{
    char *end;
    unsigned long long number;

    if (!text || *text < '0' || *text > '9')
        return -1;
    errno = 0;
    number = strtoull(text, &end, 10);
    if (errno || *end || (!allow_zero && !number) || number > maximum)
        return -1;
    *value = (uint64_t)number;
    return 0;
}

/* Take the value that follows an option, failing when the option is last. */
static int take_value(int argc, char **argv, int *argument, const char **value)
{
    if (*argument + 1 >= argc)
        return -1;
    *value = argv[++*argument];
    return 0;
}

/* Fill options from the command line. Returns 1 for help and -1 for a usage error. */
static int parse_options(int argc, char **argv, struct options *options)
{
    const char *value;
    uint64_t number;

    *options = (struct options){
        .host = DEFAULT_HOST,
        .port = DEFAULT_PORT,
        .packet_samples = DEFAULT_PACKET_SAMPLES,
        .recovery_restarts = DEFAULT_RECOVERY_RESTARTS
    };
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help"))
            return 1;
        if (!strcmp(argv[i], "--manual")) {
            options->manual_gain = true;
            continue;
        }
        if (!strcmp(argv[i], "--host")) {
            if (take_value(argc, argv, &i, &options->host))
                return -1;
            continue;
        }
        if (!strcmp(argv[i], "--port")) {
            if (take_value(argc, argv, &i, &options->port) ||
                parse_u64(options->port, UINT16_MAX, false, &number))
                return -1;
            continue;
        }
        if (!strcmp(argv[i], "--frequency")) {
            if (take_value(argc, argv, &i, &value) ||
                parse_u64(value, UINT64_MAX, false, &options->frequency))
                return -1;
            continue;
        }
        if (!strcmp(argv[i], "--device")) {
            if (take_value(argc, argv, &i, &value) ||
                parse_u64(value, UINT32_MAX, true, &number))
                return -1;
            options->device_index = (uint32_t)number;
            continue;
        }
        if (!strcmp(argv[i], "--packet-samples")) {
            if (take_value(argc, argv, &i, &value) ||
                parse_u64(value, MAX_PACKET_SAMPLES, false, &number))
                return -1;
            options->packet_samples = (uint32_t)number;
            continue;
        }
        if (!strcmp(argv[i], "--recovery")) {
            if (take_value(argc, argv, &i, &value) ||
                parse_u64(value, UINT32_MAX, true, &number))
                return -1;
            options->recovery_restarts = (uint32_t)number;
            continue;
        }
        return -1;
    }
    return 0;
}

/* Connect fixes one destination and lets send() report asynchronous UDP errors. */
static int open_udp_sender(const char *host, const char *port)
{
    const struct addrinfo hints = {
        .ai_family = AF_UNSPEC,
        .ai_socktype = SOCK_DGRAM,
        .ai_protocol = IPPROTO_UDP
    };
    struct addrinfo *addresses = NULL;
    struct addrinfo *address;
    int fd = -1;
    int status = getaddrinfo(host, port, &hints, &addresses);

    if (status) {
        fprintf(stderr, "Destination: %s\n", gai_strerror(status));
        return -1;
    }
    for (address = addresses; address; address = address->ai_next) {
        fd = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (fd < 0)
            continue;
        if (!connect(fd, address->ai_addr, address->ai_addrlen))
            break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(addresses);
    if (fd < 0) {
        perror("Destination");
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("Destination nonblocking mode");
        close(fd);
        return -1;
    }

    /* The kernel can cap this request. Failure is not fatal because UDP remains usable. */
    int send_buffer = 4 * 1024 * 1024;
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer));
    return fd;
}

/* Errors that lose one datagram but leave the socket usable for the next. */
static bool droppable_socket_error(int error)
{
    return error == EINTR || error == EAGAIN || error == EWOULDBLOCK || error == ENOBUFS ||
           error == ECONNREFUSED || error == EHOSTUNREACH || error == ENETUNREACH;
}

/* Send one datagram of little-endian samples. A droppable error counts the
 * datagram as dropped. Any other error, or a short send, fails the stream.
 */
static int send_datagram(struct udp_stream *stream, const int16_t *iq, uint32_t count)
{
    size_t length = (size_t)count * 2u * sizeof(*iq);
    const void *data = iq;
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    uint8_t little_endian[MAX_PACKET_SAMPLES * 4u];
    for (uint32_t i = 0; i < count * 2u; ++i) {
        uint16_t value = (uint16_t)iq[i];
        little_endian[2u * i] = (uint8_t)value;
        little_endian[2u * i + 1u] = (uint8_t)(value >> 8);
    }
    data = little_endian;
#endif

    ssize_t sent = send(stream->socket_fd, data, length, 0);
    if (sent == (ssize_t)length) {
        stream->samples_sent += count;
        stream->datagrams_sent++;
        return 0;
    }
    if (sent < 0 && droppable_socket_error(errno)) {
        stream->samples_dropped += count;
        stream->datagrams_dropped++;
        return 0;
    }
    if (sent < 0)
        perror("UDP send");
    else
        fprintf(stderr, "UDP send: short datagram (%zd of %zu bytes)\n", sent, length);
    return -1;
}

/* Datagram boundaries carry no metadata. SDR++ treats each payload as raw CS16. */
static void send_samples(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct udp_stream *stream = context;
    uint32_t done = 0;

    (void)first;
    stream->samples_seen += count;
    while (done < count && !stream->error) {
        uint32_t chunk = count - done;
        if (chunk > stream->packet_samples)
            chunk = stream->packet_samples;
        if (send_datagram(stream, iq + (size_t)done * 2u, chunk))
            stream->error = 1;
        done += chunk;
    }
    if (interrupted || stream->error)
        (void)birriasdr_cancel_async(stream->dev);
}

/* Report each library gap, and keep cancelling once the user has interrupted. */
static void on_recovery(const struct birriasdr_recovery_event *event, void *context)
{
    struct udp_stream *stream = context;

    stream->gaps++;
    fprintf(stderr, "Stream gap before delivered sample %" PRIu64
                    ", attempt %u: %s\n",
            event->next_sample, event->attempt, birriasdr_strerror(event->error));
    if (interrupted)
        (void)birriasdr_cancel_async(stream->dev);
}

/* Signal handlers only set a flag; library cancellation runs in normal context. */
static void on_signal(int signal_number)
{
    (void)signal_number;
    interrupted = 1;
}

/* Open the device, apply the options and stream until Ctrl-C or an error, then
 * print the counters. A nonzero exit status means the stream did not end cleanly.
 */
int main(int argc, char **argv)
{
    struct options options;
    struct udp_stream stream = {.socket_fd = -1};
    struct birriasdr_stream_stats stats = {0};
    birriasdr_dev_t *dev = NULL;
    int result;
    int close_result = 0;
    int parsed = parse_options(argc, argv, &options);

    if (parsed > 0) {
        usage(stdout, argv[0]);
        return 0;
    }
    if (parsed < 0) {
        usage(stderr, argv[0]);
        return 2;
    }

    stream.socket_fd = open_udp_sender(options.host, options.port);
    if (stream.socket_fd < 0)
        return 1;
    stream.packet_samples = options.packet_samples;

    result = birriasdr_open(&dev, options.device_index);
    if (result < 0) {
        fprintf(stderr, "Open: %s\n", birriasdr_strerror(result));
        close(stream.socket_fd);
        return 1;
    }
    stream.dev = dev;

    if (options.frequency) {
        result = birriasdr_set_center_freq(dev, options.frequency);
        if (result < 0)
            fprintf(stderr, "Frequency: %s\n", birriasdr_strerror(result));
    }
    if (!result && options.manual_gain) {
        const struct birriasdr_gain gain = {2, 8, 0};
        result = birriasdr_set_tuner_gain_mode(dev, 1);
        if (!result)
            result = birriasdr_set_gain_codes(dev, &gain);
        if (result < 0)
            fprintf(stderr, "Gain: %s\n", birriasdr_strerror(result));
    }
    if (!result && options.recovery_restarts) {
        result = birriasdr_set_recovery(dev, options.recovery_restarts,
                                        on_recovery, &stream);
        if (result < 0)
            fprintf(stderr, "Recovery: %s\n", birriasdr_strerror(result));
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (!result) {
        fprintf(stderr,
                "Sending 40 MSps CS16 to %s:%s in packets of %u samples. Press Ctrl-C to stop.\n",
                options.host, options.port, options.packet_samples);
        result = birriasdr_read_async(dev, send_samples, &stream, 0, 0);
    }

    if (birriasdr_get_stream_stats(dev, &stats) < 0)
        memset(&stats, 0, sizeof(stats));
    close_result = birriasdr_close(dev);
    close(stream.socket_fd);

    fprintf(stderr,
            "Saw %" PRIu64 " samples, sent %" PRIu64 " in %" PRIu64
            " datagrams, dropped %" PRIu64 " in %" PRIu64
            " datagrams, gaps=%u; stream=%s; firmware state=%u; stage=%d; reset=%s\n",
            stream.samples_seen, stream.samples_sent, stream.datagrams_sent,
            stream.samples_dropped, stream.datagrams_dropped, stream.gaps,
            birriasdr_strerror(result), stats.firmware_state, stats.stage,
            birriasdr_strerror(stats.reset_error));
    return result || close_result || stream.error ? 1 : 0;
}
