/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 * Stream raw little-endian CS16 I/Q datagrams to any UDP receiver.
 */
#include "birriasdr.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netdb.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_HOST "127.0.0.1"
#define DEFAULT_PORT "1234"
#define DEFAULT_PACKET_SAMPLES 2048u
#define MAX_PACKET_SAMPLES 16376u
#define DEFAULT_RECOVERY_RESTARTS 3u

/* Written by the signal handler and read by the watcher and callback threads,
 * so it must be atomic. A lock-free atomic is also safe in a signal handler.
 */
static atomic_bool interrupted;
_Static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "the interrupt flag must be lock-free");

struct udp_stream {
    birriasdr_dev_t *dev;
    int socket_fd;
    uint32_t packet_samples;
    uint32_t pending_samples;                 /* samples waiting in pending */
    uint8_t pending[MAX_PACKET_SAMPLES * 4u]; /* little-endian datagram being filled */
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
    uint32_t rate;
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
            "  --rate SPS              Sample rate, 10, 20, 40 or 80 million (default 40000000)\n"
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
        if (!strcmp(argv[i], "--rate")) {
            if (take_value(argc, argv, &i, &value) ||
                parse_u64(value, UINT32_MAX, false, &number))
                return -1;
            options->rate = (uint32_t)number;
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

/* Errors that lose one datagram but leave the socket usable for the next: a
 * full buffer, or a receiver that is not listening yet. A lost route to the
 * destination is not one of them, because no later datagram would arrive.
 */
static bool droppable_socket_error(int error)
{
    return error == EINTR || error == EAGAIN || error == EWOULDBLOCK || error == ENOBUFS ||
           error == ECONNREFUSED;
}

/* Send the pending datagram, if any. A droppable error counts it as dropped.
 * Any other error, or a short send, fails the stream.
 */
static int send_pending(struct udp_stream *stream)
{
    uint32_t count = stream->pending_samples;
    size_t length = (size_t)count * 4u;
    ssize_t sent;

    if (!count)
        return 0;
    stream->pending_samples = 0;
    sent = send(stream->socket_fd, stream->pending, length, 0);
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

/* Encode samples as little-endian I,Q into datagrams of exactly packet_samples,
 * carrying a partial datagram over to the next callback. Datagram boundaries
 * carry no metadata. Receivers treat each payload as raw CS16.
 */
static void send_samples(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct udp_stream *stream = context;

    (void)first;
    stream->samples_seen += count;
    for (uint32_t i = 0; i < count && !stream->error; ++i) {
        uint8_t *bytes = stream->pending + (size_t)stream->pending_samples * 4u;
        uint16_t in_phase = (uint16_t)iq[2u * i];
        uint16_t quadrature = (uint16_t)iq[2u * i + 1u];
        bytes[0] = (uint8_t)in_phase;
        bytes[1] = (uint8_t)(in_phase >> 8);
        bytes[2] = (uint8_t)quadrature;
        bytes[3] = (uint8_t)(quadrature >> 8);
        if (++stream->pending_samples == stream->packet_samples && send_pending(stream))
            stream->error = 1;
    }
    if (atomic_load(&interrupted) || stream->error)
        (void)birriasdr_cancel_async(stream->dev);
}

/* Flush the partial datagram so none spans the gap, then report the gap. */
static void on_recovery(const struct birriasdr_recovery_event *event, void *context)
{
    struct udp_stream *stream = context;

    if (!stream->error && send_pending(stream))
        stream->error = 1;
    if (stream->error)
        (void)birriasdr_cancel_async(stream->dev);
    stream->gaps++;
    fprintf(stderr, "Stream gap before delivered sample %" PRIu64
                    ", attempt %u: %s\n",
            event->next_sample, event->attempt, birriasdr_strerror(event->error));
    if (atomic_load(&interrupted))
        (void)birriasdr_cancel_async(stream->dev);
}

/* Signal handlers only set a flag; library cancellation runs in normal context. */
static void on_signal(int signal_number)
{
    (void)signal_number;
    atomic_store(&interrupted, true);
}

struct interrupt_watch {
    birriasdr_dev_t *dev;
    atomic_bool done;
};

/* Cancel from a normal thread once the user interrupts, so Ctrl-C works even
 * before the first callback. Cancellation before the read starts returns STATE,
 * and repeating it is harmless, so retry until the read has returned.
 */
static void *watch_interrupt(void *context)
{
    struct interrupt_watch *watch = context;
    const struct timespec period = {.tv_nsec = 50000000};

    while (!atomic_load(&watch->done)) {
        if (atomic_load(&interrupted))
            (void)birriasdr_cancel_async(watch->dev);
        nanosleep(&period, NULL);
    }
    return NULL;
}

/* Open the device, apply the options and stream until Ctrl-C or an error, then
 * print the counters. A nonzero exit status means the stream did not end cleanly.
 */
int main(int argc, char **argv)
{
    struct options options;
    struct udp_stream stream = {.socket_fd = -1};
    struct interrupt_watch watch;
    pthread_t watcher;
    bool streaming = false;
    uint64_t unsent;
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

    /* Install the handlers before setup, so Ctrl-C while the device opens leads
     * to a clean close instead of killing the process mid-boot.
     */
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
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

    if (options.rate) {
        result = birriasdr_set_sample_rate(dev, options.rate);
        if (result < 0)
            fprintf(stderr, "Rate: %s\n", birriasdr_strerror(result));
    }
    if (!result && options.frequency) {
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

    /* Ctrl-C during setup skips the stream; the device still closes cleanly. */
    if (!result && !atomic_load(&interrupted)) {
        watch.dev = dev;
        atomic_init(&watch.done, false);
        if (pthread_create(&watcher, NULL, watch_interrupt, &watch)) {
            /* Without the watcher, Ctrl-C could not stop a read that stalls before
             * its first callback, so do not start one.
             */
            fprintf(stderr, "Interrupt watcher: cannot start a thread\n");
            result = BIRRIASDR_ERROR_NO_MEMORY;
        } else {
            streaming = true;
        }
    }
    if (streaming) {
        fprintf(stderr,
                "Sending %" PRIu32 " MSps CS16 to %s:%s in packets of %u samples."
                " Press Ctrl-C to stop.\n",
                birriasdr_get_sample_rate(dev) / 1000000u, options.host, options.port,
                options.packet_samples);
        result = birriasdr_read_async(dev, send_samples, &stream, 0, 0);
        atomic_store(&watch.done, true);
        pthread_join(watcher, NULL);
        /* Callbacks have finished, so the last partial datagram is safe to send. */
        if (!stream.error && send_pending(&stream))
            stream.error = 1;
    }

    if (birriasdr_get_stream_stats(dev, &stats) < 0)
        memset(&stats, 0, sizeof(stats));
    close_result = birriasdr_close(dev);
    close(stream.socket_fd);

    /* Samples seen but neither sent nor dropped were lost to a send error. */
    unsent = stream.samples_seen - stream.samples_sent - stream.samples_dropped;
    fprintf(stderr,
            "Saw %" PRIu64 " samples, sent %" PRIu64 " in %" PRIu64
            " datagrams, dropped %" PRIu64 " in %" PRIu64 " datagrams, unsent %" PRIu64
            ", gaps=%u; stream=%s; firmware state=%u; stage=%d; reset=%s\n",
            stream.samples_seen, stream.samples_sent, stream.datagrams_sent,
            stream.samples_dropped, stream.datagrams_dropped, unsent, stream.gaps,
            birriasdr_strerror(result), stats.firmware_state, stats.stage,
            birriasdr_strerror(stats.reset_error));
    return result || close_result || stream.error ? 1 : 0;
}
