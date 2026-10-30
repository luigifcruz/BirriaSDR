# UDP sender

The `birria_udp` example, in [birria_udp.c](../examples/birria_udp.c), streams
the receiver's samples as raw UDP datagrams. It builds with the other examples,
as the [build steps](../README.md#build-and-try) describe, and it needs the same
device setup as the recorder.

## Stream samples over UDP

The `birria_udp` program sends raw little-endian CS16 datagrams. It adds no
packet header, sequence number, timestamp or control data. Any application or
tool that reads raw interleaved signed 16-bit I/Q from a UDP port can receive
the stream.

### Configure the receiver

Configure the receiving application before you start `birria_udp`:

1. Select its UDP or network input.
2. Set it to listen on port `1234`, or the port you pass with `--port`.
3. Set the sample type to signed 16-bit complex, often labeled Int16 or CS16.
4. Set the sample rate to the one passed with `--rate`, 40 MSps (`40000000`) by default.
5. Start the receiver.

The sender does not connect to a server. The receiver binds its local UDP port
and waits for datagrams. Start the receiver first so the initial datagrams have
somewhere to go.

### Start the UDP sender

Start the sender from the repository root:

```sh
./build/birria_udp --frequency 2437000000 --manual
```

The default command uses device 0 and sends to `127.0.0.1:1234`. It places 2,048
complex samples in each datagram and permits three bounded recovery restarts.
The manual option selects the same LNA 2, filter stage 8 and digital 0 codes as
the recorder.

Set the receiving application's center frequency to the frequency passed to
`birria_udp`. This keeps its frequency labels aligned with the actual receiver
center.

### Command line options

Run the help command to see the accepted options:

```sh
./build/birria_udp --help
```

| Option | Default | Purpose and limits |
| --- | --- | --- |
| `--host HOST` | `127.0.0.1` | Selects the UDP destination address |
| `--port PORT` | `1234` | Selects a destination port from 1 to 65,535 |
| `--frequency HZ` | Device default | Selects the receiver center frequency in whole hertz, from 2156 to 2610 MHz (2190 to 2610 MHz on generic-ID boards) or 4500 to 6200 MHz. Another value fails with `UNSUPPORTED` |
| `--rate SPS` | `40000000` | Selects 10, 20, 40 or 80 MSps complex. The span equals the rate |
| `--device INDEX` | `0` | Selects a device from the `birria_rx --list` output |
| `--packet-samples COUNT` | `2048` | Selects 1 to 16,376 complex samples per datagram |
| `--recovery COUNT` | `3` | Selects the bounded full restart budget, and zero disables recovery |
| `--manual` | Disabled | Selects LNA 2, filter stage 8 and digital 0. These codes suit the 2.4 GHz band. The 5 GHz band needs LNA 4 or 5 for full sensitivity, which this option does not select |
| `--help` | Not applicable | Prints the option summary without opening hardware |

The following command selects device 1, changes the center frequency and sends
larger datagrams to a receiver on the local host:

```sh
./build/birria_udp \
    --device 1 \
    --frequency 2437123456 \
    --packet-samples 4096 \
    --manual
```

The following command sends to another host and disables automatic recovery:

```sh
./build/birria_udp \
    --host 192.0.2.20 \
    --port 1234 \
    --frequency 2437000000 \
    --recovery 0
```

## Stream format and bandwidth

| Property | Value |
| --- | --- |
| Transport | Connected UDP sender with nonblocking writes |
| Sample rate | 10, 20, 40 or 80 MSps complex, selected with `--rate`. The default is 40 MSps |
| Sample layout | Interleaved signed 16-bit I, Q pairs |
| Source range | −8192 to 8191 for each unshifted 14-bit component |
| Byte order | Little-endian |
| Datagram size | Exactly the `--packet-samples` count, except the last datagram before a gap or at shutdown, which can be shorter |
| Default payload | 2,048 complex samples, 8,192 bytes |
| Maximum payload | 16,376 complex samples, 65,504 bytes |
| Metadata | None |
| Return control | None |
| Payload rate | Four bytes per complex sample: 40 MB/s at 10 MSps, 80 MB/s at 20 MSps, 160 MB/s at 40 MSps and 320 MB/s at 80 MSps, before network overhead |

The default destination is the local host because a one gigabit network cannot
carry the payload at 40 or 80 MSps. At 10 or 20 MSps it fits, with 320 or 640
Mbit/s of payload. Remote streaming needs a faster link, enough sender and
receiver socket buffering and an MTU appropriate for the selected datagram size.
The default 8,192-byte payload requires IP fragmentation on a standard
1,500-byte Ethernet MTU.

The samples remain unshifted so the UDP output matches recorder files and
library callbacks. Their peak hardware range occupies about one quarter of the
signed 16-bit numeric range. A receiver that scales to the full 16-bit range can
therefore show a dBFS reference about 12 dB below a source that fills all 16
bits, but the transport does not discard sample bits.

## Frequency control

The raw stream has no return channel, so a receiving application cannot tune
the hardware. Changing its center frequency changes only its display and tuning
view, not the hardware receiver center.

Stop the sender with Ctrl-C and restart it with another frequency when you need
to retune:

```sh
./build/birria_udp --frequency 2450000000 --manual
```

The library supports live hardware tuning, but this raw UDP example has no
control channel or interactive command interface. For tuning and gain control
from an application, use the [SoapySDR module](soapy.md) instead.

## Recovery, loss and shutdown

The sender enables library recovery with a budget of three full restarts by
default. Before each gap the sender sends its partial datagram, so no datagram
spans a gap, and then prints a stream gap message. Clean source overruns can
also report gaps without consuming the bounded restart budget.

The UDP socket is nonblocking so a slow or absent receiver does not hold the
library callback indefinitely. The sender counts a datagram as dropped when its
local socket is full or the receiver is not listening yet, and it keeps
streaming. Any other send error, such as a lost route to the destination, stops
the stream with an error. The sender cannot detect datagrams lost after the
local kernel accepts them because the raw stream has no acknowledgements or
sequence numbers. The receiving application cannot identify the missing sample
count either.

Press Ctrl-C to stop at any time. During setup, the program finishes opening
the device, then closes it without streaming. Once streaming, the program
cancels the read, drains device packets that are already in flight, sends the
last partial datagram, closes the socket and device, then prints sample,
datagram, recovery and hardware status counters. The unsent count shows samples
lost when a send error stopped the stream. A nonzero exit status means capture,
network submission or cleanup reported an error.

## Troubleshooting

### The receiver shows no spectrum

Confirm that the receiver is listening before you start the sender. Check that
both programs use the same port, that the sample type is signed 16-bit complex
and that the sample rate matches `--rate`. Check the terminal for an open,
permission or device error.

### The frequency labels are wrong

Set the receiving application's center frequency to the same value passed
through `--frequency`. The datagrams do not carry the receiver frequency.

### The program reports dropped datagrams

Keep the destination on the local host first. Reduce other system load and
confirm that the receiving application keeps up with the selected rate. A lower
`--rate` reduces the load. A larger packet sample count reduces the datagram
rate, but larger datagrams increase fragmentation and loss risk on a physical
network.

### The program cannot open the dongle

Reconnect the dongle without a retained vendor patch, prevent the kernel Wi-Fi
driver from claiming it and verify USB permissions. The [main setup
instructions](../README.md#build-and-try) describe the supported host setup.

### The spectrum level looks low

Remember that the source preserves 14-bit values without shifting them into the
full signed 16-bit range. This affects the displayed full scale reference, not
the number of source bits delivered.

## Test the UDP packetizer

The offline UDP test checks option parsing, datagram boundaries and
little-endian CS16 encoding. It opens no hardware and needs no receiving
application.

```sh
ctest --test-dir build -R udp_packetizer --output-on-failure
```
