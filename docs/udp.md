# UDP sender

The `birria_udp` example, in [birria_udp.c](../examples/birria_udp.c), streams
the receiver's samples as raw UDP datagrams. It builds with the other examples,
as the [build steps](../README.md#build-and-try) describe, and it needs the same
device setup as the recorder.

## Stream samples to SDR++

The `birria_udp` program sends raw little-endian CS16 datagrams. It adds no
packet header, sequence number, timestamp or control data. The payload can feed
the Network source that ships with SDR++.

### Configure SDR++

Configure the source before you start `birria_udp`:

1. Select **Network** in the SDR++ source list.
2. Set **Protocol** to **UDP**.
3. Set **Host** to `127.0.0.1`.
4. Set **Port** to `1234`.
5. Set **Sample type** to **Int16**.
6. Set **Samplerate** to `40000000`, then apply the value.
7. Start the SDR++ source.

The host field does not make SDR++ connect to a server. The Network source binds
its local UDP port and waits for datagrams. Start SDR++ first so the initial
datagrams have a receiver.

### Start the UDP sender

Start the sender from the repository root:

```sh
./build/birria_udp --frequency 2437000000 --manual
```

The default command uses device 0 and sends to `127.0.0.1:1234`. It places 2,048
complex samples in each datagram and permits three bounded recovery restarts.
The manual option selects the same LNA 2, filter stage 8 and digital 0 codes as
the recorder.

Keep the SDR++ displayed center frequency equal to the frequency passed to
`birria_udp`. This keeps the frequency labels aligned with the actual receiver
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
| `--frequency HZ` | Device default | Selects the receiver center frequency in whole hertz |
| `--device INDEX` | `0` | Selects a device from the `birria_rx --list` output |
| `--packet-samples COUNT` | `2048` | Selects 1 to 16,376 complex samples per datagram |
| `--recovery COUNT` | `3` | Selects the bounded full restart budget, and zero disables recovery |
| `--manual` | Disabled | Selects LNA 2, filter stage 8 and digital 0 |
| `--help` | Not applicable | Prints the option summary without opening hardware |

The following command selects device 1, changes the center frequency and sends
larger datagrams to a local SDR++ instance:

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
| Sample rate | 40 MSps complex, fixed |
| Sample layout | Interleaved signed 16-bit I, Q pairs |
| Source range | −8192 to 8191 for each unshifted 14-bit component |
| Byte order | Little-endian |
| Default payload | 2,048 complex samples, 8,192 bytes |
| Maximum payload | 16,376 complex samples, 65,504 bytes |
| Metadata | None |
| Return control | None |
| Payload rate | 160 MB/s, or 1.28 Gbit/s before network overhead |

The default destination is the local host because a one gigabit network cannot
carry the sample payload. Remote streaming needs a faster link, enough sender
and receiver socket buffering and an MTU appropriate for the selected datagram
size. The default 8,192-byte payload requires IP fragmentation on a standard
1,500-byte Ethernet MTU.

The samples remain unshifted so the UDP output matches recorder files and
library callbacks. Their peak hardware range occupies about one quarter of the
signed 16-bit numeric range. The SDR++ application can therefore show a dBFS
reference about 12 dB below a source that fills all 16 bits, but the transport
does not discard sample bits.

## Frequency control

The SDR++ Network source does not send its tuning changes back to `birria_udp`.
Changing the SDR++ center frequency changes the display and tuning view, but it
does not change the hardware receiver center.

Stop the sender with Ctrl-C and restart it with another frequency when you need
to retune:

```sh
./build/birria_udp --frequency 2450000000 --manual
```

The library supports live hardware tuning, but this raw UDP example has no
return control channel or interactive command interface. A native SDR++ source
module or a separate control protocol would be required for direct tuning and
gain control from the SDR++ interface.

## Recovery, loss and shutdown

The sender enables library recovery with a budget of three full restarts by
default. A recovery event prints a stream gap message before resumed samples.
Clean source overruns can also report gaps without consuming the bounded restart
budget.

The UDP socket is nonblocking so a slow or absent receiver does not hold the
library callback indefinitely. The sender counts datagrams rejected by its local
socket as dropped. It cannot detect datagrams lost after the local kernel
accepts them because the raw stream has no acknowledgements or sequence numbers.
The SDR++ application also cannot identify the missing sample count.

Press Ctrl-C to stop. The program cancels the read, drains device packets that
are already in flight, closes the socket and device, then prints sample,
datagram, recovery and hardware status counters. A nonzero exit status means
capture, network submission or cleanup reported an error.

## Troubleshooting

### SDR++ shows no spectrum

Confirm that SDR++ is running the Network source before you start the sender.
Check that both programs use the same port, that the sample type is Int16 and
that the applied sample rate is `40000000`. Check the terminal for an open,
permission or device error.

### The frequency labels are wrong

Set the SDR++ displayed center to the same value passed through `--frequency`.
The Network source cannot read the receiver frequency from the datagrams.

### The program reports dropped datagrams

Keep the destination on the local host first. Reduce other system load and
confirm that SDR++ can process a 40 MSps source. A larger packet sample count
reduces packet and callback frequency, but larger datagrams increase
fragmentation and loss risk on a physical network.

### The program cannot open the dongle

Reconnect the dongle without a retained vendor patch, prevent the kernel Wi-Fi
driver from claiming it and verify USB permissions. The [main setup
instructions](../README.md#build-and-try) describe the supported host setup.

### The spectrum level looks low

Remember that the source preserves 14-bit values without shifting them into the
full signed 16-bit range. This affects the displayed full scale reference, not
the number of source bits delivered.

## Gqrx compatibility

The raw UDP format in `birria_udp` targets the SDR++ Network source. This
example does not provide a direct Gqrx input device. A Gqrx path needs a
compatible SoapySDR, gr-osmosdr or `rtl_tcp` adapter instead of the current raw
UDP receiver setup.

## Test the UDP packetizer

The offline UDP test checks option parsing, datagram boundaries and
little-endian CS16 encoding. It does not open hardware or run SDR++.

```sh
ctest --test-dir build -R udp_packetizer --output-on-failure
```
