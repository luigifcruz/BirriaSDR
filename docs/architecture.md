# Structure and extension points

For the receive path and hardware setup, see the [receiver overview](hardware.md)
and the [MT7921 device reference](mt7921.md).

| File | Responsibility |
| --- | --- |
| `include/birriasdr.h` | Public types, units, lifecycle, documented capabilities and reserved controls |
| `src/device.c` | Validation of API calls, requested settings, and synchronous and asynchronous entry points |
| `src/usb.c` | Descriptor discovery, board qualification, ownership and the libusb register transport |
| `src/io.c` | Endian helpers, monotonic deadlines and bounded register polling |
| `src/usb_stream.c` | Queued libusb transfers, the bounded host queue, cancellation and draining |
| `src/stream.c` | Acquisition lifecycle, callback batching, terminal checks and cleanup |
| `src/chips/mt7921_boot.c` | Original firmware download, ROM command framing and WFSYS reset |
| `src/chips/mt7921_rf.c` | Analog register access, clocks, bias and receive routing |
| `src/chips/mt7921_tune.c` | Frequency encoding, oscillator calibration, live retuning and rollback, and wider research sweeps |
| `src/chips/mt7921_gain.c` | Manual gain, field masks, readbacks and baseline restoration |
| `src/chips/mt7921_filter.c` | Matched analog and digital receive bandwidth, verified application and baseline restoration |
| `src/chips/mt7921_band.c` | 5 GHz and 6 GHz band profile, sub-band steps, and saving and restoring the fields it owns |
| `src/chips/mt7921_capture.c` | Capture engine (ICAP) configuration, mailbox controls and backend registration |
| `src/chips/mt7921_frame.c` | Parsing of original firmware events and conversion from raw14 to I/Q |
| `src/chips/mt7925.c` | Explicitly unimplemented backend with no register writes |
| `firmware/iq_stream.S` | Complete original device program and its constants |
| `tools/rebuild_firmware.py` | Optional rebuild and verification of the header from the assembly |

The common reader consumes the event and mailbox protocol of our original
firmware. Another backend can implement the same protocol on its own hardware.
A backend with a different sample format or mailbox protocol needs an explicit
parser and status adapter before it can share that reader. Never feed another
chipset's native descriptors into the MT7921 parser.

## Adding a chipset or board

1. Add a backend with its own identity, capabilities, default configuration,
   validation, and prepare, start, stop, status and reset operations. The
   MT7925 backend shows the deliberately unsupported starting point.
2. Add known USB IDs to the discovery table as **unqualified**. Being
   enumerated does not give a board permission to apply another board's RF
   register values.
3. Implement its transport differences, original firmware, receive setup and
   packet adaptation. Keep the analog die identity, board routing and crystal
   checks in the backend. The existing register transport in `usb.c` follows
   the convention of the MT76 USB family and is not a universal Wi-Fi protocol.
4. Test ordered register sequences, failure cleanup, framing, cancellation and
   raw sample interpretation without hardware. After that, validate RF on cold
   hardware and enable only qualified IDs and capabilities.
5. Implement reserved public controls only when their semantics are decoded and
   backed by evidence. Until then, return `BIRRIASDR_ERROR_UNSUPPORTED` and
   leave the capability unset. A qualified live gain backend supplies
   `change_gain` and sets `BIRRIASDR_CAP_LIVE_GAIN`, and the common layer
   serializes those calls. A tuning backend advertises `BIRRIASDR_CAP_TUNING`
   and provides its command bounds. If it also supports live changes, it
   supplies `change_frequency` and sets `BIRRIASDR_CAP_LIVE_TUNING`. A backend
   whose valid centers are not all reachable live, such as another band,
   supplies `validate_change`: its refusal returns before any hardware access
   and leaves the capture running, whereas a `change_frequency` error stops the
   acquisition. Document
   measured coverage separately from the command bounds, because a capability
   bit does not establish sensitivity.

Do not add guessed calibration tables, a silent fallback to vendor firmware, or
an implicit software resampler just to make an API request appear successful.

## Error and ownership rules

Every error is a public `BIRRIASDR_ERROR_*` value. Transport errors, hardware
timeouts, unexpected states, packet corruption and overruns stay
distinguishable. The final statistics record the failed stage and any reset
error separately. A successful cleanup never replaces the primary error.

A read owns its acquisition lifecycle until it returns, including any automatic
restarts. Cancellation crosses threads through an atomic run state. Internal
control failures use a separate stopping state, so recovery cannot clear an
explicit cancellation from the application. The center frequency and raw gain
setters and their getters may also run concurrently. A per-handle control mutex
serializes complete tuning and gain transactions with the prepare, start,
status, stop and reset operations. Locking individual USB transfers would not
protect a multi-step RF SPI update. The live gate opens after GO and closes on
stop, terminal status or cleanup. A failed live hardware update stops the
acquisition and preserves the error along with its RF or gain stage. Frequency
metadata is committed atomically, and only after a successful retune. Getters
do not attribute that metadata to older queued samples. The USB event thread
never calls application code or performs RF configuration. Its bounded queue
either preserves completed bytes under backpressure or reports overload for
strict readers. The design uses the
[libusb synchronous and event APIs](https://libusb.sourceforge.io/api-1.0/libusb_mtasync.html),
which coordinate their own event handling across threads. Cancellation drains
all submitted transfers before their memory is freed, as the
[libusb asynchronous I/O lifecycle](https://libusb.sourceforge.io/api-1.0/group__libusb__asyncio.html)
requires. Application callbacks must not close the handle. Instead, cancel,
wait for the read to return, join any control threads and then close. The
readback acknowledgment of a live control does not identify which
already-buffered I/Q sample it affected.

The `BS_TRY` macro is a small early-return helper for straight-line register
procedures. Functions that own resources use a single cleanup path instead.
Calibration, gain and filter restoration attempt every required cleanup write
even after an earlier failure. Unconditional register tables hold only ordered
address and value facts, while conditional logic and settling times stay in
named functions.

## Recovery ownership

When recovery is enabled, a gap callback is mandatory. The stream layer owns
cumulative delivered offsets and a bounded budget for device or transport faults.
Clean overruns neither spend nor replenish that budget. An overrun accompanied
by a reset, DMA or live-control failure still uses a bounded retry. Notification
numbers include all recoveries and saturate at the largest 32-bit value.

The device can recover source overruns without a new acquisition. It drains the
outstanding SRAM descriptor, stops capture, sends a gap marker from MCU memory,
waits for that marker to drain, and restarts capture at offset zero. Firmware,
USB, nonce, packet sequence and receiver settings stay live. The host retains
one raw packet until the next data event or a clean terminal status confirms
it. A marker discards that potentially overwritten packet and flushes the valid
prefix before the callback, so an application buffer never spans a gap.

The bounded USB queue preserves completed transfers when it fills. Completed
buffers are parked in arrival order and resubmitted only after their bytes have
been queued. Storage is bounded by the existing queue plus the allocated
transfers. Backpressure can overrun device SRAM, which is reported by the ordered
gap marker. Strict readers still fail on a full host queue.

A full restart creates a fresh parser and reader, checks a fresh firmware nonce,
and retires every asynchronous transfer before resetting hardware. The handle
stays busy through cleanup, callbacks, backoff and startup. Every gap callback
runs without the control mutex held and can cancel. Live setters remain usable
at a device-local gap but return busy during a full restart.

Separately, the chipset boot layer retries a transient preparation failure once,
before any reads are submitted or GO is released. It can repair the USB
transport with `libusb_reset_device` and then rerun the complete boot, RF, gain
and filter sequence. It never replays a partially transmitted command on its
own. Transport failures request this stronger reset for the next preparation.
A drained source overrun with successful cleanup first tries ordinary startup
without resetting USB enumeration.
Before sending new ROM commands, the transport layer drains stale bulk input
within both time and byte budgets. It never clears halts on healthy endpoints,
because doing so broke the tested board.

If a reset invalidates the USB handle, the library marks it permanently lost.
Further reads return `NO_DEVICE` without any register I/O, and close releases
only host resources. The library does not guess a replacement device from
enumeration order. A recovered read keeps the previous successful settings,
reports every restart it attempted, and returns its final error if recovery
cannot complete. Finite reads always fail rather than join separate
acquisitions.

## Tests and firmware

Normal CTest tests need neither hardware nor privileges. The RF trace fixture
was generated independently from the working Python prototype with
deterministic readbacks, not from the C code under test. Each of its 300 writes
must match, in order. The fixture covers the original bootstrap but not the
later matched-filter setup. A separate four-register model checks that setup's
analog and digital masks, preservation of unrelated bits, readback, failure at
every I/O boundary and retry after a failed rollback. The current offline suite
contains ten CTest cases. Additional fault tests cover standby readback failure,
a wrong die, gain rollback and repeated gain restoration. The live-control tests
exercise concurrent setters, coherent settings readback, startup gates and
failure-triggered cancellation. The gain register model also checks
manual-to-manual rollback and repeated release and force cycles against the
original unforced baseline.

The tuning tests check exact divider words, fractional carry, arithmetic bounds
and quantization error. By default, tuning uses all 20 fractional bits. The
register model checks that startup, live API updates and rollback preserve both
low bits. It also checks retune routing restoration, calibration cleanup, live
rollback and rejection of wider private probes while capture is active.
Public-range tests cover the exact inclusive endpoints and out-of-range
requests. These tests establish software behavior, not additional physical
tuning coverage.

The stream lifecycle test drives the public read engine with a simulated
transport and backend. It injects failures during allocation, preparation,
submission, start, stop, status, reset and live gain or tuning, then verifies
that the same handle can capture again. Terminal source faults are checked
before waiting for submitted packets that may never arrive. The transfer
ownership test also covers disconnects, queue overflow and partial allocation or
submission. On Linux, builds inject each reader allocation failure through
linker wrappers. The tuning arithmetic test round-trips every encoded word
across the command interval. These tests make no claim about reception between
measured frequencies. The recovery cases also check restart budgets,
cancellation inside the gap callback, failed rollback, fatal errors, stale
handles and preservation of the original control error when STOP also fails.
Separate tests exercise the real boot retry logic with a mocked transport, and
terminal packet accounting across the firmware's 32-bit counter wrap without
waiting for a multi-day capture. Gap tests cover fragmented markers, stale
nonces, counter wrap, malformed events, and rejection by strict readers. Stream
tests feed deliberately corrupted final packets before markers and verify their
discard, callback boundaries, cancellation, and multiple gaps without another
boot or reader. Queue tests cover parked-transfer ordering, teardown and failed
resubmission. Mixed-fault tests keep the bounded retry budget separate from
clean overruns.

The `test_hardware --run` command is an explicit physical-device test that is
not registered with CTest. It resets the dongle and exercises finite reads,
gain-mode changes, asynchronous callback order and cancellation. It requires the
same cold-start and USB ownership conditions as a real application.

The longer `test_stability --run` command uses public calls to sweep the full
command interval in 1 MHz steps, start and stop repeatedly, reopen the handle,
try every raw gain combination, interleave gain and tuning callers and hold a
five-minute stream. It is also excluded from CTest. Select individual phases
with `--run restart`, `--run live` or `--run soak`. The command stops at the
first failed phase and writes its status as JSON lines, without sample files.

The separate `test_recovery --run` command deliberately stalls a physical
acquisition. It pauses the receiver process to exhaust the queued USB reads and
cause a device source overrun. Separately, it stalls the application callback to
fill the host queue. Each failure is followed by a finite read at an endpoint or
the anchor, both on the original handle and after closing and reopening it. Its
receiver is forked before libusb is opened, and the parent only sends pause and
resume signals without ever touching USB. This test is also excluded from CTest.
Its `--run auto` mode additionally requires a gap callback followed by resumed
data within the same public read. The separate `test_control_recovery --run`
test injects host register-write errors during live gain and tuning changes and
their rollbacks. The Linux-only `test_startup_recovery --run` test deliberately
reproduces the transport fault caused by clearing a healthy halt, and requires
the real startup fallback to reset USB once and complete the requested finite
capture. None of these physical fault tests run through CTest.

Thanks to the checked-in image header, normal builds need neither an NDS32
toolchain nor Python. The optional `tools/rebuild_firmware.py --check` command
needs Python 3.10 or newer and GNU binutils targeting NDS32 to verify that the
source and image agree. It assembles only the included original assembly and
creates its PDA trailer. Keep the header in sync when changing the firmware, and
update the protocol tests if its ABI changes.
