# Receiver overview

The library turns supported Wi-Fi hardware into a receive-only I/Q source, using
libusb for device access. Each chipset backend owns its boot, RF setup, gain and
capture procedures, while the common library handles USB transfers and the
application API. See [MT7921 / MT7961 / MT7972](mt7921.md) for the implemented
receiver and [architecture.md](architecture.md) for the source layout and
extension points.

## Hardware and software roles

The RF front end tunes and filters the input. The chip's capture engine writes
complex samples into internal memory, and our RAM-resident device program
submits that memory to USB DMA. The host checks the resulting stream and
delivers I/Q samples to the application. The implemented backend pairs a
nominal analog filter with a digital bandwidth class for each supported rate of
10, 20, 40 or 80 MSps complex. The 10 MSps rate uses the 20 MHz analog setting
with a narrower digital filter. The sampled span equals the rate, and the
[filter reference](mt7921.md#filtering) documents the measured rolloff and
out-of-span rejection. Sample packing, memory layout and register sequences
all belong to the chipset backend.

The device never receives vendor RAM firmware or a ROM patch. The current
backend relies only on the device's immutable factory ROM and our original
program. All configuration is volatile, and nothing is written to flash or
eFuse.

## Ownership and acquisition

The library requires exclusive use of the USB interface. A read prepares the
radio, queues host transfers and starts capture. On exit it drains the
transfers, restores the saved gain and filter fields, and resets the device.
When the backend advertises the matching capabilities, center frequency and raw
gain can also change during a read. Each complete register operation holds a
shared control lock, and sample transfers keep flowing while the analog
settings change. Retuning causes a signal and phase transient, and the library
cannot mark the exact sample where it happens. Other settings apply between
reads. The [public header](../include/birriasdr.h) documents buffer ownership,
units and error behavior.

Stream checks detect missing, repeated, stale or malformed packets as well as
device faults. Samples that were already delivered do not make a failed capture
complete. Overruns are reported instead of being hidden by dropping or inventing
samples. An application can enable automatic recovery for asynchronous reads by
providing a fault-restart budget and a mandatory gap callback. Source overruns
can recover inside the device with frequency and gain intact; clean overruns do
not consume the fault budget. Full restarts reapply the last committed settings.
Recovery cannot bring back lost samples, and finite reads remain strict.

Recognizing a chip does not mean its receiver is supported. Unknown controls
return unsupported, and board qualification is separate from USB
identification. Follow the build and device setup instructions in the
[README](../README.md#build-and-try) before using the library.
