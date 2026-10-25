# SoapySDR module

The `soapy/` directory holds a SoapySDR module (`driver=birria`) so any
SoapySDR application can use a BirriaSDR receiver. The module wraps the
library's RX path. It runs the dongle at its fixed 40 MSps and decimates to
the requested rate with a host-side filter.

## What the module provides

| Item | Value |
| --- | --- |
| Driver key | `driver=birria`. A dongle is selected by enumeration order with `index=N` |
| Sample rates | 40 MSps divided by 1, 2, 4, 5, 8, 10, 16, 20 or 40, giving 40 MSps down to 1 MSps. The dongle always runs at 40 MSps and the module decimates with a host-side filter |
| Stream formats | `CS16` (native) and `CF32` |
| Frequency ranges | One range per library band, from `birriasdr_get_tuning_ranges()`. Currently 2156 to 2610 MHz (2190 MHz floor on `0e8d:7961`), 5150 to 5925 MHz and 5925 to 7125 MHz |
| Gains | `LNA` 0 to 3, `LPF` 0 to 8 and `DIG` 0 to 12. These are raw codes, not dB |
| Gain mode | Automatic (the library default) or manual. Gain codes apply only in manual mode. In automatic mode they are stored and sent once manual mode is selected |
| Band overrides | Device arguments `bandN_min` and `bandN_max` (Hz) replace one band's bounds through `birriasdr_set_tuning_range()`, for example `band0_min=2169e6`. A missing value keeps the board's value |

Errors from tuning, sample-rate changes and gain changes are logged, not
thrown, because some front ends do not catch exceptions from these calls and
would abort. A rejected frequency leaves the receiver where it was.

Live retuning cannot cross bands. To move between 2.4 GHz, 5 GHz and 6 GHz,
stop the stream, retune, then start it again.

## Build

The module builds as part of the normal BirriaSDR build whenever SoapySDR is
installed. No extra flag is needed:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The build links the module against the in-tree `libbirriasdr`, so no separate
library install is required. The module is written to `build/soapy/`. If
SoapySDR is not installed, CMake prints a note and skips the module. Pass
`-DBIRRIASDR_SOAPY=OFF` to skip it on purpose.

Check that SoapySDR finds the dongle:

```
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy SoapySDRUtil --find=driver=birria
```

Every command below sets `SOAPY_SDR_PLUGIN_PATH`. To avoid that, install the
module into the system module directory:

```
sudo cmake --install build
```

## USB access without root

The `tools/99-birriasdr.rules` file lets the `plugdev` group open both
supported IDs. Copy it into place, reload the rules and replug the dongle:

```
sudo cp tools/99-birriasdr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The `mt7921u` kernel driver must not be bound to the dongle. A run under
`sudo` can leave root-owned lock files in `/tmp`, and a later unprivileged run
cannot open the device until those are removed. Only one program can open the
dongle at a time, so stop one client before you start another.

## CubicSDR

CubicSDR links SoapySDR directly, so it loads the module with no rebuild when
the versions match. On Debian the packaged `cubicsdr` links the system
SoapySDR, the same version the module builds against.

```
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy CubicSDR
```

In the device dialog, pick the MediaTek entry and press Start. Turn AGC off to
use the `LNA`, `LPF` and `DIG` sliders. A reasonable start is `LNA` 2, `LPF` 8
and `DIG` 0, and `LNA` 3 at 6 GHz. CubicSDR cannot demodulate Wi-Fi, so treat
it as a spectrum and waterfall view. Stop playback before you change bands,
since live retuning cannot cross between 2.4 GHz, 5 GHz and 6 GHz.

## SDR++ server and a remote client

An SDR++ server can own the dongle on a desktop and stream to SDR++ clients,
including the Android app. The server needs the SoapySDR source module, which
some distributions omit from their package. Where it is missing, build SDR++
from source with `-DOPT_BUILD_SOAPY_SOURCE=ON` and follow the project's build
instructions.

Add a SoapySDR source instance to `~/.config/sdrpp/config.json` and select it
as the source:

```json
"moduleInstances": {
    "SoapySDR Source": { "module": "soapy_source", "enabled": true }
},
"source": "SoapySDR"
```

Start the server with the plugin path set:

```
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy sdrpp --server --addr 0.0.0.0 --port 5259
```

From a client, choose the SDR++ server source, enter the desktop address and
port 5259, and connect. In the server's source menu pick the birria device and
a sample rate. The sample rate sets the waterfall span. A good start is 5 MSps,
and 10 or 20 MSps work when the network link keeps up. For a remote client, set
the sample type to int8 and turn compression on to reduce bandwidth. Enter
frequencies in Hz, for example `2437000000` for Wi-Fi channel 6, and use mode
RAW because SDR++ cannot demodulate Wi-Fi. Good test points are 2437 MHz on
2.4 GHz, 5220 MHz on 5 GHz and 6135 MHz on 6 GHz, where access points were
received during the library's qualification. Stop playback before you change
bands.

## Manual stream test

The `soapy/tests/test_stream.cpp` program captures through the module and
reports the averaged spectrum peak, to check decimation and the frequency axis.
CMake does not build it:

```
g++ -O2 -o build/test_stream soapy/tests/test_stream.cpp -lSoapySDR
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy build/test_stream <freq_hz> <rate> <seconds> [out.cf32]
```

To reproduce an application's problem, replay its call sequence against the
module in the same order: gain mode, gains, frequency, sample rate, then the
stream.
