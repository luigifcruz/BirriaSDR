# Supported devices

The library recognizes several USB IDs but enables only the boards that were
tested on hardware. This page lists each board, what works on it and how to
add another one. It answers the question of how a dongle becomes qualified.

## Supported boards

| Device | USB ID | 2.4 GHz receive | 5 GHz receive | 6 GHz receive | Radio bands | Tuning range | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| COMFAST MT7961/MT7972 | `3574:6211` | ✅ | ❌ | ❌ | Dual band | 2156 to 2610 MHz, nominal | Qualified |
| EDUP EP-AX1672 (AX3000M, MT7921AU) | `0e8d:7961` | ✅ | ✅ | ✅ | Tri-band 802.11ax | 2169 to 2610 MHz, 5150 to 5924 MHz and 5925 to 7125 MHz, measured | Proposed |

The legend for the receive columns follows.

| Mark | Meaning |
| --- | --- |
| ✅ | Working and measured on the listed board |
| ❌ | Not implemented in the library, whatever the radio can do |
| ? | Implemented but untested on the listed board |

Every measurement on this page comes from the board named in its row. The
EDUP row is proposed until the qualification change that enables it is merged.
Its 5 GHz and 6 GHz marks depend on a separate change that adds the high-band
tuning path, so they hold only once that change is merged too. On the tested
EDUP unit, signals were received from 5220 to 5785 MHz and from 6135 to 6855 MHz.
Tuning, calibration and streaming succeeded across the whole listed range.
The library's `get_tuning_range()` still reports only 2156 to 2610 MHz. The
COMFAST board was not tested on 5 GHz or 6 GHz, so its marks stay ❌.

## Recognized but unsupported

These IDs appear in the device table in `src/usb.c`, but the library refuses to
open them.

| Device | USB ID | Reason |
| --- | --- | --- |
| MediaTek MT7925 | `0e8d:7925` | The MT7925 backend is not implemented |
| Netgear A8500 | `0846:9050` | The MT7925 backend is not implemented |
| Netgear A9000 | `0846:9072` | The MT7925 backend is not implemented |

## What qualified means

A qualified board passed the checks below on at least one physical unit, and
its entry in `src/usb.c` has the `qualified` flag set. Qualification covers one
board model and one receive path. It does not mean the board is calibrated.
Sensitivity, frequency accuracy and gain stay unmeasured unless a row says
otherwise.

A board can be qualified with limits. A narrower tuning range, for example, is
listed in the matrix and described in the board's evidence page.

## Preparing the unit

Follow [Build and try](../README.md#build-and-try) first. The checks fail on
any board that still has a kernel Wi-Fi driver attached or a vendor ROM patch
loaded, so reconnect the dongle after you stop the driver from binding. Run the
commands with `sudo` unless you have set up USB access for your user.

## Required checks

Run each check on the board you want to list and keep the output.

1. Run `ctest --test-dir build --output-on-failure`. The offline tests need no
   hardware.
2. Run `./build/birria_rx --list` and confirm that the board appears with its
   USB ID.
3. Record eight million samples with
   `./build/birria_rx capture.cs16 8000000 manual` and check the exit status.
4. Plot or inspect the spectrum and confirm that a known signal appears at the
   expected frequency, such as a nearby access point on channel 6.
5. Capture 65,536 samples at several center frequencies across the tuning range
   with `--frequency`. If a frequency fails, bisect to the lowest and highest
   frequency that pass.
6. Run `./build/test_stability --run`, or its `restart`, `live` and `soak`
   phases one at a time, then `./build/test_recovery --run` and
   `./build/test_control_recovery --run`.

A failed check is still useful. Record it, and the board can be listed with the
limit it exposed.

## Adding a device

1. Open an issue with the output of `lsusb -v -d <vid>:<pid>` and a link to the
   product, so a maintainer can see whether the ID is shared with other boards.
2. Add or edit the entry in the device table in `src/usb.c`. Keep the
   `qualified` flag false in the proposal until the maintainers agree on it.
3. Run the required checks with the flag set locally.
4. Write `docs/qualification-<vid>-<pid>.md` with the board, the host, the
   kernel version, the USB port speed and the result of each check. Name the
   tested board next to every measurement.
5. Add a row to the matrix above. Use ❌ for any band the library does not
   implement.
6. Open a pull request that links the issue and the evidence page.

Generic IDs need extra care. MediaTek reuses `0e8d:7961` across many dongles,
so a pull request for such an ID should say how the tested unit differs from
the others and propose a narrower match, such as a USB string or the chip ID
that the backend reads.

## Reporting a board that does not work

Open an issue with the board, its USB ID, the `lsusb -v` output and the exact
line that `birria_rx` prints on failure. The stage and firmware state in that
line show how far initialization got.
