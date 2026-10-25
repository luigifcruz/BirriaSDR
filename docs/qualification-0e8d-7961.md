# Qualification: EDUP AX3000M (MT7921AU), USB `0e8d:7961`

This page records tests of one board that uses MediaTek's generic USB ID
`0e8d:7961`. The tests ran on one unit, bought as an EDUP "WiFi 6E Adapter
AX3000M" (the listing says MT7921AU, tri-band). Other dongles may share the ID
with different RF wiring. Only this unit was tested.

## Relation to upstream generic-ID support

Upstream commit `297d38d` (PR #4) enables `0e8d:7961` for any board and was
tested on an EDUP AX3000. This page covers a different unit. The table lists
the differences.

| | Upstream `297d38d` | This page |
| --- | --- | --- |
| Board | EDUP AX3000 | EDUP AX3000M ("WiFi 6E", tri-band) |
| Lowest working center | About 2179 MHz. Calibration fails below it | 2169 MHz. At 2168 MHz, tuning fails with `BIRRIASDR_ERROR_STATE` |
| Library floor for `0e8d:7961` | 2190 MHz, set in the device table | Unchanged. The upstream 2190 MHz floor applies |
| Matching | USB ID and chip ID | The same. A USB product string check was tried and dropped, because it would reject generic boards that upstream accepts |
| 5 GHz and 6 GHz | Not covered | Measured on this unit, see [5 GHz and 6 GHz](#5-ghz-and-6-ghz) |

The two units disagree about the floor by 10 MHz, so the limit belongs to the
board, not to the USB ID.

## USB descriptors

The tested unit reports `iManufacturer` "MediaTek Inc.", `iProduct`
"Wireless_Device", `iSerial` "000000000" and `bcdDevice` 1.00. These look like
MediaTek reference-design defaults, so they do not identify the EDUP model,
and other vendors may report the same strings. The library matches
`0e8d:7961` on the USB ID and on the chip ID that `identify()` reads from MMIO
`0x70010200` (low 16 bits `0x7961`). The chip ID check runs before any boot or
reset write.

## Result

The 2.4 GHz receive path works on this board with the existing channel 6
profile. Capture is clean from **2169 to 2610 MHz**. At 2168 MHz and below,
retuning fails, so the 2156 MHz lower limit of the COMFAST board does not hold
on this unit. Upstream's 2190 MHz floor for generic-ID boards keeps requests
above the failure.

The 5 GHz and 6 GHz receive paths also work on this unit. Tuning, calibration
and streaming succeed from **5150 to 7125 MHz**, and Wi-Fi traffic was received
at centers from **5220 to 5785 MHz** and from **6135 to 6855 MHz**. The high-band
profile was decoded from this unit's stock firmware. It has not been run on the
COMFAST board. See [5 GHz and 6 GHz](#5-ghz-and-6-ghz) below.

## Test setup

- The host is an x86-64 PC running Debian 12 and Linux 6.12.95, with the
  dongle on a USB 3.2 root port (`5000M`).
- The `mt7921u` kernel module was unloaded and blocked, then the dongle was
  replugged so that no vendor ROM patch was loaded. The binaries ran with
  `sudo`.
- The chip ID check in `src/chips/mt7921_capture.c` accepts the unit, and the
  analog die check during bring-up passes.
- The tests ran before upstream `297d38d`, with only the `qualified` flag and
  display name changed for this ID. Upstream's entry now provides the same
  thing, plus the 2190 MHz floor. The tests were not rerun against it.

## Evidence

| Check | Command | Result |
| --- | --- | --- |
| Offline tests | `ctest --test-dir build` | All ten tests pass |
| Enumeration | `birria_rx --list` | The list shows `0e8d:7961` with RX implemented |
| Capture | `birria_rx cap.cs16 8000000 manual` | Exit 0, stream status success, firmware state 3, reset success |
| Spectrum | 4,096-point averaged FFT of the 8,000,000 samples | Noise floor about 63 dB at the band edges, signals up to about 80 dB, strongest peak near −5 MHz from 2437 MHz |
| Tuning sweep | `birria_rx --frequency <hz> out.cs16 65536 manual` at 2200, 2250, 2300, 2350, 2400, 2437, 2480, 2500, 2550, 2600 and 2610 MHz | All exit 0 with stream status success |
| Lower limit | The same command, bisected | 2169 MHz passes and 2168 MHz fails. The 2156 and 2160 MHz points fail with `BIRRIASDR_ERROR_STATE` (−12) at stage 5 |
| Soak | `test_stability --run soak` (five minutes at 2437 MHz) | Two runs passed with about 12 billion samples and no DMA errors. An earlier run overran after about 18 seconds (−11, no DMA errors) and did not repeat |
| Control recovery | `test_control_recovery --run` | Passes |

## Known failures

- Before upstream `297d38d`, the `test_stability --run restart` and
  `--run live` phases and the `test_recovery --run` and `--run auto` commands
  failed at their tunes to 2156 MHz (`MT_RF_MIN_HZ`), the same limit as above.
  Both tests now take their endpoints from `birriasdr_get_tuning_range()`,
  which reports 2190 MHz for this ID. They are expected to pass but were not
  rerun.
- The cause of the 2168 MHz limit was not investigated. It may be the
  oscillator calibration or the readback check in `mt_tune_2g()` in
  `src/chips/mt7921_tune.c`. It is unknown whether the COMFAST board has the
  same limit.
- Sensitivity, frequency accuracy and filter response were not measured. As on
  the reference board, they are uncalibrated.

## 5 GHz and 6 GHz

The high-band profile in `src/chips/mt7921_band.c` and `mt_tune_high()` comes
from register snapshots of this unit under the stock `mt7921u` driver and its WM
firmware. The driver was loaded with `modprobe --ignore-install`, the interface
was put in monitor mode, and `iw dev <if> set freq` tuned it to 24 centers from
2412 to 7115 MHz. At each center a small libusb reader dumped RF ports 0, 1 and
5 (`0x000–0xffc`, three passes) and the PHY MMIO ranges through the same vendor
control requests the library uses. The kernel driver stayed bound, because
usbfs passes vendor requests without claiming the interface. `iw phy` lists
5 GHz and 5955–7115 MHz (6 GHz, no-IR) for this unit.

Hazard: reading RF SPI port 2 stops every later analog-die read until the
dongle loses power. A firmware reload through the driver's `chip_reset` does
not recover it. The library never uses port 2.

Test signals were nearby access points found by a beacon survey in monitor
mode: the strongest at 5220 MHz (about −47 dBm), others at 5320, 5500, 5640 and
5785 MHz, a 6 GHz network at 6135 and 6455 MHz, and weaker 6 GHz traffic at 6775
and 6855 MHz. 5180 MHz had no beacons and served as the negative control. All
captures used manual gain codes `{2, 8, 0}` unless stated, 40 MSps, and a burst
detector on 20 µs power windows relative to the median floor.

| Check | Command | Result |
| --- | --- | --- |
| Offline tests | `ctest --test-dir build` | All ten tests pass, including the unchanged channel 6 trace and the stock high-band words in `test_tune` |
| 5 GHz reception | `birria_rx --frequency 5220000000 c.cs16 20000000 manual` | 20 MHz OFDM bursts about 22 dB above the floor, centered at 0 MHz |
| 5 GHz offset | The same at 5230 MHz | The burst centroid moves to −10.1 MHz, so the spectrum is not inverted |
| Negative control | The same at 5180 MHz | No bursts. The peak 20 µs window is 3.3 dB above the floor |
| 6 GHz reception | The same at 6135 MHz | 20 MHz OFDM frames 12 dB above the floor, with short frames every 20 ms, the 6 GHz discovery interval |
| 6 GHz offset | The same at 6145 MHz | The centroid moves by −9.1 MHz. The burst sits near the −20 MHz edge, where the filter rolloff pulls the centroid in |
| Other AP channels | 500 ms captures at 5320, 5500, 5640, 5785, 6455, 6775 and 6855 MHz | Bursts 5.7 to 12.2 dB above the floor, centered within 0.4 MHz of 0. Empty channels peak at 0.5 to 0.8 dB |
| Tuning range | 100 ms captures every 50 MHz from 5150 to 5924 MHz and every 100 MHz from 5925 to 7125 MHz | All succeed |
| Live retune, 5 GHz | `read_async` at 5220 MHz, then `set_center_freq` to 5230, 6135 and 5180 MHz from a control thread. A second run went 5220 to 5785 MHz, across the front-end trim and channel-group steps | 5230 and 5180 succeed and the burst moves −10 MHz, then disappears. After 5785 the AP's bursts appear centered at +0.1 MHz. 6135 returns `BIRRIASDR_ERROR_UNSUPPORTED` and the stream continues |
| Live retune, 6 GHz | `read_async` at 6135 MHz, then 6455, 6855 and 5220 MHz | 6455 and 6855 succeed. Their segments peak 7.0 and 7.5 dB above the floor, as weak as the separate captures there. 5220 returns `BIRRIASDR_ERROR_UNSUPPORTED` and the stream continues |
| Restore path | 2437 MHz, then 5220 MHz, then 2437 MHz, without replug. Also a 2437 MHz capture right after a 6855 MHz one | Both 2.4 GHz captures receive normally. After 6 GHz, all 35 analog fields the band profile owns match the 2.4 GHz state after the 5 GHz restore. After 5 GHz, 33 of 35 matched the state before; RF `0x5ec` and `0x5f0` differed for a reason outside the band code, see below |
| Gain codes | 500 ms at 5220 MHz with `{0,8,0}`, `{3,8,0}`, `{2,0,0}` and `{2,8,12}`, and at 6135 MHz with `{0,8,0}`, `{3,8,0}` and `{2,8,12}` | At 5220 MHz, LNA 3 against LNA 0 raises the burst peak from 10.3 to 18.9 dB above the floor. At 6135 MHz it raises it from 1.5 dB (lost in the floor) to 10.0 dB. Digital code 12 raises the floor about 16 to 17 dB on both bands. Filter-stage gain has little effect at 5220 MHz. Released gain mode also streams at 5 GHz |
| 2.4 GHz regression | `test_hardware --run`, `test_control_recovery --run` on this build | Both pass |
| Soak | `read_async` for 300 s at 5220 MHz and at 6135 MHz | 6135 MHz passed with 12.0 billion samples and no DMA errors. At 5220 MHz one run overran after 196 s (−11, firmware state 9, no DMA errors) and a second run passed with 12.0 billion samples |

### Known limits

- Live retuning cannot cross bands, and 5 GHz to 6 GHz counts as crossing.
  Restart the acquisition at the new center instead.
- The edges of the command intervals are a choice, not a measurement. The tuning
  sweep shows calibration completes across them, but reception was verified
  only at the centers listed above. The 5 GHz/6 GHz switch at 5925 MHz is
  between the stock firmware's measured profiles at 5885 and 5955 MHz.
- Sub-band steps (front-end trim, synthesizer presets, PHY channel group) use
  edges midway between measured centers.
- The per-gain tables, firmware calibration results and PHY AGC tables that the
  stock firmware writes are not applied. Sensitivity relative to the stock
  driver was not measured.
- RF `0x5ec` and `0x5f0` on ports 0 and 1 change between sessions without
  the band code. After a replug and one 2.4 GHz session they read `0x4060c207`
  and `0x0022041c`. After a 5220 MHz session and another 2.4 GHz session they
  read `0x404f90f5` and `0x00220419`, and stayed there through later 2.4 GHz
  sessions. The band profile saves its fields before writing and restores
  exactly those bits, so its fields were already at the second value when the
  5220 MHz session started. Bits outside its fields changed too, and the second
  value matches a dump taken before the band code existed. Which step writes
  them is unknown. Neither value stopped 2.4 GHz reception.
- None of this has run on the COMFAST `3574:6211` reference board.
