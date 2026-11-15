# Transmitter firmware (airborne beacon)

Arduino sketch for the beacon that flies in the rocket. It reports position,
an EKF-fused track, inertial forensics and flight events over 433 MHz LoRa,
and keeps a crash-survivable flight log in the MCU's own flash.

## Hardware

| Part | Interface | Notes |
|------|-----------|-------|
| Adafruit ItsyBitsy M4 Express (SAMD51, 120 MHz) | – | FQBN `adafruit:samd:adafruit_itsybitsy_m4` |
| u-blox MAX-M10S GPS | `Serial1` (pins 0/1), 115200 | boot self-heals the baud from 9600 (UBX-CFG-VALSET + PUBX,41) |
| BNO085 IMU | I2C, address `0x4A` | reset pin 10 (optional) |
| E22-400M33S (SX1268) LoRa | SPI | pins below |

Radio pins (`include/mpu_config.h`): CS 5, DIO1 9, BUSY 7, NRST 2, RF switch
TXEN A4 / RXEN A5 (MCU-driven), backup-channel jumper A1.

Radio settings: channel `ch` = 433.000 + 0.25 x ch MHz (8 channels), SF10,
62.5 kHz bandwidth, CR 4/8, private sync word 0x12, 22 dBm, 8-symbol
preamble. The channel is set at build time (`LORA_CHANNEL`); the carrier
board has no channel-select hardware.

## Flight states and cadence

Production values (`TESTING_MODE 0` in `include/config.h`; intervals live in
`config.h`, the state-to-interval mapping in `flight_cadence.cpp`).

| State | Entered | On the air |
|-------|---------|------------|
| TURN_ON | boot, for 60 s | heartbeat every 5 s; one full position when the first good fix lands |
| PRE_LAUNCH | after TURN_ON | heartbeat every 60 s; a position audit copy every 600 s |
| LAUNCH | launch confirmed (IMU, or GPS-altitude fallback) | continuous TX: inertial trace, fused track every 1.5 s, launch T0 |
| POST_LAUNCH | 1 s after launch | fused track every 1.5 s, inertial trace every 6 s (1 s during an anomaly), maxima recap every 15 s, flight events - ~87% of the radio's airtime, ~90% with three commercial altimeters riding the recap |
| BATTERY_SAVE | landing detected, or 600 s after POST_LAUNCH began | position every 60 s, fused track every 30 s (none while the fix is stale); with no usable fix a heartbeat every 5 s for the first 30 min (walk-in, RSSI direction finding), then every 30 s |

The callsign (FCC ID) airs at boot and every 300 s whenever the beacon is
not in continuous TX.

**Mid-flight reboot.** A persistent FLYING marker in the flight-log region
survives a watchdog bite or brown-out. `setup()` then resumes LAUNCH with the
launch time set to the resume, appends to the surviving log (never erases
it), re-airs the callsign and certifies an `ANOM_REBOOT` flight event. The
marker is rewritten LANDED at the landing latch or, when the latch never
trips (GPS lost at touchdown, a tree, a dead IMU), when the 600 s recovery
window ends - so a later power-on is a normal pad boot.

## Packets

Layouts are in `include/packet_format.h`. The receiver has its own copy in
`receiver/firmware/inc/`; edit both together (see `AGENTS.md`).

| Type | Code | Purpose |
|------|------|---------|
| GPS | 0x01 | raw position (pad audit, battery save) |
| FUSED | 0x04 | EKF position + velocity, flags (dead reckoning, landed...) |
| HEARTBEAT | 0x05 | no-fix keepalive with GPS health and reset cause |
| IMU | 0x06 | inertial trace: accel, gyro, peak accel |
| LAUNCH_T0 | 0x07 | one-shot launch declaration |
| CMD / ACK | 0x08 / 0x09 | receiver command and the beacon's answer (PING) |
| FLIGHT_EVENT | 0x0A | apogee, drogue, main, landed, anomalies (repeated) |
| MAXIMA | 0x0B | running flight maxima; the recognised commercial altimeters (format, apogee, fired charges) ride it as a tail |
| HELLO | 0x0C | boot identity (firmware hash) |
| callsign | text | `CALLSIGN-id CHn` |

**IMU trace V3.** Bit 15 of `ts_ms` (`IMU_TS_V3_FLAG`) marks V3 units:
acceleration in centi-g, gyro in 0.1 deg/s (+-3276 deg/s), peak in milli-g,
all saturating. Earlier firmware sent m/s^2-based values that wrapped; the
receiver logs an `ImuFmt` column so both decode correctly.

**Two-way channel.** In pad states the beacon listens about once a second
for 40 ms, and keeps listening up to 2.2 s once a preamble is detected. The
receiver sends its bind-time PING with a long preamble so a listen window
always lands inside it.

## Commercial altimeter inputs

The carrier's J2, J5 and J6 (D13, D12, D11) take the serial telemetry of
commercial altimeters: 5 V TTL through 10k/20k dividers, 9600 8N1, receive
only. A 38.4 kHz TC3 interrupt samples all three lines (software UARTs:
SERCOM3, the only other SERCOM on these pins besides SERCOM5, runs the
GPS), and each input recognises its altimeter from the data, so any
altimeter works on any jack:

| Altimeter | Setup | What the beacon reads |
|-----------|-------|------------------------|
| PerfectFlite StratoLogger CF | telemetry OnPad or OnLaunch (DT2/DataCast) | one altitude per line, ft (OnPad's first line, the site elevation MSL, is dropped at launch) |
| Missile Works RRC3 | telemetry pre-launch @ 0.5 s (data control 1), **all data items** (31, or 63 with battery on v1.60+), imperial units | altitude, fired drogue/main/aux (UPPERCASE while firing; on v1.5x also lowercase after `-`) |
| Eggtimer (Quantum/Quasar/Proton) | telemetry output | `{` altitude/100 ft (`<` feet on the Quark pre-D3), `%` apogee after nose-over, `~` channels (1 = drogue, 2 = main) |

A format locks after three consistent records. While airborne the
recognised inputs ride every maxima recap (3 bytes each: format, fired
charges, stale flag and apogee; 0x0D, a separate relay packet, was retired
before it flew), and a recap goes out at once when an altimeter reports a
newly fired charge; with no altimeter recognised the recap is the plain 11
bytes. In flight the
flash log keeps a record per reporting input every second and at each new
event (DUMP kind 11). An unplugged jack is held low by its pull-down and
produces nothing. D13 is also the ItsyBitsy's red LED, which loads J2's
divider: check the signal level on the bench before relying on J2.

## Build and flash

```sh
make -C transmitter/firmware compile ID=3 CHANNEL=5   # -DROCKET_ID=3 -DLORA_CHANNEL=5
make -C transmitter/firmware upload  ID=3 CHANNEL=5 PORT=/dev/ttyACM0
```

`ROCKET=n` sets both ID and channel. The finished sketch must end below
`0x00040000`, where the 256 KiB flight-log region starts.

## Serial monitor and the flight log

USB serial at 115200 prints boot banners (reset cause, watchdog or
brown-out recovery), state changes and radio results. Typing `DUMP` + Enter
prints the on-chip flight log as CSV (refused while airborne). The first
lines describe the IMU units; IMU rows end with a format byte (3 = V3).
Save that output to a file and run `python3 tools/flight_log_dump.py
capture.txt` for the last known position, the flight timeline (boot causes,
launch, events, landing) and the maxima.

## GPS watchdog

When the UART goes silent for 30 s, or bytes flow without a good fix for
60 s, the watchdog sends a real UBX-CFG-RST: hot start, then warm, then
cold, followed by a settings re-send. After three attempts it cools down
for 10 minutes. In flight it never resets the receiver, it only re-sends the
settings. See `docs/gps-watchdog.md`.

## Tests

```sh
make -C transmitter/tests test       # every module + a main-loop flight simulation
make -C transmitter/tests sanitize   # same under ASan + UBSan + float-cast-overflow
make -C transmitter/tests coverage   # line + branch coverage per module
```
