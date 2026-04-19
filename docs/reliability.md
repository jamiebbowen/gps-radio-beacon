# Reliability

This document consolidates the defence-in-depth measures the system uses to
avoid transmitting or displaying bad data, and the LoRa link-budget rationale
behind the current radio configuration.

It supersedes the earlier scattered write-ups:

- `RELIABILITY_IMPROVEMENTS.md` (transmitter / GPS validation)
- `RECEIVER_RELIABILITY_IMPROVEMENTS.md` (receiver / signal quality)
- `LORA_RELIABILITY_GUIDE.md` (radio link budget)

---

## 1. Reliability layers at a glance

| Layer | Component | Guards against |
|---|---|---|
| Hardware | LoRa CRC | Bit errors |
| Hardware | LoRa FEC (CR 4/7) | Noise corruption |
| Hardware | GPS multi-constellation | Poor satellite geometry |
| Transmitter | Min 4 satellites | Low-accuracy positions |
| Transmitter | GGA fix-quality >= 1 | No-solution packets |
| Transmitter | Altitude sanity (-500 to 50,000 m) | GPS glitches |
| Transmitter | TX error reporting | Silent radio failures |
| Receiver | RSSI threshold (-120 dBm) | Weak/edge packets |
| Receiver | SNR threshold (-10 dB) | Noise-corrupted packets |
| Receiver | Duplicate filter | Redundant processing |
| Receiver | Staleness timeout (30 s) | Stale position display |
| Receiver | Coordinate validation | "Null Island", 100 km jumps |
| UI | "STALE DATA" / "WEAK SIGNAL" warnings | User confusion |

---

## 2. Transmitter-side validation

Before a packet is transmitted, the beacon runs these checks on the parsed
NMEA data. Failed checks log a reason and skip the transmission.

### 2.1 Minimum satellite count

- Required: **>= 4 satellites** (needed for 3-D GPS solution)
- Example log: `[Beacon] Rejecting GPS - insufficient satellites: 3`

### 2.2 GPS fix-quality indicator (GGA field 6)

| Value | Meaning |
|---|---|
| 0 | Invalid / no fix |
| 1 | Standard GPS fix (SPS) |
| 2 | DGPS fix |
| 3+ | Other (PPS, RTK, ...) |

Required: `quality >= 1`. Example log: `[Beacon] Rejecting GPS - no fix, quality: 0`.

### 2.3 Altitude sanity check

Accepts only altitudes in **-500 m to 50,000 m**. Anything outside is treated
as a GPS glitch and dropped. Example log:
`[Beacon] Rejecting GPS - unreasonable altitude: 123456.7`.

### 2.4 Transmission error reporting

When `RadioLib::transmit()` returns an error, the beacon logs the status code
instead of silently dropping the packet. Example:
`[Beacon] X Transmission failed, error code: -2`.

### 2.5 Launch-detection debouncing

Launch is only declared when the IMU reports acceleration above threshold for
a sustained window (default 100 ms at 20 m/s^2). State machine:
`IDLE -> DETECTED -> CONFIRMED`. Prevents a single sharp bump from triggering
the continuous-transmit state.

---

## 3. Receiver-side validation

### 3.1 Signal quality

Thresholds (in `receiver/firmware/inc/rf_receiver.h`):

```c
#define RF_MIN_RSSI_DBM   -120   /* LoRa SF7 sensitivity ~-123 dBm */
#define RF_MIN_SNR_DB      -10   /* LoRa can decode down to -20 dB */
```

Exposed via:

```c
void    RF_Receiver_GetSignalQuality(int16_t *rssi, int8_t *snr);
uint8_t RF_Receiver_IsSignalQualityGood(void);
```

### 3.2 Staleness detection

```c
#define RF_DATA_STALE_TIMEOUT_MS  30000   /* 30 seconds */
uint8_t  RF_Receiver_IsDataStale(uint32_t current_time_ms);
uint32_t RF_Receiver_GetLastPacketTime(void);
```

Rationale: pre-launch cadence is ~20 s, so a 30 s timeout tolerates one
missed packet. Post-launch cadence is ~2 s, so 30 s equals 15 missed
packets before giving up (tolerant enough for worst-case RF fades during
boost / tipover).

### 3.3 Duplicate filtering

Identical consecutive packets (string comparison) are skipped to avoid
redundant UI updates and wasted CPU. A counter tracks how many have been
filtered.

### 3.4 Coordinate validation

- Rejects (0,0) "Null Island"
- Rejects any movement >100 km from the last accepted fix (jump detection)
- Validates lat/lon ranges (+/- 90, +/- 180)

### 3.5 User-visible warnings

The navigation UI surfaces the state of the data so the operator is never
misled:

```
Distance: 2.5 km
Bearing:  045 deg
[STALE DATA]    <- shown when last packet > 30 s old
[WEAK SIGNAL]   <- shown when RSSI or SNR below threshold
```

Navigation mode continues to point to the **last known good position** when
data is stale, because an out-of-date bearing is still useful for recovery.

---

## 4. LoRa link budget

### 4.1 Current configuration

```c
Frequency:        433 MHz       /* US 70 cm amateur band */
Bandwidth:        125 kHz
Spreading factor: SF9
Coding rate:      4/7
TX power:         30 dBm (1 W)  /* max legal in US */
Preamble:         8 symbols
Sync word:        0x12
```

### 4.2 Budget worked through

- TX power: **+30 dBm**
- TX antenna gain (17 cm whip, ~lambda/4): **+2 dBi**
- Cable loss: **-0.5 dB**
- EIRP: **+31.5 dBm**

- Free-space path loss at 10 km, 433 MHz: **-132 dB**

- RX antenna gain: **+2 dBi**
- RX cable loss: **-0.5 dB**
- RX sensitivity (SF9, BW125): **-136 dBm**
- Effective RX floor: **-138 dBm**

```
Link margin = EIRP + RX antenna + RX sensitivity - path loss
            = 31.5 + 2 - 0.5 - (-138) - 132
            = ~39 dB at 10 km
```

**~39 dB of headroom** is enough to survive typical fading (10-20 dB),
partial obstructions (10-30 dB), and co-channel interference (~10 dB) and
still close the link.

A real flight (Apr 2026, ItsyBitsy + E22 + 17 cm whip, SF7) measured a
worst-case RSSI of -86 dBm at 2.1 km slant range, giving a **37 dB link
margin** at SF7 -- consistent with the budget above.

### 4.3 Configuration trade-offs

| Setting | Range | Air time | Data rate | Use case |
|---|---|---|---|---|
| SF7, 125 kHz | ~4-6 km | 60 ms | 5.5 kbps | Fast updates, short range |
| SF9, 125 kHz (default) | ~8-12 km | 370 ms | 1.8 kbps | Balanced rocket tracking |
| SF10, 125 kHz | ~12-18 km | 660 ms | 980 bps | Max-range stationary |
| SF12, 125 kHz | ~15-25 km | 2.6 s | 290 bps | Not useful for flight |

**Preamble**: default 8 symbols is a good compromise. Increasing to 12-16
improves low-SNR detection with only ~3-5% extra air time.

### 4.4 Antenna is the biggest single lever

Everything above assumes a properly tuned 433 MHz antenna. A mismatched or
broken antenna can cost **10-20 dB** (50-90% range reduction). Checklist:

- IPEX connector fully seated on E22 module
- Antenna is lambda/4 for 433 MHz (~17 cm)
- Polarization matches between TX and RX (both vertical)
- TX antenna has clear line of sight (mounted externally, no metal near)
- SWR below 2.0 in the target band

---

## 5. Regulatory considerations

### 5.1 US (FCC)

- 433.05-434.79 MHz is a secondary allocation inside the 70 cm amateur band
- Max power: 1 W (30 dBm) -- current default is legal
- Beacon transmits its callsign every 5 minutes (FCC identification requirement)
- An amateur radio license (Technician class or higher) is required

### 5.2 EU (ETSI)

- 433 MHz ISM is limited to **10 mW (10 dBm)** with duty-cycle limits
- Change `LORA_TX_POWER` to `10` before operating in the EU

### 5.3 Other regions

**Check your local regulations before transmitting.**

---

## 6. Tested vs. conservative

Current configuration has been verified on:

- 2 km AGL flight, Mach 0.99 peak, 125 s duration
- 0 packet losses during flight
- RSSI -86 dBm at apogee (2.1 km slant range)
- 37 dB measured link margin

Extrapolating the same RSSI/range pair via free-space loss gives a predicted
usable range on the order of 100 km line-of-sight, but real-world range will
be limited by horizon geometry, terrain, and antenna pattern long before
sensitivity becomes the limit. See `docs/flight-analysis.md` for how
`analyze_flight.py` computes predicted range from flight data.

---

## 7. Not-implemented / rejected approaches

These were considered and intentionally **not** added:

- **Packet acknowledgments** -- would require two-way comms, breaks the
  simplex design, would also halve effective TX rate
- **Frequency hopping** -- adds sync complexity for minimal benefit at our
  link margins
- **Adaptive data rate** -- requires an RSSI feedback channel the beacon
  doesn't have
- **Aggressive software filtering at the receiver** -- risks discarding
  valid edge-of-range packets that are actually still useful
