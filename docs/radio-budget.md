# Radio data budget and phases

A reference for anyone flashing between bench and flight. All values are at
the flight profile: SF10 / BW 62.5 kHz / CR 4/8 / LDRO enabled / preamble 8 /
sync 0x12. Transmit airtime is the expense; the receiver idles otherwise.

Airtime per packet type at this configuration:

| Packet                    | Bytes | Airtime |
|---------------------------|-------|---------|
| TX fused position+velocity |   20  | ~1.00 s |
| TX inertial trace (IMU)    |   20  | ~1.00 s |
| TX raw GPS (binary)        |   14  | ~0.75 s |
| TX heartbeat               |    9  | ~0.55 s |
| TX callsign (text)         |  <16  | <0.85 s |
| TX launch T0               |    6  | ~0.42 s |
| RX->TX command reply       |    6  | ~0.42 s |

Duty budget: we soften above ~85 % PA on-time per second; above that the
E22's regulator overheats during extended burns.

## Phase-by-phase policy (transmitter/firmware/flight_cadence.cpp)

Principles: the packet types you can afford are the ones that answer the
question that phase actually leaves you asking. The single FPGA-style
constraint: if something dies mid-phase, we want enough in RF to tell WHAT
died first.

| Phase | PQuestions to answer | Packets on the air |
|-------|----------------------|--------------------|
| PRE_LAUNCH (pad) | Satellite health, link margin, callsign | Fused every 5 s + GPS every 5 s + heartbeat if no fix + callsign every 5 min. Idle budget is essentially free - we're waiting for lift. |
| LAUNCH (boost) | velocity/alt thrust curve, accel axes, T0 instant | fused @1.5 s + IMU @2 s, GPS silenced. (Post L0016 lesson.) At most ~90% duty for *shortest* window of the flight. |
| POST_LAUNCH (coast-apogee-under-chute) | when and where the chutes are, ballistic drift, velocity peak, apogee T | fused @1.5 s, IMU @2 s, GPS stays silenced. Recommended where forensics live past the shred-point. |
| LANDED | touchdown anchor + drag events | drop to BATTERY_SAVE: fused @60 s + GPS @60 s and keep alive |
| WALK/RECOVERY | last-remembered point, foul antenna margins | battery-save stays; stay-tx is the cheap version |

## Two-way channel (RX→TX)

Intent: post-landing v2 board: receiver asks probe command replies (e.g.,
MAC ping / CNR-verified response) with tiny 6-14 byte packets that the
receiver hears; the beacon never busy-listens: it limits received to the
2 quiet seconds along the cadence (combined with duty).

Wire contract (wire-boundary-stable):

| Packet     | Type     | Dir       | Bytes | Purpose                     |
|------------|----------|-----------|-------|------------------------------|
| CMD        | 0x08     | RX → TX   |   6   | { type, rocket_id, cmd_code, seq_hi, seq_lo } |
| ACK        | 0x09     | TX → RX   |   6   | { type, rocket_id, cmd_code, seq_hi, LSB }    |

cmd_code reserves: 0x01=PING, 0x02=TUNE_CH (next channel hops together), rest
reserved. Both sides may drop a request if in flight: ACKs are best-effort.

Implementation status: see repository tags starting
*gently-seeded*; radios support it in software, but only scantly tested.
