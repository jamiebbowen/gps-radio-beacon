# GPS Radio Beacon Receiver - Reliability Improvements

## Summary
Added multiple layers of RF signal quality monitoring and data validation to improve receiver reliability and user awareness of data quality.

## Improvements Implemented

### 1. ✅ Data Staleness Detection
**Impact: HIGH**
- **Threshold:** 30 seconds without new packets = stale data
- **Function:** `RF_Receiver_IsDataStale(current_time_ms)`
- **Benefit:** Clearly indicates when position data is old
- **Use Case:** Navigation mode can warn users when pointing to last-known position

### 2. ✅ Signal Quality Monitoring
**Impact: MEDIUM**
- **RSSI Threshold:** -120 dBm minimum
- **SNR Threshold:** -10 dB minimum
- **Function:** `RF_Receiver_IsSignalQualityGood()`
- **Benefit:** Reject packets with poor signal quality that may have corrupted data
- **Real-time Metrics:** Available via `RF_Receiver_GetSignalQuality()`

### 3. ✅ Duplicate Packet Filtering
**Impact: MEDIUM**
- **Detection:** String comparison of packet data
- **Action:** Skip processing identical consecutive packets
- **Benefit:** Prevents wasted CPU cycles and duplicate position updates
- **Counter:** Tracks number of duplicates rejected

### 4. ✅ Packet Timestamp Tracking
**Impact: HIGH**
- **Updated:** On every successfully parsed packet
- **Accessor:** `RF_Receiver_GetLastPacketTime()`
- **Benefit:** Enables age calculation and stale data detection
- **Use Case:** Display "STALE DATA" warnings in navigation mode

## Configuration Parameters

```c
/* In rf_receiver.h */
#define RF_DATA_STALE_TIMEOUT_MS  30000  /* 30 seconds */
#define RF_MIN_RSSI_DBM          -120    /* Minimum RSSI */
#define RF_MIN_SNR_DB            -10     /* Minimum SNR */
```

## New API Functions

### Signal Quality
```c
void RF_Receiver_GetSignalQuality(int16_t *rssi, int8_t *snr);
uint8_t RF_Receiver_IsSignalQualityGood(void);
```

### Data Age
```c
uint8_t RF_Receiver_IsDataStale(uint32_t current_time_ms);
uint32_t RF_Receiver_GetLastPacketTime(void);
```

## Already Existing Reliability Features

### Coordinate Validation (from memories)
- ✅ Rejects (0,0) "Null Island" coordinates
- ✅ Jump detection (>100km movements rejected)
- ✅ Latitude/longitude range validation
- ✅ Tracks invalid coordinate errors

### LoRa Hardware Features
- ✅ Built-in CRC validation
- ✅ Forward error correction
- ✅ Automatic packet framing
- ✅ Guaranteed complete packets or nothing

### Navigation Mode Features
- ✅ Uses last known good position when data is stale
- ✅ Displays "STALE DATA" warning
- ✅ Continues updating compass relative direction

## Reliability Layers Summary

| Layer | Feature | Protection Against |
|-------|---------|-------------------|
| **Hardware** | LoRa CRC | Bit errors |
| **Hardware** | LoRa FEC | Noise corruption |
| **Software** | RSSI/SNR check | Weak signals |
| **Software** | Duplicate filter | Redundant processing |
| **Software** | Staleness detection | Outdated position |
| **Software** | Timestamp tracking | Data age awareness |
| **Software** | Coordinate validation | GPS glitches |
| **Display** | Stale warnings | User confusion |

## Example Usage in Navigation Mode

```c
/* Check if RF data is fresh and good quality */
uint8_t data_fresh = !RF_Receiver_IsDataStale(HAL_GetTick());
uint8_t signal_good = RF_Receiver_IsSignalQualityGood();

if (data_fresh && signal_good) {
    /* Use position for navigation */
    display_navigation_arrow();
} else if (!data_fresh) {
    /* Show stale data warning */
    Display_DrawTextRowCol(6, 0, "STALE DATA");
    /* Still show last known position */
} else {
    /* Poor signal quality */
    Display_DrawTextRowCol(6, 0, "WEAK SIGNAL");
}
```

## Display Mode Enhancements (Recommended)

### RF Mode Display
Add signal quality indicators:
```
RF Data: 45 pkts
RSSI: -85 dBm ✓
SNR: 8.5 dB ✓
Age: 2.3s [FRESH]
```

### Navigation Mode
Already has stale data warning:
```
Distance: 2.5 km
Bearing: 045°
[STALE DATA]  <-- Shows when >30s old
```

## Performance Impact

- **Before:** 74,896 bytes (14% of flash)
- **After:** 75,732 bytes (14% of flash)
- **Increase:** 836 bytes (<1% additional)
- **RAM Impact:** ~130 bytes (duplicate buffer + counters)
- **CPU Impact:** Minimal (simple comparisons per packet)

## Signal Quality Thresholds Rationale

### RSSI: -120 dBm Minimum
- LoRa sensitivity: ~-137 dBm (SF9, 125kHz)
- Gives 17 dB margin for reliable reception
- Typical outdoor: -60 to -100 dBm
- Edge of range: -110 to -120 dBm

### SNR: -10 dB Minimum
- LoRa can decode down to -20 dB SNR
- -10 dB threshold provides 10 dB margin
- Ensures data quality in noisy environments
- Prevents accepting marginal packets

## Data Staleness Threshold Rationale

### 30 Seconds
- Transmitter sends every 20s (PRE_LAUNCH state)
- 30s allows one missed packet
- Faster than walking speed position change
- Balances freshness vs. availability

For LAUNCH state (2s intervals):
- 30s = 15 missed packets (very tolerant)
- Appropriate for high-speed rocket tracking

## Testing Recommendations

### Signal Quality Testing
1. **Good Signal:** Walk close to transmitter
   - Should show RSSI > -80 dBm, SNR > 5 dB
   - `IsSignalQualityGood()` returns 1

2. **Poor Signal:** Walk to edge of range
   - May show RSSI < -120 dBm, SNR < -10 dB
   - `IsSignalQualityGood()` returns 0
   - System should still display last known position

3. **No Signal:** Go indoors or very far away
   - After 30s, "STALE DATA" warning appears
   - Last known position still shown
   - Compass continues to update relative arrow

### Duplicate Detection Testing
1. Monitor duplicate counter
2. Should increment when transmitter re-sends
3. No duplicate position updates on display

## Future Enhancements (Optional)

### Low Priority
- **Adjustable thresholds** - Allow user to configure RSSI/SNR limits
- **Signal strength bar graph** - Visual indicator of signal quality
- **Packet statistics** - Success rate, avg RSSI/SNR over time
- **Auto range warning** - Alert when approaching signal loss

### Not Recommended
- ❌ **Aggressive filtering** - May discard valid edge-case packets
- ❌ **Packet retry requests** - Requires two-way communication
- ❌ **Automatic TX power adjustment** - Receiver can't control transmitter

## Comparison: Before vs. After

### Before Improvements
```
❌ No signal quality checking (accepted all packets)
❌ No staleness detection (old data not marked)
❌ No duplicate filtering (wasted CPU cycles)
❌ No timestamp tracking (no age awareness)
```

### After Improvements
```
✅ RSSI/SNR thresholds ensure quality
✅ 30s staleness timeout marks old data
✅ Duplicate packets automatically rejected
✅ Timestamp enables age-based decisions
✅ Clear user warnings when data is stale
✅ Navigation continues with last known position
```

## Integration with Existing Features

### Works With Coordinate Validation
- Signal quality check happens FIRST (RF layer)
- Coordinate validation happens AFTER (GPS layer)
- Both reject bad data, different reasons

### Works With Navigation Mode
- Staleness check uses existing last_packet_time
- "STALE DATA" display already implemented
- Arrow continues rotating with compass (existing feature)

### Works With LoRa Hardware
- Software quality checks complement hardware CRC
- SNR metric directly from LoRa chipset
- RSSI metric directly from LoRa chipset

## Conclusion

The receiver now has **comprehensive data quality monitoring**:

1. **Hardware Layer** (LoRa CRC, FEC)
2. **Signal Layer** (RSSI, SNR thresholds)
3. **Duplicate Layer** (Identical packet filtering)
4. **Age Layer** (Staleness detection)
5. **Data Layer** (Coordinate validation)
6. **Display Layer** (User warnings)

These improvements ensure users always know when:
- Data is fresh and reliable
- Data is old but still useful
- Signal quality is marginal
- Position updates are duplicates

The system gracefully degrades from perfect signal to no signal, providing maximum utility in all conditions. 🚀
