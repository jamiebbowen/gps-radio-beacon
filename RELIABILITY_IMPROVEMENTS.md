# GPS Radio Beacon - Reliability Improvements

## Summary
Added multiple layers of GPS validation to prevent transmission of bad data and improve overall system reliability.

## Improvements Implemented

### 1. ✅ Minimum Satellite Count Validation
**Impact: HIGH**
- **Requirement:** Minimum 4 satellites before accepting fix
- **Why:** GPS needs 4+ satellites for 3D position accuracy
- **Benefit:** Prevents transmission of poor-quality position data
- **Example Output:** `[Beacon] Rejecting GPS - insufficient satellites: 3`

### 2. ✅ GPS Fix Quality Check (GGA)
**Impact: HIGH**
- **Parses:** GGA sentence field 6 (fix quality indicator)
- **Values:**
  - 0 = Invalid/no fix
  - 1 = GPS fix (SPS)
  - 2 = DGPS fix (better accuracy)
  - 3+ = Other fix types
- **Requirement:** Fix quality ≥ 1
- **Benefit:** Ensures GPS has valid solution before transmitting
- **Example Output:** `[Beacon] Rejecting GPS - no fix, quality: 0`

### 3. ✅ Altitude Sanity Check
**Impact: MEDIUM**
- **Range:** -500m to 50,000m
- **Why:** Detects GPS glitches that produce impossible altitudes
- **Benefit:** Prevents transmission of obviously corrupt data
- **Example Output:** `[Beacon] Rejecting GPS - unreasonable altitude: 123456.7`

### 4. ✅ LoRa Transmission Error Reporting
**Impact: MEDIUM**
- **Action:** Logs error codes when LoRa transmission fails
- **Benefit:** Makes RF problems visible for debugging
- **Example Output:** `[Beacon] ✗ Transmission failed, error code: -2`

### 5. ✅ GPS Debug Output
**Impact: LOW (diagnostic)**
- **Displays:** Bytes received and fix validity every 5 seconds
- **Benefit:** Easy diagnosis of GPS communication issues
- **Example Output:** `[GPS] Bytes rcvd: 1024 | Valid: 1`

## Additional Reliability Features (Already Present)

### Launch Detection Debouncing
- 50ms debounce time prevents false triggers
- State machine with IDLE → DETECTED → CONFIRMED states

### LoRa Built-in Features
- Hardware CRC validation (automatic)
- Automatic packet framing
- Forward error correction
- Guaranteed complete packets or nothing

### Coordinate Validation (Receiver Side)
- Rejects (0,0) "Null Island" coordinates
- Jump detection (>100km movements rejected)
- Latitude/longitude range validation

## System Reliability Summary

| Layer | Component | Protection |
|-------|-----------|------------|
| **Hardware** | LoRa CRC | Corrupt packet detection |
| **Hardware** | GPS multi-constellation | Better satellite availability |
| **Software** | Minimum 4 satellites | Position accuracy |
| **Software** | Fix quality check | Valid GPS solution |
| **Software** | Altitude sanity | Glitch detection |
| **Software** | Coordinate validation | Jump detection |
| **Software** | Launch debouncing | False trigger prevention |

## Before/After Comparison

### Before Improvements:
```
✗ Would transmit with 1 satellite (poor accuracy)
✗ Would transmit with fix_quality=0 (no solution)
✗ Would transmit altitude = 999999m (GPS glitch)
✗ Silent LoRa failures (no diagnostic info)
```

### After Improvements:
```
✓ Requires 4+ satellites (good geometry)
✓ Requires valid GPS fix (quality ≥ 1)
✓ Validates altitude range (-500m to 50km)
✓ Reports transmission failures with error codes
✓ GPS diagnostic output every 5 seconds
```

## Firmware Size Impact

- **Before:** 60,320 bytes (11% of flash)
- **After:** 66,488 bytes (13% of flash)
- **Increase:** 6,168 bytes (1% additional)
- **Verdict:** ✅ Acceptable trade-off for improved reliability

## Testing Recommendations

1. **Indoor Test (No GPS):** Should see fix_quality=0 rejections
2. **Window Test (Weak Signal):** Should see satellite count rejections
3. **Outdoor Test (Good Signal):** Should transmit normally with all validations passing
4. **Monitor Serial Output:** Check for rejection messages during acquisition

## Future Enhancements (Optional)

### Low Priority:
- **HDOP validation** - Parse horizontal dilution of precision for accuracy metric
- **Watchdog timer** - Auto-reset on firmware hangs (ItsyBitsy M4 has hardware watchdog)
- **Coordinate rate limiting** - Prevent rapid position changes
- **Battery voltage monitoring** - Track power supply health
- **Transmission retry logic** - Retry failed LoRa sends (may reduce throughput)

### Not Recommended:
- ❌ **Error correction on receiver** - Already implemented, may be overkill
- ❌ **Packet acknowledgments** - Would require two-way communication
- ❌ **GPS cold start assistance** - GPS module handles this internally

## Conclusion

The beacon now has **defense in depth** for GPS data quality:
1. Hardware (LoRa CRC, GPS chipset)
2. Signal quality (satellites, fix quality)
3. Data sanity (altitude, coordinates)
4. Diagnostics (error reporting)

These improvements significantly reduce the chance of transmitting bad data while maintaining good transmission rates when GPS has a valid fix.
