# BNO085 IMU Launch Detection Guide

## Overview

The beacon now uses the **BNO085 9-axis IMU** for intelligent launch detection instead of a simple switch. This provides:

✅ **Reliable detection** - Detects actual rocket motor ignition acceleration  
✅ **No mechanical parts** - More reliable than switches  
✅ **Adjustable sensitivity** - Configure threshold for your rocket  
✅ **False alarm prevention** - Requires sustained acceleration  
✅ **Diagnostic data** - Real-time acceleration monitoring

## Hardware Connection

### ItsyBitsy M4 Express → BNO085

| ItsyBitsy M4 | BNO085 | Function |
|--------------|--------|----------|
| **SDA** | SDA | I2C Data |
| **SCL** | SCL | I2C Clock |
| **Pin 10** | RST | Reset (optional) |
| **3.3V** | VIN | Power |
| **GND** | GND | Ground |

### I2C Address

The BNO085 can have two I2C addresses:
- **0x4A** - Default (SA0 pin floating or low)
- **0x4B** - Alternative (SA0 pin high)

Current configuration uses **0x4A**. Change in `include/mpu_config.h` if needed:
```cpp
#define IMU_I2C_ADDR 0x4B  // If using alternative address
```

### Wiring Notes

1. **I2C Pullups**: The ItsyBitsy M4 has built-in pullups, but some BNO085 breakout boards include them too. This is usually fine.
2. **Reset Pin**: Optional but recommended for Pin 10. Can set to `-1` if not used.
3. **Power**: BNO085 needs 3.3V. ItsyBitsy M4 provides this.
4. **Orientation**: Mount with Z-axis pointing up for best results.

## Launch Detection Algorithm

### How It Works

1. **Continuous Monitoring**: Reads linear acceleration (gravity removed) at ~100Hz
2. **Threshold Detection**: Detects when total acceleration exceeds threshold
3. **Confirmation**: Requires acceleration to stay above threshold for minimum duration
4. **Launch Confirmed**: Sets launch flag and triggers beacon state machine

### State Machine

```
IDLE → acceleration > threshold → DETECTING → sustained > duration → CONFIRMED
  ↑                                    ↓
  └──────────── dropped below ─────────┘
```

- **IDLE**: Waiting on pad, monitoring acceleration
- **DETECTING**: Acceleration detected, confirming it's real
- **CONFIRMED**: Launch confirmed, never returns to IDLE

### Configuration Parameters

Edit `include/mpu_config.h`:

```cpp
// Launch detection threshold (m/s²)
#define LAUNCH_ACCEL_THRESHOLD  20.0    // ~2g

// Minimum sustained duration (milliseconds)
#define LAUNCH_ACCEL_DURATION   100     // 100ms

// Sensor settle time after power-on (milliseconds)
#define LAUNCH_SETTLE_TIME      2000    // 2 seconds
```

### Recommended Settings

| Rocket Type | Threshold | Duration | Notes |
|-------------|-----------|----------|-------|
| **Model rocket (C-D motors)** | 20 m/s² (2g) | 100ms | Default, good for most |
| **High power (E-G motors)** | 30 m/s² (3g) | 50ms | Higher acceleration |
| **Large HPR (H+ motors)** | 40 m/s² (4g) | 50ms | Very high acceleration |
| **Test/Sensitive** | 10 m/s² (1g) | 200ms | May trigger on handling |

### Tuning for Your Rocket

**If launch isn't detected:**
- ✓ Lower `LAUNCH_ACCEL_THRESHOLD` (try 15 or 10 m/s²)
- ✓ Reduce `LAUNCH_ACCEL_DURATION` (try 50ms)
- ✓ Check IMU orientation (Z-axis should point up)

**If false launches occur:**
- ✓ Increase `LAUNCH_ACCEL_THRESHOLD` (try 30 or 40 m/s²)
- ✓ Increase `LAUNCH_ACCEL_DURATION` (try 200ms)
- ✓ Check for vibrations during transport

## Serial Monitor Output

### Normal Operation

```
[Launch] Initializing BNO085 IMU...
[Launch] ✓ BNO08x Found!
[Launch] ✓ Linear acceleration enabled
[Launch] Settling for 2000 ms...
[Launch] ✓ Launch detection ready
```

### Launch Detection

```
[Launch] Acceleration detected: 25.4 m/s²
[Launch] ✓ LAUNCH CONFIRMED!
[Launch] Peak acceleration: 28.7 m/s²
```

### Troubleshooting Messages

```
[Launch] ✗ Failed to find BNO08x chip
```
- Check I2C wiring (SDA, SCL)
- Verify 3.3V power connection
- Try alternative I2C address (0x4B)

```
[Launch] Sensor was reset, re-enabling reports
```
- Normal - sensor automatically recovers
- May happen if power glitches occur

```
[Launch] False alarm, acceleration dropped
```
- Normal - filters out vibrations and handling
- If this happens during actual launch, reduce threshold

## Diagnostic Functions

You can add diagnostic output to your code:

```cpp
// Get current acceleration
float accel = launch_detect_get_current_accel();
Serial.print("Accel: ");
Serial.println(accel);

// Get individual axes
float x, y, z;
launch_detect_get_accel_xyz(&x, &y, &z);
Serial.print("X: "); Serial.print(x);
Serial.print(" Y: "); Serial.print(y);
Serial.print(" Z: "); Serial.println(z);

// Check IMU status
if (!launch_detect_get_imu_status()) {
    Serial.println("IMU not working!");
}
```

## Testing Launch Detection

### Bench Test (Safe)

1. Upload code with `make flash`
2. Open serial monitor with `make monitor`
3. Gently tap the board on a table
4. Watch for "Acceleration detected" messages
5. Adjust threshold if needed

### Shake Test

1. Set threshold low (10 m/s²) for testing
2. Quickly move/shake the board
3. Should trigger launch detection
4. **Remember to increase threshold for actual flight!**

### Field Test

1. Install in rocket with threshold at flight setting (20-30 m/s²)
2. Monitor serial output during motor test
3. Verify launch is detected when motor fires
4. Adjust if needed based on results

## Advantages Over Switch

| Feature | IMU | Switch |
|---------|-----|--------|
| **Reliability** | No moving parts | Can stick/fail |
| **Sensitivity** | Adjustable threshold | Fixed |
| **False triggers** | Duration check prevents | Single contact |
| **Diagnostics** | Real-time accel data | On/off only |
| **Installation** | Any orientation works | Must be accessible |
| **Data** | 3-axis acceleration | Binary state |

## IMU Mounting

### Best Practices

1. **Orientation**: Mount with Z-axis pointing toward nose cone
2. **Secure**: Use foam tape or hot glue to prevent movement
3. **Central**: Mount near center of gravity if possible
4. **Accessible**: Leave I2C pins accessible for troubleshooting
5. **Protected**: Shield from motor exhaust and heat

### Orientation Examples

```
Z-axis Up (Vertical flight):
    ↑ Nose Cone
    |
  [BNO085]  ← Z points up
    |
    ↓ Motor
```

If mounted differently, you can use a different axis for detection by modifying the code to use `accel_z` specifically instead of total magnitude.

## Power Consumption

- **Active**: ~10 mA
- **Standby**: ~1 mA

The IMU stays active continuously to monitor for launch. Power consumption is minimal compared to GPS (~30mA) and radio (~120mA when transmitting).

## Sensor Specifications

**BNO085** (Bosch BNO08x series):
- 9-axis IMU (accel + gyro + mag)
- Built-in sensor fusion
- Linear acceleration output (gravity removed)
- I2C/SPI interface
- Operating range: ±16g accelerometer
- Update rate: Up to 400Hz
- Operating temp: -40°C to +85°C

## Common Issues

### IMU not detected
- Check I2C wiring
- Verify power (3.3V)
- Try alternative address (0x4A vs 0x4B)
- Check for shorts or damaged board

### Launch not detected
- Reduce threshold (try 10-15 m/s²)
- Check IMU orientation
- Verify motor has enough thrust
- Monitor acceleration during motor burn

### False launches
- Increase threshold (try 30-40 m/s²)
- Increase duration (try 200-300ms)
- Secure IMU to prevent vibration
- Avoid handling rocket after arming

### Inconsistent detection
- Check for loose connections
- Verify IMU is securely mounted
- Add `LAUNCH_SETTLE_TIME` after power-on
- Check for power supply noise

## Comparison: Linear vs Total Acceleration

The code uses **linear acceleration** (gravity removed by sensor fusion):

**Advantages:**
- ✓ Detects only motion/thrust
- ✓ Works in any orientation
- ✓ Ignores constant 1g from gravity
- ✓ More reliable launch detection

**Alternative (raw acceleration):**
Would need to subtract gravity manually and handle orientation changes. Linear acceleration is better for this application.

## Future Enhancements

With the BNO085, you could add:

1. **Apogee Detection**: Detect when acceleration goes to zero
2. **Landing Detection**: Detect impact/touchdown
3. **Rotation Monitoring**: Use gyro data for spin rate
4. **Tilt Sensing**: Verify vertical launch
5. **Flight Phases**: Detect boost/coast/descent automatically

These would require additional code but the hardware supports it!

## Code Size Impact

- **Before IMU**: 49KB (9%)
- **With IMU**: 58KB (11%)
- **Increase**: 9KB for full IMU support

Still plenty of flash remaining (507KB total).

## References

- [BNO085 Datasheet](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- [Adafruit BNO08x Library](https://github.com/adafruit/Adafruit_BNO08x)
- [SH-2 Reference Manual](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Sensor-Hub-Transport-Protocol.pdf)
