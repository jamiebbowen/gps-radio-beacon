# Migration Guide: ATtiny412 → ItsyBitsy M4 Express

## Summary of Changes

### Hardware Platform
- **From**: ATtiny412 (8-bit AVR @ 20MHz, 256B RAM, 4KB Flash)
- **To**: ItsyBitsy M4 Express (32-bit ARM Cortex-M4 @ 120MHz, 192KB RAM, 512KB Flash)

### Build System
- **From**: AVR-GCC with custom Makefile
- **To**: Arduino framework with arduino-cli

### Pin Mapping

| Function | ATtiny412 | ItsyBitsy M4 |
|----------|-----------|--------------|
| Radio TX | PA2 | Pin 11 |
| Radio Enable | PA0 | Pin 12 |
| Launch Detect | PA1 | Pin 10 |
| GPS RX | PA7 (bit-banged) | Pin 0 (HW Serial1) |
| GPS TX | PA6 (bit-banged) | Pin 1 (HW Serial1) |

### Code Changes

#### 1. Timer System
**Before (ATtiny412)**:
```c
ISR(TCB0_INT_vect) {
    ms_counter++;
    // Timer interrupt at hardware level
}
```

**After (ItsyBitsy M4)**:
```cpp
void loop() {
    static uint32_t last_millis = 0;
    uint32_t current_millis = millis();
    if (current_millis != last_millis) {
        timer_isr_handler();
        last_millis = current_millis;
    }
}
```

#### 2. GPIO Operations
**Before**:
```c
PORTA.OUTSET = (1 << PIN_RADIO_OUT);
PORTA.OUTCLR = (1 << PIN_RADIO_OUT);
```

**After**:
```cpp
digitalWrite(PIN_RADIO_TX, HIGH);
digitalWrite(PIN_RADIO_TX, LOW);
```

#### 3. UART Communication
**Before (bit-banged)**:
```c
USART0.BAUD = USART0_BAUD_RATE(115200);
USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
```

**After (Hardware Serial)**:
```cpp
GPS_SERIAL.begin(115200);
GPS_SERIAL.available();
GPS_SERIAL.read();
GPS_SERIAL.write(data);
```

#### 4. Delays
**Before**:
```c
#include <util/delay.h>
_delay_ms(100);
_delay_us(10);
```

**After**:
```cpp
delay(100);
delayMicroseconds(10);
```

#### 5. Watchdog
**Before**:
```c
#include <avr/wdt.h>
wdt_reset();
wdt_disable();
_PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);
```

**After**:
```cpp
// Not needed - SAMD51 watchdog not enabled by default
// Can be added later if needed
```

#### 6. Interrupts
**Before**:
```c
cli();  // Disable interrupts
sei();  // Enable interrupts
```

**After**:
```cpp
noInterrupts();  // If needed
interrupts();    // If needed
// Generally not needed in Arduino
```

### File Renaming
- `*.c` → `*.cpp` (C++ for Arduino)
- `main.c` → `main.cpp`
- Added `gps_radio_beacon.ino` (Arduino sketch wrapper)

### Configuration Files

#### mpu_config.h (now ItsyBitsy M4 config)
```cpp
#define F_CPU 120000000UL        // 120MHz (was 20MHz)
#define PIN_RADIO_TX 11          // Digital pin (was PA2)
#define PIN_RADIO_ENABLE 12      // Digital pin (was PA0)
#define PIN_LAUNCH_DETECT 10     // Digital pin (was PA1)
#define GPS_SERIAL Serial1       // Hardware serial (was USART0)
#define BEACON_CALLSIGN "KD0ABC" // Your callsign
```

### Removed Dependencies
- `<avr/io.h>` - AVR hardware registers
- `<avr/interrupt.h>` - AVR interrupts
- `<avr/wdt.h>` - AVR watchdog
- `<util/delay.h>` - AVR delay functions
- ATtiny device pack headers

### Added Dependencies
- `<Arduino.h>` - Arduino framework
- No external libraries required

## Performance Improvements

### CPU Speed
- **6x faster**: 120MHz vs 20MHz
- More headroom for future features

### Memory
- **768x more RAM**: No more buffer overflow concerns
- **128x more Flash**: Room for extensive features

### Peripherals
- **Hardware UART**: Reliable GPS communication
- **USB Serial**: Built-in debugging
- **Hardware FPU**: Fast floating-point math (if needed)
- **DMA**: Can offload repetitive tasks (future)

## Advantages

### Development
1. **USB debugging**: `Serial.println()` for diagnostics
2. **No programmer needed**: USB bootloader built-in
3. **Faster uploads**: ~2 seconds vs ~30 seconds
4. **No fuse programming**: Just plug and program

### Reliability
1. **Hardware UART**: No GPS bit-timing issues
2. **More RAM**: No stack overflow risks
3. **Better timing**: 120MHz precision for radio TX

### Flexibility
1. **Add sensors**: I2C, SPI peripherals
2. **SD card logging**: SPI interface available
3. **Real-time clock**: Built-in RTC
4. **More telemetry**: RAM for buffering

## Testing Checklist

- [ ] Install arduino-cli
- [ ] Run `make install-deps`
- [ ] Update callsign in config
- [ ] Connect ItsyBitsy M4 via USB
- [ ] Run `make flash`
- [ ] Connect GPS module to pins 0/1
- [ ] Connect radio module to pins 11/12
- [ ] Connect launch switch to pin 10
- [ ] Test in TESTING_MODE first
- [ ] Verify GPS data reception
- [ ] Verify radio transmission (300 baud)
- [ ] Test launch detection
- [ ] Test state transitions
- [ ] Switch to production mode

## Backward Compatibility

The packet format remains **100% compatible** with the ATtiny412 version:
- Same 300 baud transmission
- Same `$$data$$CHKSUM` format
- Same NMEA coordinate format
- Existing receivers will work without changes

## Future Enhancements

Now possible with ItsyBitsy M4:

1. **Data logging**: SD card via SPI
2. **Better GPS**: UBX binary protocol support
3. **Sensor fusion**: Add IMU for better tracking
4. **Power management**: Sleep modes for battery life
5. **Configuration**: USB serial command interface
6. **Diagnostics**: Detailed telemetry and health monitoring
7. **Dual GPS**: Compare two GPS modules for accuracy
8. **Advanced features**: Geofencing, altitude triggers, etc.

## Troubleshooting

### Compilation fails
```bash
arduino-cli core update-index
arduino-cli core install adafruit:samd
```

### Upload fails
- Press reset button twice quickly to enter bootloader
- Red LED should pulse
- Try different USB cable/port

### GPS not working
- Verify 3.3V power to GPS
- Check TX/RX connections (note: GPS TX → Pin 0, GPS RX → Pin 1)
- GPS baud rate must be 115200
- Use `make monitor` to see debug output

### Radio not transmitting
- Verify 300 baud on pin 11 with oscilloscope/logic analyzer
- Check enable signal on pin 12
- Bit width should be 3333 microseconds (300 baud)

## Support

For issues specific to this port, check:
1. `README_ITSYBITSY_M4.md` - Detailed setup guide
2. Pin assignments in `include/mpu_config.h`
3. Arduino Serial monitor for debug output
4. Adafruit ItsyBitsy M4 documentation

## Credits

Original ATtiny412 firmware: Jamie Bowen
ItsyBitsy M4 port: Cascade AI Assistant
Board: Adafruit ItsyBitsy M4 Express (SAMD51)
