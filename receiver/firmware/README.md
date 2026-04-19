# GPS Radio Beacon Receiver Firmware

This firmware is designed for an STM32F401 microcontroller-based receiver that tracks the location of a transmitter using GPS coordinates.

## Hardware Components
- STM32F401 Microcontroller
- QMX588L Compass Module
- NEO6M GPS Module
- RXM-433-LR RF Receiver
- SSD1309 OLED Display

## Functionality
The system receives GPS coordinates from a remote transmitter via the RF receiver, compares them with the local GPS position, and uses the compass to determine the relative direction and distance to the transmitter. The information is displayed on the OLED screen.

## Display Information
- Direction (degrees off from straight ahead)
- Distance to transmitter (meters)
- Time since last received ping

## Project Structure
- `inc/` - Header files
- `src/` - Source files
- `Makefile` - Build configuration
