# Makefile for ItsyBitsy M4 Express GPS Radio Beacon
# Using Arduino CLI for build system

# Board configuration
BOARD_FQBN = adafruit:samd:adafruit_itsybitsy_m4
PORT ?= /dev/ttyACM0

# Arduino CLI
ARDUINO_CLI = arduino-cli

# Build directory
BUILD_DIR = build

# Per-airframe build: rocket ID and channel are independent (any ID can fly
# on any channel). Set either or both without editing any source:
#   make flash ID=3 CHANNEL=5   -> -DROCKET_ID=3 -DLORA_CHANNEL=5
#   make flash CHANNEL=5        -> only the channel, ID keeps its default
# Channels are 0-7 (433.00-434.75 MHz, 250 kHz spacing; see mpu_config.h).
# ROCKET=n is kept as shorthand for ID=n CHANNEL=n.
# Each ID/channel combo gets its own build dir so objects from one build
# never leak into another airframe's binary.
ifdef ROCKET
ID ?= $(ROCKET)
CHANNEL ?= $(ROCKET)
endif
TX_DEFS =
ifdef ID
TX_DEFS += -DROCKET_ID=$(ID)
endif
ifdef CHANNEL
TX_DEFS += -DLORA_CHANNEL=$(CHANNEL)
endif
ifneq ($(strip $(TX_DEFS)),)
BUILD_DIR = build/id$(if $(ID),$(ID),d)-ch$(if $(CHANNEL),$(CHANNEL),d)
BUILD_PROPS = --build-property "compiler.cpp.extra_flags=$(strip $(TX_DEFS))"
endif

# The sketch is firmware.ino (arduino-cli compiles every .cpp/.c in this
# directory alongside it). Historical note: a generated gps_radio_beacon.ino
# wrapper containing '#include "main.cpp"' used to live here - it resolved
# to the Arduino CORE's main.cpp via the include path and only linked by
# accident. Do not reintroduce it.

# Default target
all: compile flash

# Ensure arduino-cli is installed
check-tools:
	@which $(ARDUINO_CLI) > /dev/null || (echo "arduino-cli not found. Install from https://arduino.github.io/arduino-cli/" && exit 1)
	@echo "✓ arduino-cli found"

# Install required cores and libraries
install-deps: check-tools
	@echo "Installing Adafruit SAMD board support..."
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install adafruit:samd
	@echo "✓ Dependencies installed"

# Create build directory
prepare:
	@mkdir -p $(BUILD_DIR)

# Compile the sketch
compile: check-tools prepare
	@echo "Compiling for ItsyBitsy M4 Express..."
	$(ARDUINO_CLI) compile --fqbn $(BOARD_FQBN) --build-path $(BUILD_DIR) $(BUILD_PROPS) .

# Upload to board
upload: compile
	@echo "Uploading to ItsyBitsy M4 Express on $(PORT)..."
	$(ARDUINO_CLI) upload --fqbn $(BOARD_FQBN) --port $(PORT) --input-dir $(BUILD_DIR)

# Alternative upload using custom script (works around snap library issues)
upload-direct: compile
	@echo "Using direct upload method..."
	./upload.sh $(PORT)

# Combined flash target (compile + upload)
flash: upload

# Alternative flash using direct method
flash-direct: upload-direct

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)
	rm -f *.o *.elf *.hex

# Monitor serial output
monitor:
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=115200

# Alternative monitor using screen (works around snap issues)
monitor-screen:
	@echo "Starting serial monitor (Ctrl+A then K to exit)..."
	screen $(PORT) 115200

# Alternative monitor using minicom (works around snap issues)
monitor-minicom:
	minicom -D $(PORT) -b 115200

# Help target
help:
	@echo "Available targets:"
	@echo "  all               - Compile the project (default)"
	@echo "  compile           - Compile for ItsyBitsy M4 Express"
	@echo "  upload            - Upload to ItsyBitsy M4 Express"
	@echo "  upload-direct     - Upload using direct method (snap workaround)"
	@echo "  flash             - Compile and upload (combined)"
	@echo "  flash-direct      - Compile and upload using direct method"
	@echo "  monitor           - Open serial monitor at 115200 baud"
	@echo "  monitor-screen    - Open serial monitor using screen (recommended)"
	@echo "  monitor-minicom   - Open serial monitor using minicom"
	@echo "  clean             - Remove build artifacts"
	@echo "  install-deps      - Install Arduino CLI and board support"
	@echo "  check-tools       - Verify arduino-cli is installed"
	@echo "  help              - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make flash-direct       - Build and upload (recommended)"
	@echo "  make monitor            - View serial output"
	@echo "  make PORT=/dev/ttyACM1 flash-direct  - Upload to specific port"
	@echo "  make flash ID=3 CHANNEL=5  - Rocket ID 3 transmitting on CH5 (434.25)"
	@echo "  make flash CHANNEL=2       - Default ID, CH2 (433.50)"
	@echo "  make flash ROCKET=2        - Shorthand for ID=2 CHANNEL=2"
	@echo ""
	@echo "Note: ItsyBitsy M4 uses native USB, no external programmer needed"

.PHONY: all compile upload upload-direct flash flash-direct monitor monitor-screen monitor-minicom clean install-deps check-tools prepare help
