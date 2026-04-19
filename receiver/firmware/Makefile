# GPS Radio Beacon Receiver Firmware Makefile

# Project name
PROJECT = gps_radio_beacon_receiver

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build
BIN_DIR = bin

# Toolchain
TOOLCHAIN_PREFIX = arm-none-eabi
CC = $(TOOLCHAIN_PREFIX)-gcc
AS = $(TOOLCHAIN_PREFIX)-as
LD = $(TOOLCHAIN_PREFIX)-gcc
OBJCOPY = $(TOOLCHAIN_PREFIX)-objcopy
SIZE = $(TOOLCHAIN_PREFIX)-size

# CPU/MCU flags
CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Startup file
STARTUP = lib/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f401xc.s

# Source files
C_SOURCES = \
	$(SRC_DIR)/main.c \
	$(SRC_DIR)/button.c \
	$(SRC_DIR)/compass.c \
	$(SRC_DIR)/display.c \
	$(SRC_DIR)/gps.c \
	$(SRC_DIR)/gps_parser.c \
	$(SRC_DIR)/math_utils.c \
	$(SRC_DIR)/lora.c \
	$(SRC_DIR)/rf_receiver.c \
	$(SRC_DIR)/rf_parser.c \
	$(SRC_DIR)/sd_card.c \
	$(SRC_DIR)/sd_diskio.c \
	$(SRC_DIR)/ff_gen_drv.c \
	$(SRC_DIR)/display_modes/gps_mode.c \
	$(SRC_DIR)/display_modes/compass_mode.c \
	$(SRC_DIR)/display_modes/rf_mode.c \
	$(SRC_DIR)/display_modes/navigation_mode.c \
	$(SRC_DIR)/display_modes/test_pattern.c \
	$(SRC_DIR)/display_modes/i2c_devices.c \
	$(SRC_DIR)/display_modes/imu_test_mode.c \
	$(SRC_DIR)/display_modes/sd_card_mode.c \
	lib/STM32CubeF4/Middlewares/Third_Party/FatFs/src/ff.c \
	lib/STM32CubeF4/Middlewares/Third_Party/FatFs/src/diskio.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
	lib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
	lib/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

# Include paths
INCLUDES = \
  -I$(INC_DIR) \
  -Ilib/STM32CubeF4/Drivers/CMSIS/Include \
  -Ilib/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
  -Ilib/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc \
  -Ilib/STM32CubeF4/Middlewares/Third_Party/FatFs/src

# Compiler flags
CFLAGS = $(CPU) -Wall -O2 -g3
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -DUSE_HAL_DRIVER -DSTM32F401xC
CFLAGS += $(INCLUDES)

# Linker script
LDSCRIPT = STM32F401CCUx_FLASH.ld

# Linker flags
LDFLAGS = $(CPU)
LDFLAGS += -specs=nano.specs -specs=nosys.specs -u _printf_float -u _scanf_float
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--cref
LDFLAGS += -T $(LDSCRIPT)

# Libraries
LIBS = -lc -lm

# Output files
ELF = $(BIN_DIR)/$(PROJECT).elf
BIN = $(BIN_DIR)/$(PROJECT).bin
HEX = $(BIN_DIR)/$(PROJECT).hex

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(BUILD_DIR)/startup_stm32f401xc.o
vpath %.c $(sort $(dir $(C_SOURCES)))

# Default target
all: $(BUILD_DIR) $(BIN_DIR) $(ELF) $(BIN) $(HEX)

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Create bin directory
$(BIN_DIR):
	mkdir -p $@

# Build rules
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "Compiling $<"
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/startup_stm32f401xc.o: $(STARTUP) | $(BUILD_DIR)
	@echo "Assembling $<"
	@$(CC) -c $(CPU) -x assembler-with-cpp -o $@ $<

# Link object files
$(ELF): $(OBJECTS) Makefile | $(BIN_DIR)
	@echo "Linking $@"
	$(LD) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SIZE) $@

# Create binary file
$(BIN): $(ELF) | $(BIN_DIR)
	$(OBJCOPY) -O binary $< $@

# Create hex file
$(HEX): $(ELF) | $(BIN_DIR)
	$(OBJCOPY) -O ihex $< $@

# Flash the firmware to the device using DFU mode
flash: $(BIN)
	dfu-util -a 0 -s 0x08000000:leave -D $(BIN)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(BIN_DIR)

# Clean and rebuild
rebuild: clean all

.PHONY: all clean rebuild flash
