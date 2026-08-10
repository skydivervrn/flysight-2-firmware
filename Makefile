# CLI build for FlySight 2 (engo-hud branch) — mirrors the STM32CubeIDE Release config.
# Produces build/FlySight.elf and build/UserApp.bin (raw image for Deploy/deploy_firmware.py).
# Self-contained: sources are globbed in-Makefile (no external file needed).
# Build:  make PREFIX=~/opt/arm-gnu-toolchain-12.3.rel1-darwin-x86_64-arm-none-eabi/bin/arm-none-eabi-

TARGET      = FlySight
BUILD_DIR   = build
LDSCRIPT    = STM32WB5MMGHX_FLASH.ld
STARTUP     = Core/Startup/startup_stm32wb5mmghx.s

# Toolchain — MUST be official Arm GNU **12.3** (ST-matching). Pitfalls (don't repeat):
#   * GCC 14.x builds INSTALL but DO NOT BOOT on this device. Never use 14.x.
#   * Homebrew arm-none-eabi-gcc ships WITHOUT newlib (no stdint.h) — unusable.
#   * Get it (no sudo) from the Arm tarball, extract to ~/opt:
#       https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-darwin-x86_64-arm-none-eabi.tar.xz
# Auto-detect ONLY a 12.3 tarball under ~/opt; override with `make PREFIX=...`.
TCDIR      ?= $(firstword $(wildcard $(HOME)/opt/arm-gnu-toolchain-12.3*-arm-none-eabi))
PREFIX     ?= $(TCDIR)/bin/arm-none-eabi-
CC          = $(PREFIX)gcc
AS          = $(PREFIX)gcc -x assembler-with-cpp
CP          = $(PREFIX)objcopy
SZ          = $(PREFIX)size

CPU         = -mcpu=cortex-m4
FPU         = -mfpu=fpv4-sp-d16
FLOAT_ABI   = -mfloat-abi=hard
MCU         = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

C_DEFS      = -DUSE_HAL_DRIVER -DSTM32WB5Mxx

C_INCLUDES = \
 -ICore/Inc \
 -IFlySight \
 -IDrivers/BSP \
 -IDrivers/STM32WBxx_HAL_Driver/Inc \
 -IDrivers/STM32WBxx_HAL_Driver/Inc/Legacy \
 -IDrivers/CMSIS/Device/ST/STM32WBxx/Include \
 -IDrivers/CMSIS/Include \
 -IFATFS/App \
 -IFATFS/Target \
 -ISTM32_WPAN/App \
 -IUSB_Device/App \
 -IUSB_Device/Target \
 -IUtilities/lpm/tiny_lpm \
 -IUtilities/sequencer \
 -IMiddlewares/Third_Party/FatFs/src \
 -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
 -IMiddlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc \
 -IMiddlewares/ST/STM32_WPAN \
 -IMiddlewares/ST/STM32_WPAN/ble \
 -IMiddlewares/ST/STM32_WPAN/ble/core \
 -IMiddlewares/ST/STM32_WPAN/ble/core/auto \
 -IMiddlewares/ST/STM32_WPAN/ble/core/template \
 -IMiddlewares/ST/STM32_WPAN/ble/svc/Inc \
 -IMiddlewares/ST/STM32_WPAN/ble/svc/Src \
 -IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread \
 -IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci \
 -IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl \
 -IMiddlewares/ST/STM32_WPAN/utilities

OPT         = -Os
# -fno-tree-loop-distribute-patterns: stop GCC 12+ rewriting init loops into memcpy/memset
# calls that run before the C runtime is ready → classic bare-metal boot crash.
# EXTRA_CFLAGS: hook for CI. The nightly build passes
#   EXTRA_CFLAGS='-DHUD_VERSION=\"0.0.15-n.abc1234\"'
# so the version shown on the glasses identifies the exact commit (see
# .github/workflows/nightly.yml). HUD_VERSION in activelook_mode0.c is wrapped
# in #ifndef precisely so this override works.
CFLAGS      = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=gnu11 -fno-tree-loop-distribute-patterns $(EXTRA_CFLAGS)
ASFLAGS     = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections

# -u _printf_float: newlib-nano drops %f/%g from printf/snprintf by default; without
# this, snprintf("%.*f",...) returns an EMPTY string (CubeIDE's "use float with printf").
LDFLAGS     = $(MCU) --specs=nano.specs -u _printf_float -T$(LDSCRIPT) -Wl,--gc-sections -static \
              -Wl,--start-group -lc -lm -Wl,--end-group -Wl,--print-memory-usage

# Source set — globbed directly in-Makefile so a FRESH CHECKOUT builds with no
# pre-generated file (build/ is gitignored, so build/all_sources.txt is never present).
# Mirrors the CubeIDE Release source list; HAL modules not enabled in
# stm32wbxx_hal_conf.h compile to (near-)empty objects, so globbing all of them is safe.
C_SOURCES  := \
  $(wildcard Core/Src/*.c) \
  $(wildcard FlySight/*.c) \
  $(wildcard Drivers/STM32WBxx_HAL_Driver/Src/*.c) \
  $(wildcard Drivers/BSP/*.c) \
  $(wildcard FATFS/App/*.c) $(wildcard FATFS/Target/*.c) \
  $(wildcard USB_Device/App/*.c) $(wildcard USB_Device/Target/*.c) \
  $(wildcard STM32_WPAN/App/*.c) $(wildcard STM32_WPAN/Target/*.c) \
  $(wildcard Utilities/lpm/tiny_lpm/*.c) $(wildcard Utilities/sequencer/*.c) \
  $(wildcard Middlewares/Third_Party/FatFs/src/*.c) \
  Middlewares/Third_Party/FatFs/src/option/syscall.c \
  $(wildcard Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c) \
  $(wildcard Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/*.c) \
  $(wildcard Middlewares/ST/STM32_WPAN/ble/core/auto/*.c) \
  $(wildcard Middlewares/ST/STM32_WPAN/ble/core/template/*.c) \
  $(wildcard Middlewares/ST/STM32_WPAN/ble/svc/Src/*.c) \
  $(wildcard Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/*.c) \
  $(wildcard Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/*.c) \
  $(wildcard Middlewares/ST/STM32_WPAN/utilities/*.c)
OBJECTS    := $(addprefix $(BUILD_DIR)/obj/,$(C_SOURCES:.c=.o))
OBJECTS    += $(BUILD_DIR)/obj/startup.o

# Pre-create all object subdirectories at parse time (avoids a parallel-make race
# where two recipes try to mkdir the same dir and the .d dependency write fails).
$(shell mkdir -p $(sort $(dir $(OBJECTS))))

all: prebuild $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/UserApp.bin

prebuild:
	@cd Scripts 2>/dev/null && ../prebuild.sh 2>/dev/null || (cd $(CURDIR) && GIT_TAG=$$(git describe --tags --always) && printf '#ifndef VERSION_H\n#define VERSION_H\n\n#define GIT_TAG "%s"\n\n#endif // VERSION_H\n' "$$GIT_TAG" > FlySight/version.h)
	@echo "version.h:"; cat FlySight/version.h | grep GIT_TAG

$(BUILD_DIR)/obj/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MMD -MP $< -o $@

$(BUILD_DIR)/obj/startup.o: $(STARTUP)
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/UserApp.bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@
	@echo "Wrote $@"

clean:
	rm -rf $(BUILD_DIR)/obj $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/UserApp.bin

.PHONY: all clean prebuild
-include $(shell find $(BUILD_DIR)/obj -name '*.d' 2>/dev/null)
