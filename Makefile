# ─────────────────────────────────────────────────────────────────────────────
# ESP32-S3 e-Paper MP3 Player — Build & Flash
# ─────────────────────────────────────────────────────────────────────────────

SKETCH     := ESP32_EPAPER_PLAYER
BUILD_DIR  := /tmp/epaper_build
PORT       ?= /dev/cu.usbmodem21201
BAUD       ?= 921600

FQBN := esp32:esp32:esp32s3:FlashSize=8M,FlashMode=qio,PartitionScheme=custom,\
PSRAM=opi,CPUFreq=240,DebugLevel=none,USBMode=hwcdc,CDCOnBoot=cdc

ARDUINO := arduino-cli
ESPTOOL := $(HOME)/Library/Arduino15/packages/esp32/tools/esptool_py/5.1.0/esptool

BOOTLOADER := $(BUILD_DIR)/$(SKETCH).ino.bootloader.bin
PARTITIONS := $(BUILD_DIR)/$(SKETCH).ino.partitions.bin
FIRMWARE   := $(BUILD_DIR)/$(SKETCH).ino.bin

.PHONY: all build flash flash-model flash-all clean port-detect help

all: build

## build  — compile the sketch
build:
	$(ARDUINO) compile \
	  --fqbn "$(FQBN)" \
	  --output-dir $(BUILD_DIR) \
	  $(SKETCH)

## flash  — flash bootloader + partitions + firmware
flash:
	$(ESPTOOL) \
	  --chip esp32s3 \
	  --port $(PORT) \
	  --baud $(BAUD) \
	  write-flash \
	  --flash-mode keep \
	  --flash-freq keep \
	  --flash-size keep \
	  0x0000  $(BOOTLOADER) \
	  0x8000  $(PARTITIONS) \
	  0x10000 $(FIRMWARE)

## flash-model  — flash voice model binary (srmodels.bin) to the model partition
flash-model:
	$(ESPTOOL) \
	  --chip esp32s3 \
	  --port $(PORT) \
	  --baud $(BAUD) \
	  write-flash \
	  --flash-mode keep \
	  --flash-freq keep \
	  --flash-size keep \
	  0x400000 $(SKETCH)/srmodels.bin

## flash-all  — flash firmware + model in one pass (first-time setup)
flash-all:
	$(ESPTOOL) \
	  --chip esp32s3 \
	  --port $(PORT) \
	  --baud $(BAUD) \
	  write-flash \
	  --flash-mode keep \
	  --flash-freq keep \
	  --flash-size keep \
	  0x0000   $(BOOTLOADER) \
	  0x8000   $(PARTITIONS) \
	  0x10000  $(FIRMWARE) \
	  0x400000 $(SKETCH)/srmodels.bin

## build-flash  — compile then flash firmware in one step
build-flash: build flash

## clean  — remove build artefacts
clean:
	rm -rf $(BUILD_DIR)

## port-detect  — show connected ESP32 serial ports
port-detect:
	@ls /dev/cu.usbmodem* 2>/dev/null || echo "No usbmodem device found"

## help  — list available targets
help:
	@grep -E '^##' Makefile | sed 's/## /  make /'
