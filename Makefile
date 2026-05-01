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

.PHONY: all build flash flash-model flash-model-hiesp flash-all build-flash clean port-detect help monitor

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
	  write_flash \
	  0x0000  $(BOOTLOADER) \
	  0x8000  $(PARTITIONS) \
	  0x10000 $(FIRMWARE)

ARDUINO_LIBS := $(HOME)/Library/Arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.5-9bb7aa84-v2/esp32s3/esp_sr

## flash-model  — flash voice model binary (srmodels.bin) to the model partition
## Uses custom srmodels.bin from sketch dir if present; falls back to official Hi ESP model.
flash-model:
	$(ESPTOOL) \
	  --chip esp32s3 \
	  --port $(PORT) \
	  --baud $(BAUD) \
	  write_flash \
	  0x400000 $(if $(wildcard $(SKETCH)/srmodels.bin),$(SKETCH)/srmodels.bin,$(ARDUINO_LIBS)/srmodels.bin)

## flash-model-hiesp  — flash official Hi ESP + English command model (wn9_hiesp + mn5q8_en)
flash-model-hiesp:
	$(ESPTOOL) \
	  --chip esp32s3 \
	  --port $(PORT) \
	  --baud $(BAUD) \
	  write_flash \
	  0x400000 $(ARDUINO_LIBS)/srmodels.bin

## flash-all  — flash firmware + model in one pass (first-time setup)
## Uses $(SKETCH)/srmodels.bin if present, otherwise official Hi ESP model
flash-all:
	$(ESPTOOL) \
	  --chip esp32s3 \
	  --port $(PORT) \
	  --baud $(BAUD) \
	  write_flash \
	  0x0000   $(BOOTLOADER) \
	  0x8000   $(PARTITIONS) \
	  0x10000  $(FIRMWARE) \
	  0x400000 $(if $(wildcard $(SKETCH)/srmodels.bin),$(SKETCH)/srmodels.bin,$(ARDUINO_LIBS)/srmodels.bin)

## build-flash  — compile then flash firmware in one step
build-flash: build flash

## clean  — remove build artefacts
clean:
	rm -rf $(BUILD_DIR)

## port-detect  — show connected ESP32 serial ports
port-detect:
	@ls /dev/cu.usbmodem* 2>/dev/null || echo "No usbmodem device found"

## monitor  — open serial monitor at 921600 baud
monitor:
	@python3 -c "import serial, time, sys; s=serial.Serial('$(PORT       ?= /dev/cu.usbmodem21201

## help  — list available targets
help:
	@grep -E '^##' Makefile | sed 's/## /  make /'
