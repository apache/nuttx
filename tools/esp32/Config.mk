############################################################################
# tools/esp32/Config.mk
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# These are the macros that will be used in the NuttX make system to compile
# and assemble source files and to insert the resulting object files into an
# archive.  These replace the default definitions at tools/Config.mk

ifeq ($(CONFIG_ESP32_FLASH_2M),y)
	FLASH_SIZE := 2MB
else ifeq ($(CONFIG_ESP32_FLASH_4M),y)
	FLASH_SIZE := 4MB
else ifeq ($(CONFIG_ESP32_FLASH_8M),y)
	FLASH_SIZE := 8MB
else ifeq ($(CONFIG_ESP32_FLASH_16M),y)
	FLASH_SIZE := 16MB
endif

ifeq ($(CONFIG_ESP32_FLASH_MODE_DIO),y)
	FLASH_MODE := dio
else ifeq ($(CONFIG_ESP32_FLASH_MODE_DOUT),y)
	FLASH_MODE := dout
else ifeq ($(CONFIG_ESP32_FLASH_MODE_QIO),y)
	FLASH_MODE := qio
else ifeq ($(CONFIG_ESP32_FLASH_MODE_QOUT),y)
	FLASH_MODE := qout
endif

ifeq ($(CONFIG_ESP32_FLASH_FREQ_80M),y)
	FLASH_FREQ := 80m
else ifeq ($(CONFIG_ESP32_FLASH_FREQ_40M),y)
	FLASH_FREQ := 40m
else ifeq ($(CONFIG_ESP32_FLASH_FREQ_26M),y)
	FLASH_FREQ := 26m
else ifeq ($(CONFIG_ESP32_FLASH_FREQ_20M),y)
	FLASH_FREQ := 20m
endif

ifeq ($(CONFIG_ESP32_FLASH_DETECT),y)
	ESPTOOL_WRITEFLASH_OPTS := -fs detect -fm dio -ff $(FLASH_FREQ)
else
	ESPTOOL_WRITEFLASH_OPTS := -fs $(FLASH_SIZE) -fm dio -ff $(FLASH_FREQ)
endif

# ESPTOOL_BINDIR -- Directory for the Bootloader and Partition Table binary
#                   files. If not provided, assume the current directory.

ESPTOOL_BINDIR ?= .

# Configure the variables according to build environment

ifeq ($(CONFIG_ESP32_BOOTLOADER_BUILD),y)
	BL_OFFSET       := 0x1000
	BOOTLOADER      := nuttx.bin
	FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)
	ESPTOOL_BINS    := $(FLASH_BL)
else ifeq ($(CONFIG_ESP32_APP_FORMAT_LEGACY),y)
	BL_OFFSET       := 0x1000
	PT_OFFSET       := 0x8000
	APP_OFFSET      := 0x10000
	BOOTLOADER      := $(ESPTOOL_BINDIR)/bootloader-esp32.bin
	PARTITION_TABLE := $(ESPTOOL_BINDIR)/partition-table-esp32.bin
	APP_IMAGE       := nuttx.bin
	FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)
	FLASH_PT        := $(PT_OFFSET) $(PARTITION_TABLE)
	FLASH_APP       := $(APP_OFFSET) $(APP_IMAGE)
	ESPTOOL_BINS    := $(FLASH_BL) $(FLASH_PT) $(FLASH_APP)
else ifeq ($(CONFIG_ESP32_APP_FORMAT_MCUBOOT),y)
	BL_OFFSET       := 0x1000
	BOOTLOADER      := $(ESPTOOL_BINDIR)/mcuboot-esp32.bin
	FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)

	ifeq ($(CONFIG_ESP32_ESPTOOL_TARGET_PRIMARY),y)
		VERIFIED    := --confirm
		APP_OFFSET  := $(CONFIG_ESP32_OTA_PRIMARY_SLOT_OFFSET)
	else ifeq ($(CONFIG_ESP32_ESPTOOL_TARGET_SECONDARY),y)
		VERIFIED    :=
		APP_OFFSET  := $(CONFIG_ESP32_OTA_SECONDARY_SLOT_OFFSET)
	endif

	APP_IMAGE       := nuttx_signed.bin
	FLASH_APP       := $(APP_OFFSET) $(APP_IMAGE)
	ESPTOOL_BINS    := $(FLASH_BL) $(FLASH_APP)
endif

ifeq ($(CONFIG_ESP32_QEMU_IMAGE),y)
	MK_QEMU_IMG=$(TOPDIR)/tools/esp32/mk_qemu_img.sh -b $(BOOTLOADER) -p $(PARTITION_TABLE)
else
	MK_QEMU_IMG=
endif

# POSTBUILD -- Perform post build operations

ifeq ($(CONFIG_ESP32_APP_FORMAT_MCUBOOT),y)
define POSTBUILD
	$(Q) echo "MKIMAGE: ESP32 binary"
	$(Q) if ! imgtool version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "imgtool not found.  Please run: \"pip install imgtool\""; \
		echo ""; \
		echo "Run make again to create the nuttx_signed.bin image."; \
		exit 1; \
	fi
	imgtool sign --pad --pad-sig $(VERIFIED) --align 4 -v 0 \
		-H $(CONFIG_ESP32_APP_MCUBOOT_HEADER_SIZE) --pad-header \
		-S $(CONFIG_ESP32_OTA_SLOT_SIZE) \
		nuttx.bin nuttx_signed.bin
	$(Q) echo "Generated: nuttx_signed.bin (MCUboot compatible)"
endef
else
define POSTBUILD
	$(Q) echo "MKIMAGE: ESP32 binary"
	$(Q) if ! esptool.py version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "esptool.py not found.  Please run: \"pip install esptool\""; \
		echo ""; \
		echo "Run make again to create the nuttx.bin image."; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration for the ESP32 chip."; \
		exit 1; \
	fi
	$(eval ESPTOOL_ELF2IMG_OPTS := -fs $(FLASH_SIZE) -fm $(FLASH_MODE) -ff $(FLASH_FREQ))
	esptool.py -c esp32 elf2image $(ESPTOOL_ELF2IMG_OPTS) -o nuttx.bin nuttx
	$(Q) echo "Generated: nuttx.bin (ESP32 compatible)"
	$(Q) $(MK_QEMU_IMG)
endef
endif

# ESPTOOL_BAUD -- Serial port baud rate used when flashing/reading via esptool.py

ESPTOOL_BAUD ?= 921600

# DOWNLOAD -- Download binary image via esptool.py

define DOWNLOAD
	$(Q) if [ -z $(ESPTOOL_PORT) ]; then \
		echo "DOWNLOAD error: Missing serial port device argument."; \
		echo "USAGE: make download ESPTOOL_PORT=<port> [ ESPTOOL_BAUD=<baud> ] [ ESPTOOL_BINDIR=<dir> ]"; \
		exit 1; \
	fi
	esptool.py -c esp32 -p $(ESPTOOL_PORT) -b $(ESPTOOL_BAUD) write_flash $(ESPTOOL_WRITEFLASH_OPTS) $(ESPTOOL_BINS)
endef
