############################################################################
# tools/esp32s3/Config.mk
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

ifeq ($(CONFIG_ESP32S3_FLASH_4M),y)
	FLASH_SIZE := 4MB
else ifeq ($(CONFIG_ESP32S3_FLASH_8M),y)
	FLASH_SIZE := 8MB
else ifeq ($(CONFIG_ESP32S3_FLASH_16M),y)
	FLASH_SIZE := 16MB
endif

ifeq ($(CONFIG_ESP32S3_FLASH_MODE_DIO),y)
	FLASH_MODE := dio
else ifeq ($(CONFIG_ESP32S3_FLASH_MODE_DOUT),y)
	FLASH_MODE := dout
else ifeq ($(CONFIG_ESP32S3_FLASH_MODE_QIO),y)
	FLASH_MODE := qio
else ifeq ($(CONFIG_ESP32S3_FLASH_MODE_QOUT),y)
	FLASH_MODE := qout
endif

ifeq ($(CONFIG_ESP32S3_FLASH_FREQ_120M),y)
	FLASH_FREQ := 120m
else ifeq ($(CONFIG_ESP32S3_FLASH_FREQ_80M),y)
	FLASH_FREQ := 80m
else ifeq ($(CONFIG_ESP32S3_FLASH_FREQ_40M),y)
	FLASH_FREQ := 40m
else ifeq ($(CONFIG_ESP32S3_FLASH_FREQ_20M),y)
	FLASH_FREQ := 20m
endif

ifeq ($(CONFIG_ESP32S3_FLASH_DETECT),y)
	ESPTOOL_WRITEFLASH_OPTS := -fs detect -fm dio -ff $(FLASH_FREQ)
else
	ESPTOOL_WRITEFLASH_OPTS := -fs $(FLASH_SIZE) -fm dio -ff $(FLASH_FREQ)
endif

ESPTOOL_FLASH_OPTS := -fs $(FLASH_SIZE) -fm $(FLASH_MODE) -ff $(FLASH_FREQ)

# Configure the variables according to build environment

ifdef ESPTOOL_BINDIR
	ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)
		BL_OFFSET       := 0x0
		PT_OFFSET       := $(CONFIG_ESP32S3_PARTITION_TABLE_OFFSET)
		BOOTLOADER      := $(ESPTOOL_BINDIR)/bootloader-esp32s3.bin
		PARTITION_TABLE := $(ESPTOOL_BINDIR)/partition-table-esp32s3.bin
		FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)
		FLASH_PT        := $(PT_OFFSET) $(PARTITION_TABLE)
		ESPTOOL_BINS    := $(FLASH_BL) $(FLASH_PT)
	else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)
		BL_OFFSET       := 0x0000
		BOOTLOADER      := $(ESPTOOL_BINDIR)/mcuboot-esp32s3.bin
		FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)
		ESPTOOL_BINS    := $(FLASH_BL)
	endif
endif

ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)
	APP_OFFSET     := 0x10000
	APP_IMAGE      := nuttx.bin
	FLASH_APP      := $(APP_OFFSET) $(APP_IMAGE)
else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)
	ifeq ($(CONFIG_ESP32S3_ESPTOOL_TARGET_PRIMARY),y)
		VERIFIED   := --confirm
		APP_OFFSET := $(CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET)
	else ifeq ($(CONFIG_ESP32S3_ESPTOOL_TARGET_SECONDARY),y)
		VERIFIED   :=
		APP_OFFSET := $(CONFIG_ESP32S3_OTA_SECONDARY_SLOT_OFFSET)
	endif

	APP_IMAGE      := nuttx.bin
	FLASH_APP      := $(APP_OFFSET) $(APP_IMAGE)
	IMGTOOL_ALIGN_ARGS := --align 4
	IMGTOOL_SIGN_ARGS  := --pad $(VERIFIED) $(IMGTOOL_ALIGN_ARGS) -v 0 -s auto \
		-H $(CONFIG_ESP32S3_APP_MCUBOOT_HEADER_SIZE) --pad-header \
		-S $(CONFIG_ESP32S3_OTA_SLOT_SIZE)
endif

ESPTOOL_BINS += $(FLASH_APP)

ifeq ($(CONFIG_BUILD_PROTECTED),y)
	ESPTOOL_BINS += $(CONFIG_ESP32S3_USER_IMAGE_OFFSET) nuttx_user.bin
endif

# MERGEBIN -- Merge raw binary files into a single file

define MERGEBIN
	$(Q) if [ -z $(ESPTOOL_BINDIR) ]; then \
		echo "MERGEBIN error: Missing argument for binary files directory."; \
		echo "USAGE: make ESPTOOL_BINDIR=<dir>"; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration for the ESP32-S3 chip."; \
		exit 1; \
	fi
	esptool.py -c esp32s3 merge_bin --output nuttx.merged.bin $(ESPTOOL_FLASH_OPTS) $(ESPTOOL_BINS)
	$(Q) echo nuttx.merged.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.merged.bin"
endef

# MKIMAGE -- Convert an ELF file into a compatible binary file

ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)
define MKIMAGE
	$(Q) echo "MKIMAGE: ESP32-S3 binary"
	$(Q) if ! esptool.py version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "esptool.py not found.  Please run: \"pip install esptool\""; \
		echo ""; \
		echo "Run make again to create the nuttx.bin image."; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration for the ESP32-S3 chip."; \
		exit 1; \
	fi
	esptool.py -c esp32s3 elf2image $(ESPTOOL_FLASH_OPTS) -o nuttx.bin nuttx
	$(Q) echo nuttx.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.bin (ESP32-S3 compatible)"
endef
else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)
define MKIMAGE
	$(Q) echo "MKIMAGE: ESP32-S3 binary"
	$(Q) if ! imgtool version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "imgtool not found.  Please run: \"pip install imgtool\""; \
		echo ""; \
		echo "Run make again to create the nuttx.bin image."; \
		exit 1; \
	fi
	imgtool sign $(IMGTOOL_SIGN_ARGS) nuttx.hex nuttx.bin
	$(Q) echo nuttx.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.bin (MCUboot compatible)"
endef
endif

# POSTBUILD -- Perform post build operations

define POSTBUILD
	$(call MKIMAGE)
	$(if $(CONFIG_ESP32S3_MERGE_BINS),$(call MERGEBIN))
endef

# ESPTOOL_BAUD -- Serial port baud rate used when flashing/reading via esptool.py

ESPTOOL_BAUD ?= 921600

# FLASH -- Download a binary image via esptool.py

define FLASH
	$(Q) if [ -z $(ESPTOOL_PORT) ]; then \
		echo "FLASH error: Missing serial port device argument."; \
		echo "USAGE: make flash ESPTOOL_PORT=<port> [ ESPTOOL_BAUD=<baud> ] [ ESPTOOL_BINDIR=<dir> ]"; \
		exit 1; \
	fi
	$(eval ESPTOOL_OPTS := -c esp32s3 -p $(ESPTOOL_PORT) -b $(ESPTOOL_BAUD) $(if $(CONFIG_ESP32S3_ESPTOOLPY_NO_STUB),--no-stub))
	esptool.py $(ESPTOOL_OPTS) write_flash $(ESPTOOL_WRITEFLASH_OPTS) $(ESPTOOL_BINS)
endef
