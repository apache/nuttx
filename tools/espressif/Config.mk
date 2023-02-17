############################################################################
# tools/espressif/Config.mk
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

# Remove quotes from CONFIG_ESPRESSIF_CHIP_SERIES configuration

CHIP_SERIES = $(patsubst "%",%,$(CONFIG_ESPRESSIF_CHIP_SERIES))

# These are the macros that will be used in the NuttX make system to compile
# and assemble source files and to insert the resulting object files into an
# archive.  These replace the default definitions at tools/Config.mk

ifeq ($(CONFIG_ESPRESSIF_FLASH_2M),y)
	FLASH_SIZE := 2MB
else ifeq ($(CONFIG_ESPRESSIF_FLASH_4M),y)
	FLASH_SIZE := 4MB
else ifeq ($(CONFIG_ESPRESSIF_FLASH_8M),y)
	FLASH_SIZE := 8MB
else ifeq ($(CONFIG_ESPRESSIF_FLASH_16M),y)
	FLASH_SIZE := 16MB
endif

ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_DIO),y)
	FLASH_MODE := dio
else ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_DOUT),y)
	FLASH_MODE := dout
else ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_QIO),y)
	FLASH_MODE := qio
else ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_QOUT),y)
	FLASH_MODE := qout
endif

ifeq ($(CONFIG_ESPRESSIF_FLASH_FREQ_80M),y)
	FLASH_FREQ := 80m
else ifeq ($(CONFIG_ESPRESSIF_FLASH_FREQ_48M),y)
	FLASH_FREQ := 48m
else ifeq ($(CONFIG_ESPRESSIF_FLASH_FREQ_40M),y)
	FLASH_FREQ := 40m
else ifeq ($(CONFIG_ESPRESSIF_FLASH_FREQ_26M),y)
	FLASH_FREQ := 26m
else ifeq ($(CONFIG_ESPRESSIF_FLASH_FREQ_20M),y)
	FLASH_FREQ := 20m
endif

# Configure the variables according to build environment

ifdef ESPTOOL_BINDIR
	BL_OFFSET       := 0x0
	PT_OFFSET       := $(CONFIG_ESPRESSIF_PARTITION_TABLE_OFFSET)
	BOOTLOADER      := $(ESPTOOL_BINDIR)/bootloader-$(CHIP_SERIES).bin
	PARTITION_TABLE := $(ESPTOOL_BINDIR)/partition-table-$(CHIP_SERIES).bin
	FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)
	FLASH_PT        := $(PT_OFFSET) $(PARTITION_TABLE)
	ESPTOOL_BINS    := $(FLASH_BL) $(FLASH_PT)
endif

APP_OFFSET := 0x10000
APP_IMAGE  := nuttx.bin
FLASH_APP  := $(APP_OFFSET) $(APP_IMAGE)

ESPTOOL_BINS += $(FLASH_APP)

ifeq ($(CONFIG_BUILD_PROTECTED),y)
	ESPTOOL_BINS += $(CONFIG_ESPRESSIF_USER_IMAGE_OFFSET) nuttx_user.bin
endif

# MERGEBIN -- Merge raw binary files into a single file

define MERGEBIN
	$(Q) if [ -z $(ESPTOOL_BINDIR) ]; then \
		echo "MERGEBIN error: Missing argument for binary files directory."; \
		echo "USAGE: make ESPTOOL_BINDIR=<dir>"; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration."; \
		exit 1; \
	fi
	esptool.py -c $(CHIP_SERIES) merge_bin --output nuttx.merged.bin $(ESPTOOL_BINS)
	$(Q) echo nuttx.merged.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.merged.bin"
endef

# MKIMAGE -- Convert an ELF file into a compatible binary file

define MKIMAGE
	$(Q) echo "MKIMAGE: NuttX binary"
	$(Q) if ! esptool.py version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "esptool.py not found.  Please run: \"pip install esptool\""; \
		echo ""; \
		echo "Run make again to create the nuttx.bin image."; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration."; \
		exit 1; \
	fi
	$(eval ELF2IMAGE_OPTS := -fs $(FLASH_SIZE) -fm $(FLASH_MODE) -ff $(FLASH_FREQ))
	esptool.py -c $(CHIP_SERIES) elf2image $(ELF2IMAGE_OPTS) -o nuttx.bin nuttx
	$(Q) echo nuttx.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.bin"
endef

# POSTBUILD -- Perform post build operations

define POSTBUILD
	$(call MKIMAGE)
	$(if $(CONFIG_ESPRESSIF_MERGE_BINS),$(call MERGEBIN))
endef

# ESPTOOL_BAUD -- Serial port baud rate used when flashing/reading via esptool.py

ESPTOOL_BAUD ?= 921600

# FLASH -- Download a binary image via esptool.py

define FLASH
	$(Q) if [ -z $(ESPTOOL_PORT) ]; then \
		echo "FLASH error: Missing serial port device argument."; \
		echo "USAGE: make flash ESPTOOL_PORT=<port> [ ESPTOOL_BAUD=<baud> ]"; \
		exit 1; \
	fi

	$(eval ESPTOOL_OPTS := -c $(CHIP_SERIES) -p $(ESPTOOL_PORT) -b $(ESPTOOL_BAUD) $(if $(CONFIG_ESPRESSIF_ESPTOOLPY_NO_STUB),--no-stub))
	$(eval WRITEFLASH_OPTS := $(if $(CONFIG_ESPRESSIF_MERGE_BINS),0x0 nuttx.merged.bin,$(if $(CONFIG_ESPRESSIF_FLASH_DETECT),-fs detect) -fm dio $(ESPTOOL_BINS)))
	esptool.py $(ESPTOOL_OPTS) write_flash $(WRITEFLASH_OPTS)
endef
