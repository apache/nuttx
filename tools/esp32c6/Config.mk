############################################################################
# tools/esp32c6/Config.mk
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

ifdef ESPTOOL_BINDIR
	BL_OFFSET=0x0
	PT_OFFSET=0x8000
	BOOTLOADER=$(ESPTOOL_BINDIR)/bootloader.bin
	PARTITION_TABLE=$(ESPTOOL_BINDIR)/partition-table.bin
	FLASH_BL=$(BL_OFFSET) $(BOOTLOADER)
	FLASH_PT=$(PT_OFFSET) $(PARTITION_TABLE)
endif

ifeq ($(CONFIG_ESP32C6_FLASH_2M),y)
	FLASH_SIZE="2MB"
else ifeq ($(CONFIG_ESP32C6_FLASH_4M),y)
	FLASH_SIZE="4MB"
else ifeq ($(CONFIG_ESP32C6_FLASH_8M),y)
	FLASH_SIZE="8MB"
else ifeq ($(CONFIG_ESP32C6_FLASH_16M),y)
	FLASH_SIZE="16MB"
endif

# POSTBUILD -- Perform post build operations

define POSTBUILD
	$(Q) echo "MKIMAGE: ESP32-C6 binary"
	$(Q) if ! esptool.py version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "esptool.py not found.  Please run: \"pip install esptool\""; \
		echo "Or run: \"make -C $(TOPDIR)/tools/esp32c6\" to install all IDF tools."; \
		echo ""; \
		echo "Run make again to create the nuttx.bin image."; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration for the ESP32-C6 chip."; \
		exit 1; \
	fi
	esptool.py --chip esp32c6 elf2image --flash_mode dio --flash_size $(FLASH_SIZE) -o nuttx.bin nuttx
	$(Q) echo "Generated: nuttx.bin (ESP32-C6 compatible)"
endef

# ESPTOOL_BAUD -- Serial port baud rate used when flashing/reading via esptool.py

ESPTOOL_BAUD ?= 921600

# DOWNLOAD -- Download binary image via esptool.py

define DOWNLOAD
	$(Q) if [ -z $(ESPTOOL_PORT) ]; then \
		echo "DOWNLOAD error: Missing serial port device argument."; \
		echo "USAGE: make download ESPTOOL_PORT=<port> [ ESPTOOL_BAUD=<baud> ]"; \
		exit 1; \
	fi
	esptool.py --chip esp32c6 --port $(ESPTOOL_PORT) --baud $(ESPTOOL_BAUD) write_flash $(FLASH_BL) $(FLASH_PT) 0x10000 $(1).bin
endef
