############################################################################
# boards/xtensa/esp32/esp32-core/scripts/Config.mk
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
# and assembly source files and to insert the resulting object files into an
# archive.  These replace the default definitions at tools/Config.mk

# POSTBUILD -- Perform post build operations

ifeq ($(CONFIG_ESP32CORE_FLASH_IMAGE),y)
define POSTBUILD
	@echo "MKIMAGE: ESP32 binary"
        $(Q) if ! esptool.py version ; then \
		echo ""; \
		echo "Please install ESP-IDF tools"; \
		echo ""; \
		echo "Check https://docs.espressif.com/projects/esp-idf/en/v4.0/get-started/index.html#installation-step-by-step or run the following command"; \
		echo ""; \
		echo "cd tools/esp32 && make && cd ../.."; \
		echo ""; \
		echo "run make again to create the nuttx.bin image."; \
	else \
		echo "Generating: $(NUTTXNAME).bin (ESP32 compatible)"; \
		esptool.py --chip esp32 elf2image --flash_mode dio --flash_size 4MB -o nuttx.bin nuttx; \
		echo "Generated: $(NUTTXNAME).bin (ESP32 compatible)"; \
		echo "Generating: flash_image.bin"; \
		dd if=/dev/zero bs=1024 count=4096 of=flash_image.bin && \
		dd if=$(IDF_PATH)/hello_world/build/bootloader/bootloader.bin bs=1 seek=$(shell printf "%d" 0x1000) of=flash_image.bin conv=notrunc && \
		dd if=$(IDF_PATH)/hello_world/build/partition_table/partition-table.bin bs=1 seek=$(shell printf "%d" 0x8000) of=flash_image.bin conv=notrunc && \
		dd if=$(NUTTXNAME).bin bs=1 seek=$(shell printf "%d" 0x10000) of=flash_image.bin conv=notrunc && \
		echo "Generated: flash_image.bin (it can be run with 'qemu-system-xtensa -nographic -machine esp32 -drive file=flash_image.bin,if=mtd,format=raw')"; \
	fi
endef
endif
