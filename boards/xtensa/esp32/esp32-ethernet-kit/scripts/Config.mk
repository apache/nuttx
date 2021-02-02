############################################################################
# boards/xtensa/esp32/esp32-ethernet-kit/scripts/Config.mk
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

# POSTBUILD -- Perform post build operations

ifdef BLOBDIR
	BOOTLOADER=${BLOBDIR}/esp32core/bootloader.bin
	PARTITION_TABLE=${BLOBDIR}/esp32core/partition-table.bin
else
	BOOTLOADER=$(IDF_PATH)/hello_world/build/bootloader/bootloader.bin
	PARTITION_TABLE=$(IDF_PATH)/hello_world/build/partition_table/partition-table.bin
endif

ifeq ($(CONFIG_ESP32_QEMU_IMAGE),y)
	MK_QEMU_IMG=$(TOPDIR)/tools/esp32/mk_qemu_img.sh $(BOOTLOADER) $(PARTITION_TABLE) $(NUTTXNAME)
else
	MK_QEMU_IMG=
endif

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
		esptool.py --chip esp32 elf2image --flash_mode dio --flash_size 4MB -o $(NUTTXNAME).bin nuttx; \
		echo "Generated: $(NUTTXNAME).bin (ESP32 compatible)"; \
	fi
	$(MK_QEMU_IMG)
endef
