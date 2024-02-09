############################################################################
# arch/xtensa/src/esp32s3/Bootloader.mk
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

.PHONY: bootloader clean_bootloader

ifeq ($(CONFIG_ESPRESSIF_SIMPLE_BOOT),)

TOOLSDIR           = $(TOPDIR)/tools/espressif
CHIPDIR            = $(TOPDIR)/arch/xtensa/src/chip
HALDIR             = $(CHIPDIR)/esp-hal-3rdparty

BOOTLOADER_DIR     = $(CHIPDIR)/bootloader
BOOTLOADER_SRCDIR  = $(BOOTLOADER_DIR)/esp-nuttx-bootloader
BOOTLOADER_VERSION = main
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader
BOOTLOADER_OUTDIR  = out
BOOTLOADER_CONFIG  = $(BOOTLOADER_DIR)/bootloader.conf

MCUBOOT_SRCDIR     = $(BOOTLOADER_DIR)/mcuboot
MCUBOOT_ESPDIR     = $(MCUBOOT_SRCDIR)/boot/espressif
MCUBOOT_URL        = https://github.com/mcu-tools/mcuboot

$(BOOTLOADER_DIR):
	$(Q) mkdir -p $(BOOTLOADER_DIR) &>/dev/null

# Helpers for creating the configuration file

cfg_en  = echo "$(1)=$(if $(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),1,y)";
cfg_val = echo "$(1)=$(2)";

$(BOOTLOADER_CONFIG): $(TOPDIR)/.config $(BOOTLOADER_DIR)
	$(Q) echo "Creating Bootloader configuration"
	$(Q) { \
		$(if $(CONFIG_ESP32S3_FLASH_4M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_4MB)) \
		$(if $(CONFIG_ESP32S3_FLASH_8M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_8MB)) \
		$(if $(CONFIG_ESP32S3_FLASH_16M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_16MB)) \
		$(if $(CONFIG_ESP32S3_FLASH_32M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_32MB)) \
		$(if $(CONFIG_ESP32S3_FLASH_MODE_DIO),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_DIO)) \
		$(if $(CONFIG_ESP32S3_FLASH_MODE_DOUT),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_DOUT)) \
		$(if $(CONFIG_ESP32S3_FLASH_MODE_QIO),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_QIO)) \
		$(if $(CONFIG_ESP32S3_FLASH_MODE_QOUT),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_QOUT)) \
		$(if $(CONFIG_ESP32S3_FLASH_FREQ_120M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_120M)) \
		$(if $(CONFIG_ESP32S3_FLASH_FREQ_80M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_80M)) \
		$(if $(CONFIG_ESP32S3_FLASH_FREQ_40M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_40M)) \
		$(if $(CONFIG_ESP32S3_FLASH_FREQ_20M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_20M)) \
	} > $(BOOTLOADER_CONFIG)
ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)
	$(Q) { \
		$(call cfg_val,CONFIG_ESP_BOOTLOADER_OFFSET,0x0000) \
		$(call cfg_val,CONFIG_ESP_BOOTLOADER_SIZE,0xF000) \
		$(call cfg_val,CONFIG_ESP_IMAGE0_PRIMARY_START_ADDRESS,$(CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_APPLICATION_SIZE,$(CONFIG_ESP32S3_OTA_SLOT_SIZE)) \
		$(call cfg_val,CONFIG_ESP_IMAGE0_SECONDARY_START_ADDRESS,$(CONFIG_ESP32S3_OTA_SECONDARY_SLOT_OFFSET)) \
		$(call cfg_en,CONFIG_ESP_MCUBOOT_WDT_ENABLE) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_OFFSET,$(CONFIG_ESP32S3_OTA_SCRATCH_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_SIZE,$(CONFIG_ESP32S3_OTA_SCRATCH_SIZE)) \
		$(call cfg_en,CONFIG_ESP_CONSOLE_UART) \
		$(if $(CONFIG_UART0_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_UART_NUM,0)) \
		$(if $(CONFIG_UART1_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_UART_NUM,1)) \
		$(call cfg_en,CONFIG_BOOTLOADER_FLASH_XMC_SUPPORT) \
	} >> $(BOOTLOADER_CONFIG)
else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)
	$(Q) { \
		$(call cfg_en,CONFIG_PARTITION_TABLE_CUSTOM) \
		$(call cfg_val,CONFIG_PARTITION_TABLE_CUSTOM_FILENAME,\"partitions.csv\") \
		$(call cfg_val,CONFIG_PARTITION_TABLE_OFFSET,$(CONFIG_ESP32S3_PARTITION_TABLE_OFFSET)) \
	} >> $(BOOTLOADER_CONFIG)
endif
endif

ifeq ($(CONFIG_ESPRESSIF_SIMPLE_BOOT),y)
bootloader:
	$(Q) echo "Using direct bootloader to boot NuttX."

else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)

BOOTLOADER_BIN        = $(TOPDIR)/mcuboot-esp32s3.bin

$(MCUBOOT_SRCDIR): $(BOOTLOADER_DIR)
	$(Q) echo "Cloning MCUboot"
	$(Q) git clone --quiet $(MCUBOOT_URL) $(MCUBOOT_SRCDIR)
	$(Q) git -C "$(MCUBOOT_SRCDIR)" checkout --quiet $(CONFIG_ESP32S2_MCUBOOT_VERSION)
	$(Q) git -C "$(MCUBOOT_SRCDIR)" submodule --quiet update --init --recursive ext/mbedtls

$(BOOTLOADER_BIN): chip/$(ESP_HAL_3RDPARTY_REPO) $(MCUBOOT_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader"
	$(Q) $(TOOLSDIR)/build_mcuboot.sh \
		-c esp32s3 \
		-f $(BOOTLOADER_CONFIG) \
		-p $(BOOTLOADER_DIR) \
		-e $(HALDIR)
	$(call COPYFILE, $(BOOTLOADER_DIR)/$(BOOTLOADER_OUTDIR)/mcuboot-esp32s3.bin, $(TOPDIR))

bootloader: $(BOOTLOADER_CONFIG) $(BOOTLOADER_BIN)

clean_bootloader:
	$(call DELDIR,$(BOOTLOADER_DIR))
	$(call DELFILE,$(BOOTLOADER_BIN))

else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)

$(BOOTLOADER_SRCDIR): $(BOOTLOADER_DIR)
	$(Q) git clone $(BOOTLOADER_URL) $(BOOTLOADER_SRCDIR) -b $(BOOTLOADER_VERSION)

bootloader: $(BOOTLOADER_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader binaries"
	$(Q) $(BOOTLOADER_SRCDIR)/build_idfboot.sh -c esp32s3 -s -f $(BOOTLOADER_CONFIG)
	$(call COPYFILE,$(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/bootloader-esp32s3.bin,$(TOPDIR))
	$(call COPYFILE,$(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/partition-table-esp32s3.bin,$(TOPDIR))

clean_bootloader:
	$(call DELDIR,$(BOOTLOADER_DIR))
	$(call DELFILE,$(TOPDIR)/bootloader-esp32s3.bin)
	$(call DELFILE,$(TOPDIR)/partition-table-esp32s3.bin)

endif
