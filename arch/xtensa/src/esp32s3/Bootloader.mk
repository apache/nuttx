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

BOOTLOADER_DIR     = $(CHIPDIR)/bootloader
BOOTLOADER_SRCDIR  = $(BOOTLOADER_DIR)/esp-nuttx-bootloader
BOOTLOADER_VERSION = main
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader
BOOTLOADER_OUTDIR  = out
BOOTLOADER_CONFIG  = $(BOOTLOADER_DIR)/bootloader.conf
HALDIR             = $(BOOTLOADER_DIR)/esp-hal-3rdparty-mcuboot

MCUBOOT_SRCDIR     = $(BOOTLOADER_DIR)/mcuboot
MCUBOOT_ESPDIR     = $(MCUBOOT_SRCDIR)/boot/espressif
MCUBOOT_TOOLCHAIN  = $(TOPDIR)/tools/esp32s3/mcuboot_toolchain_esp32s3.cmake

ifndef MCUBOOT_VERSION
	MCUBOOT_VERSION = $(CONFIG_ESP32S3_MCUBOOT_VERSION)
endif

ifndef MCUBOOT_URL
	MCUBOOT_URL = https://github.com/mcu-tools/mcuboot
endif

ifndef ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT
	ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT = 3f02f2139e79ddc60f98ca35ed65c62c6914f079
endif

$(BOOTLOADER_DIR):
	$(Q) mkdir -p $(BOOTLOADER_DIR) &>/dev/null

# Helpers for creating the configuration file

cfg_en  = echo "$(1)=$(if $(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),1,y)";
cfg_val = echo "$(1)=$(2)";

$(BOOTLOADER_CONFIG): $(TOPDIR)/.config $(BOOTLOADER_DIR)
	$(Q) echo "Creating Bootloader configuration"
	$(Q) { \
		$(call cfg_en,NON_OS_BUILD) \
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
		$(call cfg_val,CONFIG_ESPTOOLPY_FLASHFREQ,$(CONFIG_ESPRESSIF_FLASH_FREQ)) \
	} > $(BOOTLOADER_CONFIG)
ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)
	$(Q) { \
		$(call cfg_en,CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) \
		$(call cfg_val,CONFIG_ESP_BOOTLOADER_OFFSET,0x0000) \
		$(call cfg_val,CONFIG_ESP_BOOTLOADER_SIZE,0xF000) \
		$(call cfg_val,CONFIG_ESP_IMAGE0_PRIMARY_START_ADDRESS,$(CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_APPLICATION_SIZE,$(CONFIG_ESP32S3_OTA_SLOT_SIZE)) \
		$(call cfg_val,CONFIG_ESP_IMAGE0_SECONDARY_START_ADDRESS,$(CONFIG_ESP32S3_OTA_SECONDARY_SLOT_OFFSET)) \
		$(call cfg_en,CONFIG_ESP_MCUBOOT_WDT_ENABLE) \
		$(call cfg_en,CONFIG_LIBC_NEWLIB) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_OFFSET,$(CONFIG_ESP32S3_OTA_SCRATCH_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_SIZE,$(CONFIG_ESP32S3_OTA_SCRATCH_SIZE)) \
		$(call cfg_en,CONFIG_ESP_CONSOLE_UART) \
		$(if $(CONFIG_UART0_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_UART_NUM,0)) \
		$(if $(CONFIG_UART1_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_UART_NUM,1)) \
		$(if $(CONFIG_UART0_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM,0)) \
		$(if $(CONFIG_UART1_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM,1)) \
		$(call cfg_en,CONFIG_BOOTLOADER_FLASH_XMC_SUPPORT) \
		$(call cfg_en,CONFIG_IDF_TARGET_ARCH_XTENSA) \
		$(call cfg_val,CONFIG_BOOTLOADER_LOG_LEVEL,3) \
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

define CLONE_ESP_HAL_3RDPARTY_REPO_MCUBOOT
	$(call CLONE, $(ESP_HAL_3RDPARTY_URL),$(HALDIR))
endef

$(MCUBOOT_SRCDIR): $(BOOTLOADER_DIR)
	$(Q) echo "Cloning MCUboot"
	$(Q) git clone --quiet $(MCUBOOT_URL) $(MCUBOOT_SRCDIR)
	$(Q) git -C "$(MCUBOOT_SRCDIR)" checkout --quiet $(MCUBOOT_VERSION)
	$(Q) git -C "$(MCUBOOT_SRCDIR)" submodule --quiet update --init --recursive ext/mbedtls

$(HALDIR):
	$(Q) echo "Cloning Espressif HAL for 3rd Party Platforms (MCUBoot build)"
	$(Q) $(call CLONE_ESP_HAL_3RDPARTY_REPO_MCUBOOT)
	$(Q) echo "Espressif HAL for 3rd Party Platforms (MCUBoot build): ${ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT}"
	$(Q) git -C $(HALDIR) checkout --quiet $(ESP_HAL_3RDPARTY_VERSION_FOR_MCUBOOT)

$(BOOTLOADER_BIN): $(HALDIR) $(MCUBOOT_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader"
	$(Q) $(TOOLSDIR)/build_mcuboot.sh \
		-c esp32s3 \
		-f $(BOOTLOADER_CONFIG) \
		-p $(BOOTLOADER_DIR) \
		-e $(HALDIR) \
		-d $(MCUBOOT_TOOLCHAIN)
	$(call COPYFILE, $(BOOTLOADER_DIR)/$(BOOTLOADER_OUTDIR)/mcuboot-esp32s3.bin, $(TOPDIR))

bootloader: $(BOOTLOADER_CONFIG) $(BOOTLOADER_BIN)

clean_bootloader:
	$(call DELDIR,$(HALDIR))
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
	$(call DELDIR,$(HALDIR))
	$(call DELDIR,$(BOOTLOADER_DIR))
	$(call DELFILE,$(TOPDIR)/bootloader-esp32s3.bin)
	$(call DELFILE,$(TOPDIR)/partition-table-esp32s3.bin)

endif
