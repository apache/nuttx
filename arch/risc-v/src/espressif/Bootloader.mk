############################################################################
# arch/risc-v/src/espressif/Bootloader.mk
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

TOOLSDIR             = $(TOPDIR)/tools/espressif
CHIPDIR              = $(TOPDIR)/arch/risc-v/src/chip
HALDIR               = $(CHIPDIR)/esp-hal-3rdparty
BOOTLOADER_SRCDIR    = $(CHIPDIR)/bootloader
BOOTLOADER_OUTDIR    = $(BOOTLOADER_SRCDIR)/out
BOOTLOADER_CONFIG    = $(BOOTLOADER_SRCDIR)/bootloader.conf

# MCUboot

MCUBOOT_SRCDIR     = $(BOOTLOADER_SRCDIR)/mcuboot
MCUBOOT_ESPDIR     = $(MCUBOOT_SRCDIR)/boot/espressif
MCUBOOT_URL        = https://github.com/mcu-tools/mcuboot

# IDFboot

BLBIN_VERSION = latest
BLBIN_URL     = https://github.com/espressif/esp-nuttx-bootloader/releases/download/$(BLBIN_VERSION)

# Helpers for creating the configuration file

cfg_en  = echo "$(1)=$(if $(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),1,y)";
cfg_dis = echo "$(1)=$(if $(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),0,n)";
cfg_val = echo "$(1)=$(2)";

$(BOOTLOADER_SRCDIR):
	$(Q) mkdir -p $(BOOTLOADER_SRCDIR) &>/dev/null

$(BOOTLOADER_CONFIG): $(TOPDIR)/.config $(BOOTLOADER_SRCDIR)
	$(Q) echo "Creating Bootloader configuration"
	$(Q) { \
		$(if $(CONFIG_ESPRESSIF_FLASH_2M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_2MB)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_4M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_4MB)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_MODE_DIO),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_DIO)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_MODE_DOUT),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_DOUT)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_MODE_QIO),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_QIO)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_MODE_QOUT),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHMODE_QOUT)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_FREQ_80M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_80M)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_FREQ_48M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_48M)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_FREQ_40M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_40M)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_FREQ_26M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_26M)) \
		$(if $(CONFIG_ESPRESSIF_FLASH_FREQ_20M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHFREQ_20M)) \
	} > $(BOOTLOADER_CONFIG)
ifeq ($(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),y)
	$(Q) { \
		$(call cfg_val,CONFIG_ESP_BOOTLOADER_OFFSET,0x0000) \
		$(call cfg_val,CONFIG_ESP_BOOTLOADER_SIZE,0xF000) \
		$(call cfg_val,CONFIG_ESP_IMAGE0_PRIMARY_START_ADDRESS,$(CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_APPLICATION_SIZE,$(CONFIG_ESPRESSIF_OTA_SLOT_SIZE)) \
		$(call cfg_val,CONFIG_ESP_IMAGE0_SECONDARY_START_ADDRESS,$(CONFIG_ESPRESSIF_OTA_SECONDARY_SLOT_OFFSET)) \
		$(call cfg_en,CONFIG_ESP_MCUBOOT_WDT_ENABLE) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_OFFSET,$(CONFIG_ESPRESSIF_OTA_SCRATCH_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_SIZE,$(CONFIG_ESPRESSIF_OTA_SCRATCH_SIZE)) \
		$(call cfg_en,CONFIG_ESP_CONSOLE_UART) \
		$(if $(CONFIG_UART0_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_UART_NUM,0)) \
		$(if $(CONFIG_UART1_SERIAL_CONSOLE),$(call cfg_val,CONFIG_ESP_CONSOLE_UART_NUM,1)) \
	} >> $(BOOTLOADER_CONFIG)
else
	$(Q) { \
		$(call cfg_en,CONFIG_PARTITION_TABLE_CUSTOM) \
		$(call cfg_val,CONFIG_PARTITION_TABLE_CUSTOM_FILENAME,\"partitions.csv\") \
		$(call cfg_val,CONFIG_PARTITION_TABLE_OFFSET,$(CONFIG_ESPRESSIF_PARTITION_TABLE_OFFSET)) \
	} >> $(BOOTLOADER_CONFIG)
endif

ifeq ($(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),y)

BOOTLOADER_BIN = $(TOPDIR)/mcuboot-$(CHIP_SERIES).bin

$(MCUBOOT_SRCDIR): $(BOOTLOADER_SRCDIR)
	$(Q) echo "Cloning MCUboot"
	$(Q) git clone --quiet $(MCUBOOT_URL) $(MCUBOOT_SRCDIR)
	$(Q) git -C "$(MCUBOOT_SRCDIR)" checkout --quiet $(CONFIG_ESPRESSIF_MCUBOOT_VERSION)
	$(Q) git -C "$(MCUBOOT_SRCDIR)" submodule --quiet update --init --recursive ext/mbedtls

$(BOOTLOADER_BIN): chip/$(ESP_HAL_3RDPARTY_REPO) $(MCUBOOT_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building MCUboot"
	$(Q) $(TOOLSDIR)/build_mcuboot.sh \
		-c $(CHIP_SERIES) \
		-f $(BOOTLOADER_CONFIG) \
		-p $(BOOTLOADER_SRCDIR) \
		-e $(HALDIR)
	$(call COPYFILE, $(BOOTLOADER_OUTDIR)/mcuboot-$(CHIP_SERIES).bin, $(TOPDIR))

bootloader: $(BOOTLOADER_CONFIG) $(BOOTLOADER_BIN)

clean_bootloader:
	$(call DELDIR,$(BOOTLOADER_SRCDIR))
	$(call DELFILE,$(BOOTLOADER_BIN))

else

bootloader:
	$(Q) echo "Downloading Bootloader binaries"
	$(call DOWNLOAD,$(BLBIN_URL),bootloader-$(CHIP_SERIES).bin,$(TOPDIR)/bootloader-$(CHIP_SERIES).bin)
	$(call DOWNLOAD,$(BLBIN_URL),partition-table-$(CHIP_SERIES).bin,$(TOPDIR)/partition-table-$(CHIP_SERIES).bin)

clean_bootloader:
	$(call DELFILE,$(TOPDIR)/bootloader-$(CHIP_SERIES).bin)
	$(call DELFILE,$(TOPDIR)/partition-table-$(CHIP_SERIES).bin)

endif
