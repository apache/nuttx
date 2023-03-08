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

ifeq ($(CONFIG_ESP32S3_BOOTLOADER_BUILD_FROM_SOURCE),y)

CHIPDIR            = $(TOPDIR)/arch/xtensa/src/chip

BOOTLOADER_SRCDIR  = $(CHIPDIR)/esp-nuttx-bootloader
BOOTLOADER_VERSION = main
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader
BOOTLOADER_OUTDIR  = out
BOOTLOADER_CONFIG  = $(CHIPDIR)/bootloader.conf

$(BOOTLOADER_SRCDIR):
	$(Q) git clone $(BOOTLOADER_URL) $(BOOTLOADER_SRCDIR) -b $(BOOTLOADER_VERSION)

# Helpers for creating the configuration file

cfg_en  = echo "$(1)=$(if $(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),1,y)";
cfg_val = echo "$(1)=$(2)";

$(BOOTLOADER_CONFIG): $(TOPDIR)/.config
	$(Q) echo "Creating Bootloader configuration"
	$(Q) { \
		$(if $(CONFIG_ESP32S3_FLASH_4M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_4MB)) \
		$(if $(CONFIG_ESP32S3_FLASH_8M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_8MB)) \
		$(if $(CONFIG_ESP32S3_FLASH_16M),$(call cfg_en,CONFIG_ESPTOOLPY_FLASHSIZE_16MB)) \
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
		$(call cfg_val,CONFIG_ESP_APPLICATION_PRIMARY_START_ADDRESS,$(CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_APPLICATION_SIZE,$(CONFIG_ESP32S3_OTA_SLOT_SIZE)) \
		$(call cfg_val,CONFIG_ESP_APPLICATION_SECONDARY_START_ADDRESS,$(CONFIG_ESP32S3_OTA_SECONDARY_SLOT_OFFSET)) \
		$(call cfg_en,CONFIG_ESP_MCUBOOT_WDT_ENABLE) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_OFFSET,$(CONFIG_ESP32S3_OTA_SCRATCH_OFFSET)) \
		$(call cfg_val,CONFIG_ESP_SCRATCH_SIZE,$(CONFIG_ESP32S3_OTA_SCRATCH_SIZE)) \
	} >> $(BOOTLOADER_CONFIG)
else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)
	$(Q) { \
		$(call cfg_en,CONFIG_PARTITION_TABLE_CUSTOM) \
		$(call cfg_val,CONFIG_PARTITION_TABLE_CUSTOM_FILENAME,\"partitions.csv\") \
		$(call cfg_val,CONFIG_PARTITION_TABLE_OFFSET,$(CONFIG_ESP32S3_PARTITION_TABLE_OFFSET)) \
	} >> $(BOOTLOADER_CONFIG)
endif

ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)

BOOTLOADER_BIN        = $(TOPDIR)/mcuboot-esp32s3.bin

$(BOOTLOADER_BIN): $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader"
	$(Q) $(BOOTLOADER_SRCDIR)/build_mcuboot.sh -c esp32s3 -s -f $(BOOTLOADER_CONFIG)
	$(call COPYFILE, $(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/mcuboot-esp32s3.bin, $(TOPDIR))

bootloader: $(BOOTLOADER_CONFIG) $(BOOTLOADER_SRCDIR) $(BOOTLOADER_BIN)

clean_bootloader:
	$(call DELDIR,$(BOOTLOADER_SRCDIR))
	$(call DELFILE,$(BOOTLOADER_CONFIG))
	$(call DELFILE,$(BOOTLOADER_BIN))

else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)

bootloader: $(BOOTLOADER_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader binaries"
	$(Q) $(BOOTLOADER_SRCDIR)/build_idfboot.sh -c esp32s3 -s -f $(BOOTLOADER_CONFIG)
	$(call COPYFILE,$(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/bootloader-esp32s3.bin,$(TOPDIR))
	$(call COPYFILE,$(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/partition-table-esp32s3.bin,$(TOPDIR))

clean_bootloader:
	$(call DELDIR,$(BOOTLOADER_SRCDIR))
	$(call DELFILE,$(BOOTLOADER_CONFIG))
	$(call DELFILE,$(TOPDIR)/bootloader-esp32s3.bin)
	$(call DELFILE,$(TOPDIR)/partition-table-esp32s3.bin)

endif

else ifeq ($(CONFIG_ESP32S3_BOOTLOADER_DOWNLOAD_PREBUILT),y)

BOOTLOADER_VERSION = latest
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader/releases/download/$(BOOTLOADER_VERSION)

ifeq ($(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT),y)

bootloader:
	$(call DOWNLOAD,$(BOOTLOADER_URL),mcuboot-esp32s3.bin,$(TOPDIR)/mcuboot-esp32s3.bin)

clean_bootloader:
	$(call DELFILE,$(TOPDIR)/mcuboot-esp32s3.bin)

else ifeq ($(CONFIG_ESP32S3_APP_FORMAT_LEGACY),y)

bootloader:
	$(call DOWNLOAD,$(BOOTLOADER_URL),bootloader-esp32s3.bin,$(TOPDIR)/bootloader-esp32s3.bin)
	$(call DOWNLOAD,$(BOOTLOADER_URL),partition-table-esp32s3.bin,$(TOPDIR)/partition-table-esp32s3.bin)

clean_bootloader:
	$(call DELFILE,$(TOPDIR)/bootloader-esp32s3.bin)
	$(call DELFILE,$(TOPDIR)/partition-table-esp32s3.bin)

endif

endif
