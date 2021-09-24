############################################################################
# arch/xtensa/src/esp32s2/Bootloader.mk
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

ifeq ($(CONFIG_ESP32S2_BOOTLOADER_BUILD_FROM_SOURCE),y)

CHIPDIR            = $(TOPDIR)/arch/xtensa/src/chip

BOOTLOADER_SRCDIR  = $(CHIPDIR)/esp-nuttx-bootloader
BOOTLOADER_VERSION = main
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader
BOOTLOADER_OUTDIR  = out

$(BOOTLOADER_SRCDIR):
	$(Q) git clone $(BOOTLOADER_URL) $(BOOTLOADER_SRCDIR) -b $(BOOTLOADER_VERSION)

ifeq ($(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT),y)

BOOTLOADER_CONFIG = $(CHIPDIR)/mcuboot.conf

$(BOOTLOADER_CONFIG): $(TOPDIR)/.config
	$(Q) echo "Creating Bootloader configuration"
	$(Q) {                                                                                                \
		echo "CONFIG_ESP_BOOTLOADER_SIZE=0xF000";                                                         \
		echo "CONFIG_ESP_APPLICATION_PRIMARY_START_ADDRESS=$(CONFIG_ESP32S2_OTA_PRIMARY_SLOT_OFFSET)";    \
		echo "CONFIG_ESP_APPLICATION_SIZE=$(CONFIG_ESP32S2_OTA_SLOT_SIZE)";                               \
		echo "CONFIG_ESP_APPLICATION_SECONDARY_START_ADDRESS=$(CONFIG_ESP32S2_OTA_SECONDARY_SLOT_OFFSET)";\
		echo "CONFIG_ESP_MCUBOOT_WDT_ENABLE=y";                                                           \
		echo "CONFIG_ESP_SCRATCH_OFFSET=$(CONFIG_ESP32S2_OTA_SCRATCH_OFFSET)";                            \
		echo "CONFIG_ESP_SCRATCH_SIZE=$(CONFIG_ESP32S2_OTA_SCRATCH_SIZE)";                                \
	} > $(BOOTLOADER_CONFIG)

bootloader: $(BOOTLOADER_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader binaries"
	$(Q) $(BOOTLOADER_SRCDIR)/build_mcuboot.sh -c esp32s2 -s -f $(BOOTLOADER_CONFIG)
	$(call COPYFILE, $(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/mcuboot-esp32s2.bin, $(TOPDIR))

clean_bootloader:
	$(call DELDIR, $(BOOTLOADER_SRCDIR))
	$(call DELFILE, $(BOOTLOADER_CONFIG))
	$(call DELFILE, $(TOPDIR)/mcuboot-esp32s2.bin)

else ifeq ($(CONFIG_ESP32S2_APP_FORMAT_LEGACY),y)

BOOTLOADER_CONFIG = $(CHIPDIR)/sdkconfig

$(BOOTLOADER_CONFIG): $(TOPDIR)/.config
	$(Q) echo "Creating Bootloader configuration"
	$(Q) {                                                                                         \
		[ "$(CONFIG_ESP32S2_FLASH_2M)"        = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_4M)"        = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_8M)"        = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_16M)"       = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y"; \
		[ "$(CONFIG_ESP32S2_FLASH_MODE_DIO)"  = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHMODE_DIO=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_MODE_DOUT)" = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHMODE_DOUT=y"; \
		[ "$(CONFIG_ESP32S2_FLASH_MODE_QIO)"  = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHMODE_QIO=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_MODE_QOUT)" = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHMODE_QOUT=y"; \
		[ "$(CONFIG_ESP32S2_FLASH_FREQ_80M)"  = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHFREQ_80M=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_FREQ_40M)"  = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHFREQ_40M=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_FREQ_26M)"  = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHFREQ_26M=y";  \
		[ "$(CONFIG_ESP32S2_FLASH_FREQ_20M)"  = "y" ] && echo "CONFIG_ESPTOOLPY_FLASHFREQ_20M=y";  \
		echo "CONFIG_PARTITION_TABLE_CUSTOM=y";                                                    \
		echo "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"partitions.csv\"";                          \
	} > $(BOOTLOADER_CONFIG)

bootloader: $(BOOTLOADER_SRCDIR) $(BOOTLOADER_CONFIG)
	$(Q) echo "Building Bootloader binaries"
	$(Q) $(BOOTLOADER_SRCDIR)/build_idfboot.sh -c esp32s2 -s -f $(BOOTLOADER_CONFIG)
	$(call COPYFILE, $(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/bootloader-esp32s2.bin, $(TOPDIR))
	$(call COPYFILE, $(BOOTLOADER_SRCDIR)/$(BOOTLOADER_OUTDIR)/partition-table-esp32s2.bin, $(TOPDIR))

clean_bootloader:
	$(call DELDIR, $(BOOTLOADER_SRCDIR))
	$(call DELFILE, $(BOOTLOADER_CONFIG))
	$(call DELFILE, $(TOPDIR)/bootloader-esp32s2.bin)
	$(call DELFILE, $(TOPDIR)/partition-table-esp32s2.bin)

endif

else ifeq ($(CONFIG_ESP32S2_BOOTLOADER_DOWNLOAD_PREBUILT),y)

BOOTLOADER_VERSION = latest
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader/releases/download/$(BOOTLOADER_VERSION)

ifeq ($(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT),y)

bootloader:
	$(Q) echo "Downloading Bootloader binaries"
	$(Q) curl -L $(BOOTLOADER_URL)/mcuboot-esp32s2.bin -o $(TOPDIR)/mcuboot-esp32s2.bin

clean_bootloader:
	$(call DELFILE, $(TOPDIR)/mcuboot-esp32s2.bin)

else ifeq ($(CONFIG_ESP32S2_APP_FORMAT_LEGACY),y)

bootloader:
	$(Q) echo "Downloading Bootloader binaries"
	$(Q) curl -L $(BOOTLOADER_URL)/bootloader-esp32s2.bin -o $(TOPDIR)/bootloader-esp32s2.bin
	$(Q) curl -L $(BOOTLOADER_URL)/partition-table-esp32s2.bin -o $(TOPDIR)/partition-table-esp32s2.bin

clean_bootloader:
	$(call DELFILE, $(TOPDIR)/bootloader-esp32s2.bin)
	$(call DELFILE, $(TOPDIR)/partition-table-esp32s2.bin)

endif

endif
