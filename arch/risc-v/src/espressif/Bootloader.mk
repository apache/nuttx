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

BOOTLOADER_VERSION = latest
BOOTLOADER_URL     = https://github.com/espressif/esp-nuttx-bootloader/releases/download/$(BOOTLOADER_VERSION)

bootloader:
	$(Q) echo "Downloading Bootloader binaries"
	$(call DOWNLOAD,$(BOOTLOADER_URL),bootloader-$(CONFIG_ESPRESSIF_CHIP_SERIES).bin,$(TOPDIR)/bootloader-$(CONFIG_ESPRESSIF_CHIP_SERIES).bin)
	$(call DOWNLOAD,$(BOOTLOADER_URL),partition-table-$(CONFIG_ESPRESSIF_CHIP_SERIES).bin,$(TOPDIR)/partition-table-$(CONFIG_ESPRESSIF_CHIP_SERIES).bin)

clean_bootloader:
	$(call DELFILE,$(TOPDIR)/bootloader-$(CONFIG_ESPRESSIF_CHIP_SERIES).bin)
	$(call DELFILE,$(TOPDIR)/partition-table-$(CONFIG_ESPRESSIF_CHIP_SERIES).bin)
