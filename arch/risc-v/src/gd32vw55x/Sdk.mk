############################################################################
# arch/risc-v/src/gd32vw55x/Sdk.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# Locates the GigaDevice GD32VW55x SDK and exposes it under chip/sdk.
#
# By default the SDK is cloned at "context" time, pinned to a validated
# commit, with the port patches applied on top -- that is what lets CI (and
# anyone with only the NuttX tree) build with nothing preinstalled.  Same
# model as the esp-hal-3rdparty of the Espressif ports.
# CONFIG_GD32VW55X_WIFI_SDK_PATH overrides it with a local checkout.
#
# The Wi-Fi and BLE support need the SDK for the prebuilt radio libraries;
# PROGMEM needs it for the mask ROM API headers (the SiP flash is not
# programmed through the FMC registers -- see gd32vw55x_progmem.c).
#
# PINNED VERSION: SDK V1.0.3g -- see gigadevice_port/SDK_VERSION.md.  The
# prebuilt libraries are binary, and the port depends on their ABI: do not
# move this SHA without revalidating.
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

GDWIFI_SDK_REPO = GD32VW55x_WiFi_BLE_SDK

ifndef GDWIFI_SDK_URL
	GDWIFI_SDK_URL = https://github.com/GigaDeviceSemiconductor/GD32VW55x_WiFi_BLE_SDK.git
endif

ifndef GDWIFI_SDK_VERSION
	GDWIFI_SDK_VERSION = 945c6e28754f1bbdefb8bcd3049593fae8873bd5
endif

GDWIFI_PATCHES = $(abspath $(ARCH_SRCDIR)$(DELIM)gd32vw55x$(DELIM)gdwifi$(DELIM)patches$(DELIM)0001-nuttx-port.patch)

GDWIFI_SDK := $(patsubst "%",%,$(CONFIG_GD32VW55X_WIFI_SDK_PATH))

ifeq ($(GDWIFI_SDK),)

GDWIFI_SDK = $(ARCH_SRCDIR)$(DELIM)gd32vw55x$(DELIM)$(GDWIFI_SDK_REPO)

chip/$(GDWIFI_SDK_REPO):
	$(Q) echo "Cloning GigaDevice GD32VW55x Wi-Fi/BLE SDK"
	$(Q) $(call CLONE, $(GDWIFI_SDK_URL),chip/$(GDWIFI_SDK_REPO))
	$(Q) echo "GD32VW55x SDK: ${GDWIFI_SDK_VERSION}"
	$(Q) git -C chip/$(GDWIFI_SDK_REPO) checkout --quiet $(GDWIFI_SDK_VERSION)
	$(Q) echo "Applying NuttX port patches"
	$(Q) git -C chip/$(GDWIFI_SDK_REPO) apply $(GDWIFI_PATCHES)

context:: chip/$(GDWIFI_SDK_REPO)

distclean::
	$(call DELDIR, chip/$(GDWIFI_SDK_REPO))

endif

# Expose the SDK under chip/sdk so the source paths stay in-tree relative

GDWIFI_LINK := $(shell ln -sfn $(GDWIFI_SDK) $(ARCH_SRCDIR)$(DELIM)gd32vw55x$(DELIM)sdk)

SDKDIR = chip$(DELIM)sdk
MSDK   = $(SDKDIR)$(DELIM)MSDK

# The mask ROM API (rom_api.h): needed by PROGMEM, and by the radio

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(SDKDIR)$(DELIM)ROM-EXPORT$(DELIM)bootloader
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(SDKDIR)$(DELIM)ROM-EXPORT$(DELIM)halcomm
