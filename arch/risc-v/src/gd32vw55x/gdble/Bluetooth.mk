############################################################################
# arch/risc-v/src/gd32vw55x/gdble/Bluetooth.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# GD32VW55x BLE 5.3 through the vendor's own BLE host.
#
# The prebuilt libble_max.a is an all-in-one controller + RivieraWaves host
# (GAP/GATT/SMP inside the blob).  The controller talks to that host
# internally, not over HCI -- the blob is built without any HCI transport
# layer (h4tl.o is empty).  So we drive the vendor host directly (ble_adp_*,
# ble_adv_*, ble_gap_*), the way every GigaDevice BLE example does; there is
# no virtual HCI and no NuttX Bluetooth host here.
#
# Everything else libble needs (sys_*, rf_*, nvds, util/, the
# gd32vw55x_platform.c) already comes from the Wi-Fi build -- that is why
# GD32VW55X_BLE depends on GD32VW55X_WIFI.
#
# Validated HAL: SDK V1.0.3g (2026-04-23, commit 945c6e2).
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

# CFG_BLE_SUPPORT turns BLE on in platform_def.h (and, together with
# CFG_WLAN_SUPPORT, CFG_COEX).

CFLAGS += -DCFG_BLE_SUPPORT

# This .mk is included before Wireless.mk (so libble enters the linker
# group), so it defines the SDK paths on its own.

SDKDIR = chip$(DELIM)sdk
MSDK   = $(SDKDIR)$(DELIM)MSDK

# The BLE host API headers live in blesw/src/export.  config_max matches
# libble_max (role ALL, 4 connections); it must come BEFORE
# blesw/src/export, where a duplicate ble_config.h (the MIN one) would
# shadow it.

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)blesw$(DELIM)src$(DELIM)export$(DELIM)config_max
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)blesw$(DELIM)src$(DELIM)export

CHIP_CSRCS += chip$(DELIM)gdble$(DELIM)gd32_ble.c

# libble depends on rf_* (librf), so it has to stay INSIDE the
# --start-group/--end-group assembled by Wireless.mk.  Here we only announce
# the library; it is Wireless.mk that places it in the group.

GDBLE_LIB = ble_max
