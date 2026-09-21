############################################################################
# arch/arm/src/rtl8730e/ameba_board.mk
#
# SPDX-License-Identifier: Apache-2.0
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

# Per-IC build definitions for RTL8730E (AmebaSmart CA32).
#
# Unlike the KM4-based ICs, there is no NP firmware build step here: the
# KM4/KM0 firmware is vendored as prebuilt binaries in prebuilt/.
# The PREBUILD step only builds the minimal fwlib sources needed by NuttX
# drivers (flash MTD primitives and the IPC layer they depend on).
#
# Flash MTD: FLASH_ReadStream/WriteStream/EraseXIP are in ameba_flash_ram.c.
# On CA32 the lock protocol (FLASH_Write_Lock/Unlock) gates core1 and sends
# an IPC to KM4 to pause its own flash accesses before programming.  We
# compile ameba_flash_ram.c directly from SDK source so it links cleanly.
# The stubs it requires under CONFIG_ARM_CORE_CA32 are supplied by
# arch/arm/src/rtl8730e/rtl8730e_flash_stubs.c (not from SDK).

# --- SDK paths -------------------------------------------------------------

AMEBA_SOC_NAME   = amebasmart
AMEBA_SOC        = $(AMEBA_SDK)/component/soc/$(AMEBA_SOC_NAME)
AMEBA_PREBUILT   = $(BOARD_DIR)$(DELIM)prebuilt
AMEBA_PREBUILT_LIBS = $(AMEBA_PREBUILT)$(DELIM)libs
AMEBA_AUTOCONF   = $(AMEBA_PREBUILT)$(DELIM)platform_autoconf.h

# --- fwlib sources (CA32) --------------------------------------------------
#
# Minimal set required by the flash MTD and IPC drivers.  Compiled from SDK
# source into libameba_fwlib.a by PREBUILD; --gc-sections keeps only what is
# referenced.

AMEBA_FWLIB_A    = $(AMEBA_PREBUILT_LIBS)$(DELIM)libameba_fwlib.a

AMEBA_FWLIB_SRCS = $(AMEBA_SOC)/fwlib/ram_common/ameba_arch.c \
                   $(AMEBA_SOC)/swlib/log.c \
                   $(AMEBA_SOC)/swlib/sscanf_minimal.c

# IPC layer is needed by both WiFi (WHC) and Flash FS (write-lock protocol).
# Include it once regardless of which feature(s) are enabled.
ifneq ($(filter y,$(CONFIG_RTL8730E_WIFI) $(CONFIG_RTL8730E_FLASH_FS)),)
AMEBA_FWLIB_SRCS += $(AMEBA_SOC)/fwlib/ram_common/ameba_ipc_api.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_ipc_ram.c
endif

ifeq ($(CONFIG_RTL8730E_FLASH_FS),y)
AMEBA_FWLIB_SRCS += $(AMEBA_SOC)/fwlib/ram_common/ameba_flash_ram.c

# lib_rom.a: prebuilt SDK AP library that provides FLASH_TxData/RxCmd/Erase/
# SetStatus and RSIP_MMU_* called by ameba_flash_ram.c at link time.
EXTRA_LIBS += $(AMEBA_SOC)/project/project_ap/lib/soc/lib_rom.a
endif

# Include paths scoped to the fwlib compile only (never leaked to NuttX core).
# -DCONFIG_ARM_CORE_CA32 activates the CA32 code paths in ameba_flash_ram.c.
# No -mcmse (that is Cortex-M33 TrustZone; CA32 is ARMv7-A non-secure EL1).

AMEBA_FWLIB_INC  = -Wno-int-conversion -Wno-shadow \
                   -DCONFIG_ARM_CORE_CA32 \
                   -I$(TOPDIR)/arch/arm/src/common/ameba/sdk_shim \
                   -I$(AMEBA_SOC)/fwlib/include \
                   -I$(AMEBA_SOC)/fwlib/include/rom \
                   -I$(AMEBA_SOC)/swlib \
                   -I$(AMEBA_SOC)/hal/include \
                   -I$(AMEBA_SOC)/hal/src \
                   -I$(AMEBA_SDK)/component/soc/common/include \
                   -I$(AMEBA_SDK)/component/soc/common/include/cmsis \
                   -I$(AMEBA_SOC)/app/monitor/include \
                   -I$(AMEBA_SDK)/component/soc/usrcfg/$(AMEBA_SOC_NAME)/include \
                   -I$(AMEBA_SDK)/component/soc/usrcfg/common \
                   -I$(AMEBA_SOC)/misc \
                   -I$(AMEBA_SDK)/component/os/os_wrapper/include \
                   -I$(AMEBA_PREBUILT) \
                   -I$(AMEBA_SDK)/component/ssl/mbedtls-3.6.2/include \
                   -I$(AMEBA_SDK)/component/os/freertos/heap_trace

# VFS1 partition geometry comes from platform_autoconf.h (static file that
# mirrors the SDK Kconfig defaults for amebasmart NOR layout).  Extract and
# pass as neutral -D flags so ameba_flash_mtd.c never force-includes the SDK
# header.
ifneq ($(wildcard $(AMEBA_AUTOCONF)),)
AMEBA_VFS1_OFFSET := $(strip $(shell awk '$$2=="CONFIG_FLASH_VFS1_OFFSET"{print $$3}' $(AMEBA_AUTOCONF)))
AMEBA_VFS1_SIZE   := $(strip $(shell awk '$$2=="CONFIG_FLASH_VFS1_SIZE"{print $$3}' $(AMEBA_AUTOCONF)))
CFLAGS += $(if $(AMEBA_VFS1_OFFSET),-DAMEBA_FLASH_VFS1_OFFSET_XIP=$(AMEBA_VFS1_OFFSET))
CFLAGS += $(if $(AMEBA_VFS1_SIZE),-DAMEBA_FLASH_VFS1_SIZE_CFG=$(AMEBA_VFS1_SIZE))
endif

# --- WiFi (WHC host) glue lib (CONFIG_RTL8730E_WIFI) -----------------------
#
# libameba_wifi.a: NuttX-side glue the prebuilt WHC host libs expect.
# Compiled with the SDK WiFi include set (which conflicts with NuttX headers)
# in a separate PREBUILD loop, identical to the 8721dx pattern.
# ameba_lwip_off.h (force-included first) hides the SDK lwIP layer so NuttX's
# own lwIP is used.

AMEBA_WIFI_A    = $(AMEBA_PREBUILT_LIBS)$(DELIM)libameba_wifi.a
AMEBA_WIFI_DIR  = $(TOPDIR)$(DELIM)arch$(DELIM)arm$(DELIM)src$(DELIM)common$(DELIM)ameba$(DELIM)wifi

ifeq ($(CONFIG_RTL8730E_WIFI),y)
AMEBA_WIFI_SRCS = $(AMEBA_SDK)/component/soc/usrcfg/$(AMEBA_SOC_NAME)/ameba_wificfg.c \
                  $(AMEBA_SDK)/component/wifi/common/rtw_task_size.c \
                  $(AMEBA_SDK)/component/wifi/common/rtw_event.c \
                  $(AMEBA_SDK)/component/soc/common/diagnose/ameba_diagnose_none.c \
                  $(AMEBA_SDK)/component/wifi/wpa_supplicant/wpa_supplicant/wifi_p2p_disable.c \
                  $(AMEBA_WIFI_DIR)/ameba_wifi_depend.c \
                  $(AMEBA_WIFI_DIR)/ameba_wifi.c

AMEBA_WIFI_INC  = -include $(AMEBA_AUTOCONF) \
                  -include $(AMEBA_WIFI_DIR)/include/ameba_lwip_off.h \
                  -I$(AMEBA_WIFI_DIR)/include \
                  -I$(AMEBA_WIFI_DIR)/.. \
                  -I$(AMEBA_SDK)/component/wifi/api \
                  -I$(AMEBA_SDK)/component/wifi/common \
                  -I$(AMEBA_SDK)/component/wifi/driver/include \
                  -I$(AMEBA_SDK)/component/wifi/driver/intf \
                  -I$(AMEBA_SDK)/component/wifi/whc \
                  -I$(AMEBA_SDK)/component/wifi/whc/whc_host_rtos \
                  -I$(AMEBA_SDK)/component/wifi/whc/whc_host_rtos/ipc \
                  -I$(AMEBA_SDK)/component/at_cmd \
                  -I$(AMEBA_SDK)/component/wifi/wpa_supplicant/wpa_supplicant \
                  -I$(AMEBA_SDK)/component/wifi/wpa_supplicant/wpa_lite \
                  -I$(AMEBA_SDK)/component/wifi/wpa_supplicant/wpa_lite/rom \
                  -I$(AMEBA_SDK)/component/wifi/wpa_supplicant/src \
                  -I$(AMEBA_SDK)/component/wifi/wpa_supplicant/src/utils \
                  -I$(AMEBA_SDK)/component/wifi/rtk_app/wifi_auto_reconnect \
                  -I$(AMEBA_SDK)/component/network \
                  -I$(AMEBA_SDK)/component/soc/common/diagnose \
                  -I$(AMEBA_SOC)/app/monitor/include \
                  -I$(AMEBA_SOC)/fwlib/include \
                  -I$(AMEBA_SOC)/fwlib/include/rom \
                  -I$(AMEBA_SOC)/swlib \
                  -I$(AMEBA_SOC)/hal/include \
                  -I$(AMEBA_SOC)/misc \
                  -I$(AMEBA_SDK)/component/soc/common/include \
                  -I$(AMEBA_SDK)/component/soc/common/include/cmsis \
                  -I$(AMEBA_SDK)/component/os/os_wrapper/include \
                  -I$(AMEBA_SDK)/component/soc/usrcfg/$(AMEBA_SOC_NAME)/include \
                  -I$(AMEBA_SDK)/component/soc/usrcfg/common \
                  -I$(AMEBA_SDK)/component/ssl/mbedtls-3.6.2/include \
                  -I$(AMEBA_SDK)/component/os/freertos/heap_trace \
                  -I$(AMEBA_PREBUILT)
endif

# --- PREBUILD: compile fwlib sources into libameba_fwlib.a ----------------

define PREBUILD
	$(Q) mkdir -p $(AMEBA_PREBUILT_LIBS)
	$(Q) echo "GEN: libameba_fwlib.a (amebasmart CA32 fwlib from SDK source)"; \
	     rm -f $(AMEBA_FWLIB_A); \
	     objs=""; \
	     for src in $(AMEBA_FWLIB_SRCS); do \
	       obj="$(AMEBA_PREBUILT_LIBS)/$$(basename $${src%.c}).o"; \
	       echo "  CC  $$src"; \
	       $(CC) $(ARCHCPUFLAGS) -Os -ffunction-sections -fdata-sections \
	         $(AMEBA_FWLIB_INC) \
	         -c $$src -o $$obj || exit 1; \
	       objs="$$objs $$obj"; \
	     done; \
	     $(CROSSDEV)ar crs $(AMEBA_FWLIB_A) $$objs
	$(Q) if [ "$(CONFIG_RTL8730E_WIFI)" = "y" ]; then \
	       echo "GEN: libameba_wifi.a (WHC host glue from SDK source + NuttX shim)"; \
	       mkdir -p $(AMEBA_PREBUILT_LIBS)/wifi_obj; \
	       rebuild_wifi=0; wifi_objs=""; \
	       for src in $(AMEBA_WIFI_SRCS); do \
	         obj=$(AMEBA_PREBUILT_LIBS)/wifi_obj/`basename $$src .c`.o; \
	         if [ "$$src" -nt "$$obj" ] 2>/dev/null || [ ! -f "$$obj" ]; then \
	           $(CC) $(ARCHCPUFLAGS) -Os -ffunction-sections -fdata-sections \
	             $(AMEBA_WIFI_INC) -c $$src -o $$obj || exit 1; \
	           rebuild_wifi=1; \
	         fi; \
	         wifi_objs="$$wifi_objs $$obj"; \
	       done; \
	       if [ "$$rebuild_wifi" = "1" ] || [ ! -f "$(AMEBA_WIFI_A)" ]; then \
	         rm -f $(AMEBA_WIFI_A); \
	         $(CROSSDEV)ar crs $(AMEBA_WIFI_A) $$wifi_objs; \
	       fi; \
	     fi
endef

############################################################################
# POSTBUILD: assemble flashable app.bin (vendor_prefix + fip_prepend + fip)
#
# NuttX is packaged as BL33 inside a FIP.  The flash script handles this;
# POSTBUILD just copies nuttx.bin (the raw binary) to the top directory.
# The actual FIP assembly and flash packing is done by ameba_smart_flash.sh.

define POSTBUILD
endef
