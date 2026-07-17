############################################################################
# arch/risc-v/src/gd32vw55x/gdwifi/Wireless.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# Compiles the open-source layers of the GD32VW55x SDK Wi-Fi stack
# (wifi_manager, lwIP 2.2.0 + port, util, platform bsp, SPL subset) into
# libarch and links the prebuilt BSD-3 MAC/RF/supplicant libraries.
# The OS binding is gdwifi/wrapper_nuttx.c (sys_* facade).
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

# Defines: RTOS build + supplicant, never PLATFORM_OS_FREERTOS

# Flags identical to those of the vendor demo (compile_commands.json):
#   CFG_RTOS + EXEC_USING_STD_PRINTF, and NO CONFIG_WPA_SUPPLICANT.
# The full supplicant (libwpa_supplicant.a) is a path that the demo does
# not use: WPA comes from the lightweight libwpas.a, via wifi_wpa.c.
CFLAGS += -DCFG_RTOS -DEXEC_USING_STD_PRINTF -DGDWIFI_NUTTX

# The vendor SDK sources are not NuttX-warning-clean (undefined macros in
# #if, %d for uint32_t, shadowed locals, K&R prototypes, ...).  These
# suppressions are scoped to this chip's build; the macro redefinitions
# that no -Wno flag covers are fixed in the SDK patch instead
# (riscv_encoding.h, wifi_management.h).
CFLAGS += -Wno-undef -Wno-format -Wno-shadow -Wno-address
CFLAGS += -Wno-unused-function -Wno-strict-prototypes -Wno-array-parameter

# Include paths: our patched config dir must come first

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)gd32vw55x$(DELIM)gdwifi$(DELIM)config
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)gd32vw55x$(DELIM)gdwifi
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(SDKDIR)$(DELIM)config
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)gd32vw55x
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)NMSIS$(DELIM)Core$(DELIM)Include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)arch
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)arch$(DELIM)boot
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)arch$(DELIM)lib
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)arch$(DELIM)ll
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)arch$(DELIM)compiler
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)GD32VW55x_standard_peripheral
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)GD32VW55x_standard_peripheral$(DELIM)Include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)dma
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)nvds
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)raw_flash
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)reg
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)rf
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)time
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)trng
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)uart
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)wdt
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)macsw$(DELIM)export
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)macsw$(DELIM)import
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)util$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)wifi_manager
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)wifi_manager$(DELIM)wpas
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)wpa_supplicant$(DELIM)src
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)wpa_supplicant$(DELIM)src$(DELIM)utils
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)wpa_supplicant$(DELIM)src$(DELIM)crypto
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)rtos$(DELIM)rtos_wrapper

# Our NuttX-side glue

CHIP_CSRCS += chip$(DELIM)gdwifi$(DELIM)wrapper_nuttx.c
CHIP_CSRCS += chip$(DELIM)gdwifi$(DELIM)gdwifi_glue.c
CHIP_CSRCS += chip$(DELIM)gdwifi$(DELIM)gdwifi_newlib_compat.c

# The IP stack is the NuttX one (the SDK lwIP does not enter the build):
# the binary seam of the MAC firmware (lwip_import.h) is reimplemented on
# top of netdev, and the headers in gdwifi/net_compat shadow the lwIP ones
# in the SDK sources.
CFLAGS += -DGDWIFI_NUTTX_NET
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)gd32vw55x$(DELIM)gdwifi$(DELIM)net_compat
CHIP_CSRCS += chip$(DELIM)gdwifi$(DELIM)gdwifi_netdev.c
CHIP_CSRCS += chip$(DELIM)gdwifi$(DELIM)gdwifi_netif_compat.c

# wifi_manager (STA)

GDWIFI_MGR = $(MSDK)$(DELIM)wifi_manager$(DELIM)
CHIP_CSRCS += $(GDWIFI_MGR)wifi_eloop.c $(GDWIFI_MGR)wifi_init.c
CHIP_CSRCS += $(GDWIFI_MGR)wifi_management.c $(GDWIFI_MGR)wifi_net_ip.c
CHIP_CSRCS += $(GDWIFI_MGR)wifi_netlink.c $(GDWIFI_MGR)wifi_vif.c
CHIP_CSRCS += $(GDWIFI_MGR)wifi_wpa.c

# lwIP 2.2.0 core + IPv4 + api + port


# util

GDWIFI_UTIL = $(MSDK)$(DELIM)util$(DELIM)src$(DELIM)
CHIP_CSRCS += $(GDWIFI_UTIL)debug_print.c $(GDWIFI_UTIL)crc.c
CHIP_CSRCS += $(GDWIFI_UTIL)dlist.c $(GDWIFI_UTIL)slist.c
CHIP_CSRCS += $(GDWIFI_UTIL)trace_ext.c $(GDWIFI_UTIL)util.c
CHIP_CSRCS += $(GDWIFI_UTIL)cyclic_buffer.c $(GDWIFI_UTIL)user_setting.c
CHIP_CSRCS += $(GDWIFI_UTIL)aes_ecb.c

# plf/src platform pieces

GDWIFI_PLF = $(MSDK)$(DELIM)plf$(DELIM)src$(DELIM)
CHIP_CSRCS += $(GDWIFI_PLF)gd32vw55x_platform.c $(GDWIFI_PLF)init_rom.c
CHIP_CSRCS += $(GDWIFI_PLF)plf_assert.c $(GDWIFI_PLF)wakelock.c
CHIP_CSRCS += $(GDWIFI_PLF)dsp.c

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)NMSIS$(DELIM)DSP$(DELIM)Include

CHIP_CSRCS += $(GDWIFI_PLF)time$(DELIM)systime.c
CHIP_CSRCS += $(GDWIFI_PLF)trng$(DELIM)trng.c
CHIP_CSRCS += $(GDWIFI_PLF)nvds$(DELIM)nvds_flash.c
CHIP_CSRCS += $(GDWIFI_PLF)raw_flash$(DELIM)raw_flash_api.c
CHIP_CSRCS += $(GDWIFI_PLF)dma$(DELIM)dma.c

# mbedTLS 3.6 compiled from source, exactly like the vendor demo
# (link.txt: libmbedtls.a).  Some crypto symbols still resolve in the mask
# ROM -> the ROM symbol table stays in the link.

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)library
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)tests$(DELIM)include$(DELIM)spe

GDWIFI_MBEDTLS_SRCS = $(notdir $(wildcard $(GDWIFI_SDK)/MSDK/mbedtls/mbedtls/library/*.c))

# net_sockets.c is the mbedTLS TLS-over-BSD-sockets layer; we do not use TLS
# and it collides with the NuttX socket headers.
GDWIFI_MBEDTLS_SRCS := $(filter-out net_sockets.c,$(GDWIFI_MBEDTLS_SRCS))
CHIP_CSRCS += $(foreach f,$(GDWIFI_MBEDTLS_SRCS),$(MSDK)$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)library$(DELIM)$(f))

ARCHSCRIPT += $(GDWIFI_SDK)$(DELIM)ROM-EXPORT$(DELIM)symbol$(DELIM)rom_symbol_m.gcc

# SDK interrupt handler bodies (Wi-Fi ISRs calling into the MAC blob)

CHIP_CSRCS += $(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)gd32vw55x$(DELIM)gd32vw55x_it.c

# SPL subset

GDWIFI_SPL = $(MSDK)$(DELIM)plf$(DELIM)GD32VW55x_standard_peripheral$(DELIM)Source$(DELIM)
# Note: the SPL eclic driver is NOT compiled; eclic_irq_enable/disable are
# provided by gdwifi_glue.c on top of the NuttX IRQ layer so the SDK cannot
# reprogram ECLIC levels behind the kernel's back.

SPL_SRCS = rcu gpio fmc efuse trng crc dma pmu syscfg exti timer eclic \
           rtc cau cau_aes hau hau_sha_md5 pkcau
CHIP_CSRCS += $(foreach f,$(SPL_SRCS),$(GDWIFI_SPL)gd32vw55x_$(f).c)

# Prebuilt libraries (BSD-3): MAC firmware variant per Kconfig

ifeq ($(CONFIG_GD32VW55X_WIFI_LIB_MINSRAM),y)
GDWIFI_MACLIB = wifi_minsram
else ifeq ($(CONFIG_GD32VW55X_WIFI_LIB_MESH_SMART),y)
GDWIFI_MACLIB = wifi_mesh_smart
else ifeq ($(CONFIG_GD32VW55X_WIFI_LIB_MULTISTREAM),y)
GDWIFI_MACLIB = wifi_multistream
else ifeq ($(CONFIG_GD32VW55X_WIFI_LIB_SOFTAP_MANY),y)
GDWIFI_MACLIB = wifi_softap_many_clients
else
GDWIFI_MACLIB = wifi
endif

# The demo links -lwpas (lightweight WPA).  libwpa_supplicant.a is the full
# supplicant and must only be used with -DCONFIG_WPA_SUPPLICANT (a path not
# exercised by the vendor).

ifeq ($(CONFIG_GD32VW55X_WIFI_WPS_EAP_TLS),y)
GDWIFI_WPASLIB = wpas_wps_eap-tls
else
GDWIFI_WPASLIB = wpas
endif

EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)lib
EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)$(MSDK)$(DELIM)plf$(DELIM)riscv$(DELIM)NMSIS$(DELIM)Library$(DELIM)DSP$(DELIM)GCC
EXTRA_LIBS += --start-group
EXTRA_LIBS += -l$(GDWIFI_MACLIB) $(foreach l,$(GDWIFI_WPASLIB),-l$(l)) -lrf
EXTRA_LIBS += $(if $(GDBLE_LIB),-l$(GDBLE_LIB))
EXTRA_LIBS += -lnmsis_dsp_rv32imafc
EXTRA_LIBS += --end-group

# The prebuilt supplicant calls setvbuf(stdout) during init; kernel-side
# tasks have no initialized stdio streams and dead-lock on the stream
# mutex.  Buffering is irrelevant here - neutralize it.

LDFLAGS += --wrap=setvbuf
