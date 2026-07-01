############################################################################
# arch/arm/src/rtl8721dx/ameba_board.mk
#
# Shared board-build definitions for the RTL8721Dx (Ameba WHC) -- the SDK
# include sets, fwlib/WiFi source lists, image2 linker-script generation,
# PREBUILD and POSTBUILD.  A board's scripts/Make.defs is a thin wrapper that
# just `include`s this file, so every RTL8721Dx board shares one definition
# (and a second board on this IC needs no copy).  Board-specific paths come
# from $(BOARD_DIR), so each board keeps its own prebuilt/ staging dir.
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

include $(TOPDIR)/.config
include $(TOPDIR)/tools/Config.mk

# Resolve (and, when AMEBA_SDK is unset, auto-fetch) the one shared ameba-rtos
# SDK, and prepend the SDK-matched asdk toolchain to PATH.  This MUST precede
# Toolchain.defs: that file probes the compiler version to compute flags (e.g.
# --param=min-pagesize=0 on newer GCC), so the SDK toolchain has to be on PATH
# first or the flags get computed for the wrong compiler.  AMEBA_SOC_NAME
# selects this IC's SoC subtree.

AMEBA_SOC_NAME = amebadplus
include $(TOPDIR)/arch/arm/src/common/ameba/ameba_sdk.mk

include $(TOPDIR)/arch/arm/src/armv8-m/Toolchain.defs

############################################################################
# Linker script
#
# The `nuttx` ELF must be linked as the Realtek KM4 image2 .axf, so the link
# uses the *SDK's own* layout + image2 + ROM-symbol linker scripts (exactly as
# the prototype link_img2.sh did):
#
#   ameba_layout.ld  (included by ameba_img2_all.ld)
#   ameba_img2_all.ld
#   ameba_rom_symbol_acut_s.ld   (CONFIG_LINK_ROM_SYMB)
#
# ameba_img2_all.ld is C-preprocessed (it #includes ameba_layout.ld and the
# generated platform_autoconf.h), then ameba_rom_symbol_acut_s.ld is appended.
# Finally NuttX's own .vectors orphan section is folded into the loadable SRAM
# data region so the (large power-of-two aligned) vector table does not land in
# and overflow the tiny fixed KM4_IMG2_ENTRY region.  This reproduces the
# generated rlx8721d.ld byte-for-byte without editing any SDK source file
# (the SDK tree stays read-only).
#
# The combined script is generated into scripts/ as ld.script and used as the
# single ARCHSCRIPT.  ld.script content is already fully expanded (no macros,
# no #include), so the standard ARCHSCRIPT cpp .tmp pass is idempotent.
############################################################################

# SDK linker-script sources and the vendored, config-derived autoconf header.

AMEBA_SOC       = $(AMEBA_SDK)/component/soc/$(AMEBA_SOC_NAME)
AMEBA_PROJ      = $(AMEBA_SOC)/project
AMEBA_KM4_LD    = $(AMEBA_PROJ)/project_km4/ld
AMEBA_LAYOUT_LD = $(AMEBA_PROJ)/ameba_layout.ld
AMEBA_IMG2_LD   = $(AMEBA_KM4_LD)/ameba_img2_all.ld
AMEBA_ROM_LD    = $(AMEBA_KM4_LD)/ameba_rom_symbol_acut_s.ld

AMEBA_PREBUILT      = $(BOARD_DIR)$(DELIM)prebuilt
AMEBA_PREBUILT_LIBS = $(AMEBA_PREBUILT)$(DELIM)libs
AMEBA_AUTOCONF      = $(AMEBA_PREBUILT)$(DELIM)platform_autoconf.h

# fwlib register-layer objects, compiled FROM SDK SOURCE into libameba_fwlib.a
# (linked via arch/.../rtl8721dx/Make.defs EXTRA_LIBS) instead of vendoring a
# prebuilt blob -- so a bare checkout that only auto-fetched the SDK source can
# still link.  Grow AMEBA_FWLIB_SRCS as NuttX drivers call more fwlib functions;
# --gc-sections keeps only what is referenced.  AMEBA_FWLIB_INC is the SDK's
# fwlib include set, kept scoped to this compile so SDK headers never leak into
# the NuttX core build (where they would clash).

AMEBA_FWLIB_A    = $(AMEBA_PREBUILT_LIBS)$(DELIM)libameba_fwlib.a
AMEBA_FWLIB_SRCS = $(AMEBA_SOC)/fwlib/ram_common/ameba_arch.c \
                   $(AMEBA_SOC)/fwlib/ram_common/ameba_loguart.c \
                   $(AMEBA_SOC)/fwlib/ram_common/ameba_ipc_api.c \
                   $(AMEBA_SOC)/fwlib/ram_common/ameba_ipc_ram.c \
                   $(AMEBA_SOC)/misc/ameba_pmu.c \
                   $(AMEBA_SOC)/swlib/log.c \
                   $(AMEBA_SOC)/swlib/sscanf_minimal.c

# NuttX owns the image2 entry: arch/arm/src/rtl8721dx/ameba_app_start.c is a
# NuttX adaptation of the SDK app_start() that runs all OS-independent silicon
# init (cache/MPU/brown-out/data-flash/...), zeroes BSS, then calls NuttX's
# main() (ameba_start.c).  It is built with the SDK fwlib include set, so the
# fwlib sources it pulls in are listed alongside it.
AMEBA_FWLIB_SRCS += $(TOPDIR)/arch/arm/src/rtl8721dx/ameba_app_start.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_mpu_ram.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_bor.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_reset.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_clk.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_rtc_io.c \
                    $(AMEBA_SOC)/fwlib/ram_common/ameba_adc.c \
                    $(AMEBA_SOC)/fwlib/ram_km4/ameba_data_flashclk.c

# Flash primitives (FLASH_ReadStream/WriteStream/EraseXIP) for the MTD data
# filesystem.  These self-lock against the NP core (inter-core HW semaphore +
# IPC pause) and disable IRQs around erase/program.
ifeq ($(CONFIG_RTL8721DX_FLASH_FS),y)
AMEBA_FWLIB_SRCS += $(AMEBA_SOC)/fwlib/ram_common/ameba_flash_ram.c
endif
AMEBA_FWLIB_INC  = -mcmse \
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
                   -I$(AMEBA_SDK)/component/soc/common/crashdump/include \
                   -I$(AMEBA_PREBUILT)

# --- WiFi (WHC host) glue lib (CONFIG_RTL8721DX_WIFI) ----------------------
#
# libameba_wifi.a holds the NuttX-side glue the precompiled host WiFi libs
# expect: a couple of SDK config sources (the user WiFi config + the task-size
# table, compiled from source so their struct layouts match the libs' ABI) and
# the NuttX lwIP/event shim (arch/.../wifi/ameba_wifi_depend.c).  It is built
# with the vendor WiFi include set -- which collides with NuttX's own headers
# (lwIP vs NuttX atomic.h, ...) -- so it is compiled in its own PREBUILD loop
# (like libameba_fwlib.a), NOT through the normal arch CSRCS path.  The shim
# include dir comes FIRST so its minimal <lwip_netconf.h> shadows the SDK's.
AMEBA_WIFI_A    = $(AMEBA_PREBUILT_LIBS)$(DELIM)libameba_wifi.a
AMEBA_WIFI_DIR  = $(TOPDIR)$(DELIM)arch$(DELIM)arm$(DELIM)src$(DELIM)common$(DELIM)ameba$(DELIM)wifi
AMEBA_WIFI_SRCS = $(AMEBA_SDK)/component/soc/usrcfg/$(AMEBA_SOC_NAME)/ameba_wificfg.c \
                  $(AMEBA_SDK)/component/wifi/common/rtw_task_size.c \
                  $(AMEBA_SDK)/component/wifi/common/rtw_event.c \
                  $(AMEBA_SDK)/component/soc/common/diagnose/ameba_diagnose_none.c \
                  $(AMEBA_SDK)/component/wifi/wpa_supplicant/wpa_supplicant/wifi_p2p_disable.c \
                  $(AMEBA_WIFI_DIR)/ameba_wifi_depend.c \
                  $(AMEBA_WIFI_DIR)/ameba_wifi.c
#
# Force-include the SDK autoconf so EVERY wifi source sees CONFIG_WHC_HOST /
# CONFIG_WHC_INTF_IPC / ... (the SDK build force-includes platform_autoconf.h
# globally).  Without this, sources that don't #include it themselves silently
# compile the wrong #if branch -- e.g. rtw_task_size.c's wifi_set_task_size()
# becomes empty, leaving g_rtw_task_size all-zero, so the WHC host event tasks
# are created with stack size 0 and never run -> wifi_connect() blocks forever
# waiting for the join-status event.
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
                  -I$(AMEBA_SDK)/component/ssl/mbedtls-3.6.5/include \
                  -I$(AMEBA_PREBUILT)

# The combined script is generated under prebuilt/ (gitignored) by PREBUILD
# (below) and used as the single ARCHSCRIPT.  Its content is already fully
# expanded (no macros, no #include), so the standard ARCHSCRIPT cpp .tmp pass
# is idempotent.  ARCHSCRIPT must name a plain file here (not a rule target):
# defining an explicit recipe in this globally-included Make.defs would hijack
# the default make goal, so the generation is done in PREBUILD instead.

GENLDSCRIPT = $(AMEBA_PREBUILT)$(DELIM)ld.script.gen

ARCHSCRIPT += $(GENLDSCRIPT)

# NP (network core, km0 on this IC) image + bootloader.  AP/NP share many SDK
# sources, so the NP image must come from the SAME pinned SDK -- a fixed release
# bin would drift.
#
# The NP (km0) image and boot are ALWAYS (re)built from the pinned SDK source on
# every NuttX build -- no "use a stale vendored blob" shortcut -- so the two
# cores can never drift out of alignment.  The NP build runs AFTER the AP (km4 /
# NuttX) link because the SDK WiFi "noused" generator strips any host WiFi API
# the AP image does not reference (calling a stripped API later deadlocks the NP
# -- "Compile NP after AP!").  So it is a POSTBUILD step handed NuttX's own
# disassembly (target_img2.asm, produced earlier in POSTBUILD) as the AP image
# via AMEBA_AP_ASM.
#
# AMEBA_PY_SOC is the ameba.py SoC identifier for this board (public name),
# distinct from AMEBA_SOC_NAME (the SDK soc/ subdir).
AMEBA_PY_SOC      = RTL8721Dx
AMEBA_BUILD_NP_SH = $(AMEBA_COMMON_DIR)$(DELIM)tools$(DELIM)ameba_build_np.sh

# platform_autoconf.h is regenerated from the SDK menuconfig on every build (see
# ameba_gen_autoconf.sh) -- the SDK Kconfig is the single source of truth for
# the flash layout (CONFIG_FLASH_VFS1_*) and feature switches (CONFIG_WHC_* ...).
# AMEBA_AP_PROJECT is the AP-core SDK project subdir (km4 on amebadplus).
AMEBA_AP_PROJECT   = km4
AMEBA_GEN_AUTOCONF = $(AMEBA_COMMON_DIR)$(DELIM)tools$(DELIM)ameba_gen_autoconf.sh
AMEBA_NP_PREBUILD  = true
AMEBA_NP_POSTBUILD = $(AMEBA_BUILD_NP_SH) $(AMEBA_SDK) $(AMEBA_PY_SOC) \
                      $(AMEBA_PREBUILT) $(AMEBA_PKGDIR)$(DELIM)target_img2.asm \
                      km0 $(AMEBA_AP_PROJECT)

# PREBUILD -- (re)generate the combined image2 linker script before the build.
#
# ameba_img2_all.ld is C-preprocessed (it #includes ameba_layout.ld and the
# vendored, config-derived platform_autoconf.h, staged as
# project_km4/platform_autoconf.h on the include path), then
# ameba_rom_symbol_acut_s.ld is appended, then NuttX's .vectors orphan is folded
# into the loadable SRAM data region.  This reproduces the SDK-generated
# rlx8721d.ld without editing any SDK source file.

define PREBUILD
	$(Q) test -n "$(AMEBA_SDK)" || \
	  { echo "ERROR: AMEBA_SDK is not set. Export it to your ameba-rtos checkout."; exit 1; }
	$(Q) $(AMEBA_FETCH_TOOLCHAIN) $(AMEBA_SDK) $(AMEBA_TOOLCHAIN_DIR)
	$(Q) $(AMEBA_SETUP_ENV) $(AMEBA_SDK) $(AMEBA_TOOLCHAIN_DIR)
	$(Q) $(AMEBA_GEN_AUTOCONF) $(AMEBA_SDK) $(AMEBA_PY_SOC) $(AMEBA_AP_PROJECT) \
	     $(AMEBA_AUTOCONF)
	$(Q) $(AMEBA_NP_PREBUILD)
	$(Q) echo "GEN: libameba_fwlib.a (fwlib from SDK source)"
	$(Q) mkdir -p $(AMEBA_PREBUILT_LIBS)$(DELIM)fwlib_obj
	$(Q) rebuild_fwlib=0; \
	     for src in $(AMEBA_FWLIB_SRCS); do \
	       obj=$(AMEBA_PREBUILT_LIBS)/fwlib_obj/`basename $$src .c`.o; \
	       if [ "$$src" -nt "$$obj" ] 2>/dev/null || [ ! -f "$$obj" ]; then \
	         $(CC) $(ARCHCPUFLAGS) -Os -ffunction-sections -fdata-sections \
	           $(AMEBA_FWLIB_INC) -c $$src -o $$obj || exit 1; \
	         rebuild_fwlib=1; \
	       fi; \
	     done; \
	     if [ "$$rebuild_fwlib" = "1" ] || [ ! -f "$(AMEBA_FWLIB_A)" ]; then \
	       $(CROSSDEV)ar crs $(AMEBA_FWLIB_A) $(AMEBA_PREBUILT_LIBS)/fwlib_obj/*.o; \
	     fi
	$(Q) if [ "$(CONFIG_RTL8721DX_WIFI)" = "y" ]; then \
	       echo "GEN: libameba_wifi.a (WHC host glue from SDK source + NuttX shim)"; \
	       mkdir -p $(AMEBA_PREBUILT_LIBS)$(DELIM)wifi_obj; \
	       rebuild_wifi=0; \
	       for src in $(AMEBA_WIFI_SRCS); do \
	         obj=$(AMEBA_PREBUILT_LIBS)/wifi_obj/`basename $$src .c`.o; \
	         if [ "$$src" -nt "$$obj" ] 2>/dev/null || [ ! -f "$$obj" ]; then \
	           $(CC) $(ARCHCPUFLAGS) -Os -ffunction-sections -fdata-sections \
	             $(AMEBA_WIFI_INC) -c $$src -o $$obj || exit 1; \
	           rebuild_wifi=1; \
	         fi; \
	       done; \
	       if [ "$$rebuild_wifi" = "1" ] || [ ! -f "$(AMEBA_WIFI_A)" ]; then \
	         $(CROSSDEV)ar crs $(AMEBA_WIFI_A) $(AMEBA_PREBUILT_LIBS)/wifi_obj/*.o; \
	       fi; \
	     fi
	$(Q) echo "GEN: ld.script.gen (Ameba AP km4 image2)"
	$(Q) mkdir -p $(AMEBA_PREBUILT)$(DELIM)project_km4
	$(Q) cp $(AMEBA_AUTOCONF) $(AMEBA_PREBUILT)$(DELIM)project_km4$(DELIM)platform_autoconf.h
	$(Q) $(CC) -E -P -xc -c $(AMEBA_IMG2_LD) -o $(GENLDSCRIPT) -I $(AMEBA_PREBUILT)
	$(Q) cat $(AMEBA_ROM_LD) >> $(GENLDSCRIPT)
	$(Q) sed -i 's|^\([[:space:]]*\)\*(\.data\*)|\1*(.vectors*)\n\1*(.data*)|' $(GENLDSCRIPT)
endef

CFLAGS := $(ARCHCFLAGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES)
CPICFLAGS = $(ARCHPICFLAGS) $(CFLAGS)
CXXFLAGS := $(ARCHCXXFLAGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHXXINCLUDES) $(ARCHDEFINES)
CXXPICFLAGS = $(ARCHPICFLAGS) $(CXXFLAGS)
CPPFLAGS := $(ARCHINCLUDES) $(ARCHDEFINES)

CFLAGS   += $(EXTRAFLAGS)
CPPFLAGS += $(EXTRAFLAGS)

# VFS1 data-partition geometry for the littlefs MTD comes from the SDK flash
# layout (CONFIG_FLASH_VFS1_OFFSET/SIZE in the regenerated platform_autoconf.h),
# so ameba_flash_mtd.c never hardcodes it.  Extract the two values and pass them
# as neutral -D macros (a NuttX C file must not force-include the SDK autoconf).
# The header falls back to the vendor default if these are absent (e.g. the very
# first parse on a clean clone, before PREBUILD has generated the autoconf).
ifneq ($(wildcard $(AMEBA_AUTOCONF)),)
AMEBA_VFS1_OFFSET := $(strip $(shell awk '$$2=="CONFIG_FLASH_VFS1_OFFSET"{print $$3}' $(AMEBA_AUTOCONF)))
AMEBA_VFS1_SIZE   := $(strip $(shell awk '$$2=="CONFIG_FLASH_VFS1_SIZE"{print $$3}' $(AMEBA_AUTOCONF)))
CFLAGS += $(if $(AMEBA_VFS1_OFFSET),-DAMEBA_FLASH_VFS1_OFFSET_XIP=$(AMEBA_VFS1_OFFSET))
CFLAGS += $(if $(AMEBA_VFS1_SIZE),-DAMEBA_FLASH_VFS1_SIZE_CFG=$(AMEBA_VFS1_SIZE))
endif

AFLAGS := $(CFLAGS) -D__ASSEMBLY__

# NXFLAT module definitions

NXFLATLDFLAGS1 = -r -d -warn-common
NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)$(DELIM)binfmt$(DELIM)libnxflat$(DELIM)gnu-nxflat-pcrel.ld -no-check-sections
LDNXFLATFLAGS = -e main -s 2048

############################################################################
# POSTBUILD -- package the linked image2 into a flashable app.bin
#
# Runs after `nuttx` (== the KM4 image2 .axf) is linked.  This folds the
# prototype package_app.sh into NuttX's build:
#
#   1. Replicate the SDK image2 postbuild prologue (copy map/axf, nm/objdump).
#   2. Run the SDK AP (km4) image2 postbuild.cmake (axf2bin) -> km4_image2_all.bin.
#   3. Run the SDK project firmware_package postbuild.cmake combining the
#      freshly built AP (km4) image2 with the freshly built NP (km0) image2 ->
#      app.bin, copied into $(TOPDIR).
#
# The NP (km0) image2 is (re)built from the pinned SDK source in the NP POSTBUILD
# step above (NuttX only owns the AP / km4 image2).  The SDK scripts/cmake/python
# are invoked read-only; only artefacts under the build staging dir are written.
#
# Tooling resolved from the asdk toolchain on PATH and the python interpreter
# ameba_setup_env.sh resolved (recorded in .amebapy/bindir: a real SDK .venv
# when one exists, otherwise a system-python3 shim).  CMAKE may be overridden
# via the CMAKE environment variable; it defaults to `cmake` on PATH.  That bin
# dir is prepended to PATH for the SDK cmake/axf2bin steps so `python`/`python3`
# resolve to the deps-carrying interpreter.  It is read at recipe run time (the
# `$$(cat ...)` defers until after ameba_setup_env.sh has written the file).
############################################################################

AMEBA_VENV_BIN = $$(cat $(AMEBA_SDK)/.amebapy/bindir 2>/dev/null)

CMAKE   ?= cmake
NM      ?= $(CROSSDEV)nm
OBJDUMP ?= $(CROSSDEV)objdump
STRIP   ?= $(CROSSDEV)strip
OBJCOPY ?= $(CROSSDEV)objcopy
SIZE    ?= $(CROSSDEV)size

AMEBA_SOC_PROJ = $(AMEBA_SOC)/project
AMEBA_KM4_PROJ = $(AMEBA_SOC_PROJ)/project_km4

# Staging dir for the SDK packaging steps (under the board, gitignored).

AMEBA_PKGDIR = $(AMEBA_PREBUILT)$(DELIM)pkg

define POSTBUILD
	$(Q) echo "PACK: app.bin (Ameba AP km4 image2 + NP km0 image2)"
	$(Q) test -n "$(AMEBA_SDK)" || \
	  { echo "ERROR: AMEBA_SDK is not set"; exit 1; }
	$(Q) $(AMEBA_SETUP_ENV) $(AMEBA_SDK) $(AMEBA_TOOLCHAIN_DIR)
	$(Q) rm -rf $(AMEBA_PKGDIR)
	$(Q) mkdir -p $(AMEBA_PKGDIR)
	$(Q) cp $(TOPDIR)$(DELIM)nuttx $(AMEBA_PKGDIR)$(DELIM)target_img2.axf
	$(Q) $(NM) $(AMEBA_PKGDIR)$(DELIM)target_img2.axf | sort \
	  > $(AMEBA_PKGDIR)$(DELIM)target_img2.map
	$(Q) $(OBJDUMP) -d $(AMEBA_PKGDIR)$(DELIM)target_img2.axf \
	  > $(AMEBA_PKGDIR)$(DELIM)target_img2.asm
	$(Q) $(AMEBA_NP_POSTBUILD)
	$(Q) cp $(AMEBA_PKGDIR)$(DELIM)target_img2.axf \
	  $(AMEBA_PKGDIR)$(DELIM)target_pure_img2.axf
	$(Q) $(STRIP) $(AMEBA_PKGDIR)$(DELIM)target_pure_img2.axf
	$(Q) cp $(AMEBA_PREBUILT)$(DELIM)config_km4 $(AMEBA_PKGDIR)$(DELIM).config_km4
	$(Q) cp $(AMEBA_PREBUILT)$(DELIM)config_fw $(AMEBA_PKGDIR)$(DELIM).config
	$(Q) cp $(AMEBA_PREBUILT)$(DELIM)km0_image2_all.bin \
	  $(AMEBA_PKGDIR)$(DELIM)km0_image2_all.bin
	$(Q) printf '{\n    "soc": {\n        "name": "RTL8721Dx"\n    }\n}\n' \
	  > $(AMEBA_PKGDIR)$(DELIM)soc_info.json
	$(Q) TARGET_SOC=RTL8721Dx PATH=$(AMEBA_VENV_BIN):$$PATH $(CMAKE) \
	  -Dc_BASEDIR=$(AMEBA_SDK) \
	  -Dc_CMAKE_FILES_DIR=$(AMEBA_SDK)/cmake \
	  -Dc_SOC_PROJECT_DIR=$(AMEBA_SOC_PROJ) \
	  -Dc_MCU_PROJECT_DIR=$(AMEBA_KM4_PROJ) \
	  -Dc_MCU_PROJECT_NAME=km4 \
	  -Dc_MCU_KCONFIG_FILE=$(AMEBA_PKGDIR)$(DELIM).config_km4 \
	  -Dc_SDK_IMAGE_TARGET_DIR=$(AMEBA_PKGDIR) \
	  -DKM4_BUILDDIR= \
	  -DFINAL_IMAGE_DIR=$(AMEBA_PKGDIR) \
	  -DBUILD_TYPE=NONE -DANALYZE_MP_IMG=0 -DDAILY_BUILD=0 \
	  -DEXTERN_DIR=$(AMEBA_PKGDIR) -DCODE_ANALYZE_RETRY= \
	  -DIMAGESCRIPTDIR=$(AMEBA_SDK)/tools/image_scripts \
	  -DCMAKE_SIZE=$(SIZE) -DCMAKE_OBJCOPY=$(OBJCOPY) \
	  -P $(AMEBA_KM4_PROJ)/make/image2/postbuild.cmake
	$(Q) TARGET_SOC=RTL8721Dx PATH=$(AMEBA_VENV_BIN):$$PATH $(CMAKE) \
	  -Dc_BASEDIR=$(AMEBA_SDK) \
	  -Dc_CMAKE_FILES_DIR=$(AMEBA_SDK)/cmake \
	  -Dc_MCU_KCONFIG_FILE=$(AMEBA_PKGDIR)$(DELIM).config \
	  -Dc_SOC_PROJECT_DIR=$(AMEBA_SOC_PROJ) \
	  -Dc_SDK_IMAGE_TARGET_DIR= \
	  -Dc_IMAGE_OUTPUT_DIR=$(AMEBA_PKGDIR) \
	  -Dc_APP_BINARY_NAME=app.bin \
	  -Dc_IMAGE1_ALL_FILES= \
	  -Dc_IMAGE2_ALL_FILES="$(AMEBA_PKGDIR)/km0_image2_all.bin;$(AMEBA_PKGDIR)/km4_image2_all.bin" \
	  -Dc_IMAGE3_ALL_FILES= \
	  -DFINAL_IMAGE_DIR=$(TOPDIR) \
	  -DANALYZE_MP_IMG=0 -DEXTERN_DIR=$(AMEBA_PKGDIR) \
	  -P $(AMEBA_SOC_PROJ)/postbuild.cmake
	$(Q) cp $(AMEBA_PREBUILT)$(DELIM)boot.bin $(TOPDIR)$(DELIM)boot.bin
	$(Q) cp $(AMEBA_PREBUILT)$(DELIM)km0_image2_all.bin \
	  $(TOPDIR)$(DELIM)km0_image2_all.bin
	$(Q) echo "PACK: wrote $(TOPDIR)$(DELIM){app.bin,boot.bin,km0_image2_all.bin}"
endef
