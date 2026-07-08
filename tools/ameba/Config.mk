############################################################################
# tools/ameba/Config.mk
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
#
# Realtek Ameba WHC flashing support.
#
# Including this fragment in a board's scripts/Make.defs (after
# ameba_board.mk) adds the `flash` target.  Usage:
#
#   make flash AMEBA_PORT=/dev/ttyUSB0 [ AMEBA_BAUD=1500000 ]
#
# The board must set AMEBA_FLASH_PROFILE to the profile base name
# (e.g. "RTL8721Dx" or "RTL8720F") before including this file.
#
# AMEBA_PORT  -- serial port (required)
# AMEBA_BAUD  -- baud rate (default 1500000)
#
# Dependencies:
#   The ameba-rtos SDK must already be fetched (auto-fetched on first
#   build) and its python venv or dep-ful system python must be on PATH
#   so that AmebaFlash.py (from the SDK) can import its dependencies.

# AMEBA_BAUD  -- serial baud rate (default 1500000)

AMEBA_BAUD ?= 1500000

# Flash offsets are NOT hardcoded here: they are read from the SDK flash
# layout, surfaced in the regenerated platform_autoconf.h (AMEBA_AUTOCONF)
# on every build.  This way a layout change (e.g. a bigger boot region that
# pushes the app offset) is tracked automatically -- the .rdev profile only
# supplies the on-chip flash-loader firmware, not the image placement.

# The app-slot macro is spelled CONFIG_FLASH_APP_OTA1_OFFSET on some ICs
# (e.g. RTL8720F) and CONFIG_FLASH_OTA1_OFFSET on others (e.g. RTL8721Dx);
# match either.  boot is CONFIG_FLASH_BOOT_OFFSET on all.

AMEBA_BOOT_OFFSET = $(strip $(shell awk '$$2=="CONFIG_FLASH_BOOT_OFFSET"{print $$3}' $(AMEBA_AUTOCONF) 2>/dev/null))
AMEBA_APP_OFFSET  = $(strip $(shell awk '$$2 ~ /^CONFIG_FLASH_(APP_)?OTA1_OFFSET$$/{print $$3}' $(AMEBA_AUTOCONF) 2>/dev/null))

# FLASH -- Download the built images via the SDK's AmebaFlash.py
#
# Two images are written, each at its own offset:
#   boot.bin  (SDK bootloader, prebuilt) @ CONFIG_FLASH_BOOT_OFFSET
#   nuttx.bin (AP+NP application image)   @ CONFIG_FLASH_APP_OTA1_OFFSET
# boot.bin stays in the board prebuilt/ dir; only nuttx.bin lives in $(TOPDIR).

define FLASH
	$(Q) if [ -z "$(AMEBA_PORT)" ]; then \
		echo "FLASH error: Missing serial port device argument."; \
		echo "USAGE: make flash AMEBA_PORT=/dev/ttyUSB0 [ AMEBA_BAUD=$(AMEBA_BAUD) ]"; \
		exit 1; \
	fi
	$(Q) if [ -z "$(AMEBA_BOOT_OFFSET)" ] || [ -z "$(AMEBA_APP_OFFSET)" ]; then \
		echo "FLASH error: could not read flash offsets from"; \
		echo "  $(AMEBA_AUTOCONF)"; \
		echo "  Run make first so platform_autoconf.h is generated."; \
		exit 1; \
	fi
	$(Q) AMEBAPY="$$(cat $(AMEBA_SDK)/.amebapy/bindir 2>/dev/null)/python"; \
		if [ ! -x "$$AMEBAPY" ]; then \
			AMEBAPY="python3"; \
		fi; \
		SCRIPT="$(AMEBA_SDK)/tools/ameba/Flash/AmebaFlash.py"; \
		if [ ! -f "$$SCRIPT" ]; then \
			echo "FLASH error: AmebaFlash.py not found at $$SCRIPT"; \
			echo "  Has the ameba-rtos SDK been fetched?  Run make first."; \
			exit 1; \
		fi; \
		PROFILE="$(AMEBA_SDK)/tools/ameba/Flash/Devices/Profiles/$(AMEBA_FLASH_PROFILE).rdev"; \
		if [ ! -f "$$PROFILE" ]; then \
			echo "FLASH error: profile not found: $$PROFILE"; \
			echo "  Set AMEBA_FLASH_PROFILE in the board's Make.defs."; \
			exit 1; \
		fi; \
		for spec in "$(AMEBA_PREBUILT)/boot.bin:$(AMEBA_BOOT_OFFSET)" \
		            "$(TOPDIR)/nuttx.bin:$(AMEBA_APP_OFFSET)"; do \
			IMG="$${spec%:*}"; ADDR="$${spec##*:}"; \
			if [ ! -f "$$IMG" ]; then \
				echo "FLASH error: image not found: $$IMG (run make first)"; \
				exit 1; \
			fi; \
			echo "FLASH: $$IMG @ $$ADDR"; \
			$$AMEBAPY "$$SCRIPT" \
				--download \
				--profile "$$PROFILE" \
				--memory-type nor \
				--image "$$IMG" \
				--start-address "$$ADDR" \
				--port "$(AMEBA_PORT)" \
				--baudrate "$(or $(AMEBA_BAUD),1500000)" \
				--log-level info || exit 1; \
		done
endef
