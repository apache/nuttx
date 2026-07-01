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

# FLASH -- Download the built images via the SDK's AmebaFlash.py

define FLASH
	$(Q) if [ -z "$(AMEBA_PORT)" ]; then \
		echo "FLASH error: Missing serial port device argument."; \
		echo "USAGE: make flash AMEBA_PORT=/dev/ttyUSB0 [ AMEBA_BAUD=$(AMEBA_BAUD) ]"; \
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
		$$AMEBAPY "$$SCRIPT" \
			--download \
			--profile "$$PROFILE" \
			--image-dir "$(TOPDIR)" \
			--port "$(AMEBA_PORT)" \
			--baudrate "$(or $(AMEBA_BAUD),1500000)" \
			--log-level info
endef
