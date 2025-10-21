############################################################################
# tools/espressif/Config.mk
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

# Remove quotes from CONFIG_ESPRESSIF_CHIP_SERIES configuration

CHIP_SERIES = $(patsubst "%",%,$(CONFIG_ESPRESSIF_CHIP_SERIES))

# MCUBoot requires a region in flash for the E-Fuse virtual mode.
# To avoid erasing this region, flash a dummy empty file to the
# virtual E-Fuse offset.

VIRTUAL_EFUSE_BIN := vefuse.bin

ifeq ($(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH),y)
	EFUSE_FLASH_OFFSET := $(CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH_OFFSET)
else
	EFUSE_FLASH_OFFSET := 0x10000
endif

# These are the macros that will be used in the NuttX make system to compile
# and assemble source files and to insert the resulting object files into an
# archive.  These replace the default definitions at tools/Config.mk

ifeq ($(CONFIG_ESPRESSIF_FLASH_2M),y)
	FLASH_SIZE := 2MB
else ifeq ($(CONFIG_ESPRESSIF_FLASH_4M),y)
	FLASH_SIZE := 4MB
else ifeq ($(CONFIG_ESPRESSIF_FLASH_8M),y)
	FLASH_SIZE := 8MB
else ifeq ($(CONFIG_ESPRESSIF_FLASH_16M),y)
	FLASH_SIZE := 16MB
endif

ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_DIO),y)
	FLASH_MODE := dio
else ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_DOUT),y)
	FLASH_MODE := dout
else ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_QIO),y)
	FLASH_MODE := qio
else ifeq ($(CONFIG_ESPRESSIF_FLASH_MODE_QOUT),y)
	FLASH_MODE := qout
endif

FLASH_FREQ := $(CONFIG_ESPRESSIF_FLASH_FREQ)

ifeq ($(CONFIG_ESPRESSIF_FLASH_DETECT),y)
	ESPTOOL_WRITEFLASH_OPTS := -fs detect -fm dio -ff $(FLASH_FREQ)
else
	ESPTOOL_WRITEFLASH_OPTS := -fs $(FLASH_SIZE) -fm dio -ff $(FLASH_FREQ)
endif

ifeq ($(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED),y)
	ESPTOOL_WRITEFLASH_OPTS += --encrypt
endif

# Configure the variables according to build environment

ESPTOOL_MIN_VERSION := 4.8.0

ifdef ESPTOOL_BINDIR
	ifeq ($(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),y)
		BL_OFFSET       := 0x0000
		BOOTLOADER      := $(ESPTOOL_BINDIR)/mcuboot-$(CHIP_SERIES).bin
		FLASH_BL        := $(BL_OFFSET) $(BOOTLOADER)
		ESPTOOL_BINS    := $(FLASH_BL)
	endif
endif

define MAKE_VIRTUAL_EFUSE_BIN
	$(Q)if [ ! -f "$(VIRTUAL_EFUSE_BIN)" ]; then \
		dd if=/dev/zero of=$(VIRTUAL_EFUSE_BIN) count=0 status=none; \
	fi
endef

ifeq ($(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),y)

	ESPTOOL_BINS += $(EFUSE_FLASH_OFFSET) $(VIRTUAL_EFUSE_BIN)

	ifeq ($(CONFIG_ESPRESSIF_ESPTOOL_TARGET_PRIMARY),y)
		VERIFIED   := --confirm
		APP_OFFSET := $(CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET)
	else ifeq ($(CONFIG_ESPRESSIF_ESPTOOL_TARGET_SECONDARY),y)
		VERIFIED   :=
		APP_OFFSET := $(CONFIG_ESPRESSIF_OTA_SECONDARY_SLOT_OFFSET)
	endif

	APP_IMAGE      := nuttx.bin
	FLASH_APP      := $(APP_OFFSET) $(APP_IMAGE)

	ifeq ($(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED),y)
		ifeq ($(CONFIG_ESPRESSIF_SPIFLASH),y)
			ENC_APP := $(CONFIG_ESPRESSIF_STORAGE_MTD_OFFSET) enc_mtd.bin
		endif
	endif

	ifeq ($(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED),y)
		IMGTOOL_ALIGN_ARGS := --align 32 --max-align 32
	else
		IMGTOOL_ALIGN_ARGS := --align 4
	endif

	IMGTOOL_SIGN_ARGS  := --pad $(VERIFIED) $(IMGTOOL_ALIGN_ARGS) -v $(CONFIG_ESPRESSIF_MCUBOOT_SIGN_IMAGE_VERSION) -s auto \
		-H $(CONFIG_ESPRESSIF_APP_MCUBOOT_HEADER_SIZE) --pad-header \
		-S $(CONFIG_ESPRESSIF_OTA_SLOT_SIZE)
else ifeq ($(CONFIG_ESPRESSIF_SIMPLE_BOOT),y)
	APP_OFFSET     := 0x0000
	APP_IMAGE      := nuttx.bin
	FLASH_APP      := $(APP_OFFSET) $(APP_IMAGE)
	ESPTOOL_BINDIR := .
endif

ESPTOOL_BINS += $(FLASH_APP) $(ENC_APP)

# Commands for colored and formatted output

RED    = \033[1;31m
YELLOW = \033[1;33m
BOLD   = \033[1m
RST    = \033[0m

# Flash encryption procedure

define FLASH_ENC
	$(Q) echo -e "$(YELLOW)Flash Encryption is enabled!$(RST)";

	$(Q) if [ "$(CONFIG_ESPRESSIF_EFUSE_VIRTUAL)" = "y" ]; then \
		echo -e "$(YELLOW)WARN: Virtual E-Fuses are enabled! E-Fuses will not be burned. $(RST)"; \
	fi

	$(Q) if [ "$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_USE_HOST_KEY)" = "y" ]; then \
		if [ ! -f "$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME)" ]; then \
			echo -e "$(RED)FLASH ENCRYPTION error: Key file '$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME)' not found.$(RST)"; \
			echo -e "$(YELLOW)Generate the encryption key using: espsecure.py generate_flash_encryption_key <key_name.bin>$(RST)"; \
			echo -e "$(YELLOW)Refer to the documentation on flash encryption before proceeding.$(RST)"; \
			exit 1; \
		fi; \
	fi

	$(Q) if [ "$(CONFIG_ESPRESSIF_SPIFLASH)" = "y" ]; then \
		echo "Applying encryption to user MTD partition on flash."; \
		if [ ! -f "$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME)" ]; then \
			echo -e "$(RED)Flash encryption key is required for user MTD partition encryption. Key file: '$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME)'$(RST)"; \
			echo -e "$(RED)Make sure CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME is set or disable SPI Flash.$(RST)"; \
			exit 1; \
		fi; \
		size_int=$$(( $(CONFIG_ESPRESSIF_STORAGE_MTD_SIZE) )); \
		echo -e "Encrypting user MTD partition offset: $(CONFIG_ESPRESSIF_STORAGE_MTD_OFFSET), size: $(CONFIG_ESPRESSIF_STORAGE_MTD_SIZE) ($$size_int)"; \
		dd if=/dev/zero ibs=1 count=$$size_int | LC_ALL=C tr "\000" "\377" > blank_mtd.bin; \
        espsecure.py encrypt_flash_data --aes_xts --keyfile $(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME) --address 0 --output enc_mtd.bin blank_mtd.bin; \
		rm blank_mtd.bin; \
	fi

endef

# BURN_EFUSES -- Burn the flash encryption key E-Fuses if: not already burned, not virtual, not device already encrypted

define BURN_EFUSES
	$(Q) if [ "$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED)" = "y" ]; then \
		echo -e "$(YELLOW)WARN: Device is already encrypted. Skipping flash encryption key burning. $(RST)"; \
	elif [ "$(CONFIG_ESPRESSIF_EFUSE_VIRTUAL)" = "y" ]; then \
		echo -e "$(YELLOW)WARN: Virtual E-Fuses are enabled! Skipping flash encryption key burning. $(RST)"; \
	else \
		if [ "$(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_USE_HOST_KEY)" = "y" ]; then \
			echo -e "$(YELLOW)Proceeding will burn the flash encryption key E-Fuses using: $(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME).$(RST)"; \
		else \
			echo -e "$(YELLOW)Proceeding will burn a *randomly generated* flash encryption key (NOT user-provided).$(RST)"; \
		fi; \
		echo -e "$(YELLOW)This operation is NOT REVERSIBLE! Make sure to have read the documentation.$(RST)"; \
		efuse_summary=$$(espefuse.py --port $(ESPTOOL_PORT) summary | grep -A 1 BLOCK1); \
		if echo "$$efuse_summary" | grep -q '?? ??'; then \
			echo -e "$(YELLOW)Encryption key already burned. Skipping...$(RST)"; \
		else \
			echo -e "$(YELLOW)Burning flash encryption key...$(RST)"; \
			if [ -z "$(NOCHECK)" ] ; then \
				espefuse.py --port $(ESPTOOL_PORT) burn_key BLOCK_KEY0 $(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME) XTS_AES_128_KEY; \
			else \
				espefuse.py --do-not-confirm --port $(ESPTOOL_PORT) burn_key BLOCK_KEY0 $(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME) XTS_AES_128_KEY; \
			fi; \
		fi; \
	fi
endef

# MERGEBIN -- Merge raw binary files into a single file

define MERGEBIN
	@python3 tools/espressif/check_esptool.py -v $(ESPTOOL_MIN_VERSION)
	$(Q) if [ -z $(ESPTOOL_BINDIR) ]; then \
		echo "MERGEBIN error: Missing argument for binary files directory."; \
		echo "USAGE: make ESPTOOL_BINDIR=<dir>"; \
		exit 1; \
	fi
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration."; \
		exit 1; \
	fi
	esptool.py -c $(CHIP_SERIES) merge_bin --fill-flash-size $(FLASH_SIZE) --output nuttx.merged.bin $(ESPTOOL_BINS)
	$(Q) echo nuttx.merged.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.merged.bin"
endef

# MKIMAGE -- Convert an ELF file into a compatible binary file

ifeq ($(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),y)
define MKIMAGE
	$(Q) echo "MKIMAGE: NuttX binary"
	$(Q) if ! imgtool version 1>/dev/null 2>&1; then \
		echo ""; \
		echo "imgtool not found.  Please run: \"pip install imgtool\""; \
		echo ""; \
		echo "Run make again to create the nuttx.bin image."; \
		exit 1; \
	fi
	imgtool sign $(IMGTOOL_SIGN_ARGS) nuttx.hex nuttx.bin
	$(Q) echo nuttx.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.bin (MCUboot compatible)"
endef
else
define MKIMAGE
	$(Q) echo "MKIMAGE: NuttX binary"
	@python3 tools/espressif/check_esptool.py -v $(ESPTOOL_MIN_VERSION)
	$(Q) if [ -z $(FLASH_SIZE) ]; then \
		echo "Missing Flash memory size configuration."; \
		exit 1; \
	fi
	$(eval ELF2IMAGE_OPTS := $(if $(CONFIG_ESPRESSIF_SIMPLE_BOOT),--ram-only-header) -fs $(FLASH_SIZE) -fm $(FLASH_MODE) -ff $(FLASH_FREQ))
	esptool.py -c $(CHIP_SERIES) elf2image $(ELF2IMAGE_OPTS) -o nuttx.bin nuttx
	$(Q) echo nuttx.bin >> nuttx.manifest
	$(Q) echo "Generated: nuttx.bin"
endef
endif

# POSTBUILD -- Perform post build operations

define POSTBUILD
	$(call MKIMAGE)
	$(if $(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT),$(call MAKE_VIRTUAL_EFUSE_BIN))
	$(if $(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED),$(call FLASH_ENC))
	$(if $(CONFIG_ESPRESSIF_MERGE_BINS),$(call MERGEBIN))
endef

# ESPTOOL_BAUD -- Serial port baud rate used when flashing/reading via esptool.py

ESPTOOL_BAUD ?= 921600

# FLASH -- Download a binary image via esptool.py

define FLASH
	$(Q) if [ -z $(ESPTOOL_PORT) ]; then \
		echo "FLASH error: Missing serial port device argument."; \
		echo "USAGE: make flash ESPTOOL_PORT=<port> [ ESPTOOL_BAUD=<baud> ]"; \
		exit 1; \
	fi

	$(if $(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED),$(call BURN_EFUSES))
	$(eval ESPTOOL_OPTS := -c $(CHIP_SERIES) -p $(ESPTOOL_PORT) -b $(ESPTOOL_BAUD) $(if $(CONFIG_ESPRESSIF_ESPTOOLPY_NO_STUB),--no-stub))
	$(eval WRITEFLASH_OPTS := $(if $(CONFIG_ESPRESSIF_MERGE_BINS),$(ESPTOOL_WRITEFLASH_OPTS) 0x0 nuttx.merged.bin,$(ESPTOOL_WRITEFLASH_OPTS) $(ESPTOOL_BINS)))
	esptool.py $(ESPTOOL_OPTS) write_flash $(WRITEFLASH_OPTS)
endef
