############################################################################
# arch/risc-v/src/common/espressif/esp_ulp.mk
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

ifeq ($(CONFIG_ESPRESSIF_USE_LP_CORE),y)

# Include header paths

ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)ulp
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_common$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)port$(DELIM)$(CHIP_SERIES)$(DELIM)private_include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_rom$(DELIM)$(CHIP_SERIES)$(DELIM)include$(DELIM)$(CHIP_SERIES)
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)platform_port$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)log$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)riscv$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)$(CHIP_SERIES)$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)register
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)register
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)register$(DELIM)soc
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)ulp_common
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)ulp_common$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)$(CHIP_SERIES)$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)src$(DELIM)components$(DELIM)esp_driver_gpio$(DELIM)include
ULP_INCLUDES += $(INCDIR_PREFIX)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)src$(DELIM)components$(DELIM)esp_driver_uart$(DELIM)include

# Linker scripts

ULP_LDINCLUDES += -Tchip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)ld$(DELIM)$(CHIP_SERIES).peripherals.ld
ULP_LDINCLUDES += -Tchip$(DELIM)ulp$(DELIM)ulp_sections.ld

# Source files

ULP_ASOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)port$(DELIM)${CHIP_SERIES}$(DELIM)vector_table.S
ULP_ASOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)start.S
ULP_ASOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)vector.S

ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_spi.c
# ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)src$(DELIM)components$(DELIM)esp_driver_uart$(DELIM)src$(DELIM)uart_wakeup.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)ulp_lp_core_memory_shared.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)ulp_lp_core_lp_uart_shared.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)ulp_lp_core_lp_timer_shared.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)uart_hal_iram.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)hal$(DELIM)uart_hal.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_i2c.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_startup.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_utils.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_uart.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_print.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_panic.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_interrupt.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)lp_core$(DELIM)lp_core_ubsan.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)ulp_lp_core_lp_adc_shared.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)ulp_lp_core_lp_vad_shared.c
ULP_CSOURCES += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)lp_core$(DELIM)shared$(DELIM)ulp_lp_core_critical_section_shared.c

# Add ULP app source files and directories

ULP_FOLDER = chip$(DELIM)ulp
ULP_CODE_HEADER = $(ULP_FOLDER)$(DELIM)ulp_code.h
ULP_PROJECT_PATH = $(shell echo $(CONFIG_ESPRESSIF_ULP_PROJECT_PATH))

ifeq ($(suffix $(ULP_PROJECT_PATH)),.bin)
	ULP_BIN_FILE_PATH = $(TOPDIR)$(DELIM)$(CONFIG_ESPRESSIF_ULP_PROJECT_PATH)
	ULP_BIN_FILE = "skip"
else
	ARCHSCRIPT += $(ULP_FOLDER)$(DELIM)ulp_main.ld

	ULP_BIN_FILE = $(ULP_FOLDER)$(DELIM)ulp.bin
	ULP_BIN_FILE_PATH = $(ULP_BIN_FILE)

	ULP_APP_DIR = $(shell echo $(CONFIG_ESPRESSIF_ULP_PROJECT_PATH))
	ULP_APP_PATH = $(TOPDIR)$(DELIM)$(ULP_APP_DIR)

	ULP_APP_C_SRCS = $(wildcard $(ULP_APP_PATH)/*.c $(ULP_APP_PATH)/**/*.c)
	ULP_APP_ASM_SRCS = $(wildcard $(ULP_APP_PATH)/*.s $(ULP_APP_PATH)/**/*.S)
	ULP_APP_HEADERS = $(wildcard $(ULP_APP_PATH)/**/*.h $(ULP_APP_PATH)/**/*.h)
	ULP_APP_INCLUDES = $(addprefix -I,$(sort $(dir $(ULP_APP_HEADERS))))
	ULP_APP_OBJS = $(ULP_APP_ASM_SRCS:.S=_ulp.o)
	ULP_APP_OBJS += $(ULP_APP_C_SRCS:.c=_ulp.o)

	ULP_INCLUDES += $(ULP_APP_INCLUDES)
	ULP_CSOURCES += $(ULP_APP_C_SRCS)
	ULP_ASOURCES += $(ULP_APP_ASM_SRCS)

	# Object file format for ULP will be FILE_NAME_ulp.o to distinguish it from HP core files

	ULP_COBJS = $(ULP_CSOURCES:.c=_ulp.o)
	ULP_AOBJS = $(ULP_ASOURCES:.S=_ulp.o)
	ULP_OBJS = $(ULP_COBJS) $(ULP_AOBJS)

	ULP_ELF_FILE = $(ULP_FOLDER)$(DELIM)ulp.elf
	ULP_MAP_FILE = $(ULP_FOLDER)$(DELIM)ulp.map
	ULP_SYM_FILE = $(ULP_FOLDER)$(DELIM)ulp.sym

endif

# Toolchain and output paths definitions

ULP_AS = $(CC)

ULP_READELF = $(CROSSDEV)readelf
ULP_MAPGEN_TOOL_PATH = chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)ulp$(DELIM)esp32ulp_mapgen.py
ULP_PREFIX = ulp_
ULP_BASE = 0

# To prevent redefining error of other header files in nuttx folder, nuttx/config.h file
# will be moved during ULP compilation. This step will only effect ULP

ULP_NUTTX_CONFIG = chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)config.h

# Compiler and linker flags

ULP_CFLAGS := 										\
	-Os 														\
	-ggdb 													\
	-march=rv32imac_zicsr_zifencei 	\
	-mdiv 													\
	-fdata-sections 								\
	-ffunction-sections 						\
	-DIS_ULP_COCPU									\
	$(ULP_INCLUDES)

ULP_ASFLAGS :=										\
	-Os															\
	-ggdb														\
	-march=rv32imac_zicsr_zifencei	\
	-x assembler-with-cpp 					\
	-D__ASSEMBLER__									\
	-DIS_ULP_COCPU									\
	$(ULP_INCLUDES)

ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
ULP_CFLAGS += -O0
ULP_ASFLAGS += -O0

ifeq ($(CONFIG_ESPRESSIF_ULP_ENABLE_UBSAN),y)
ULP_CFLAGS +=								\
	-fno-sanitize=shift-base	\
	-fsanitize=undefined

ULP_ASFLAGS +=							\
	-fno-sanitize=shift-base	\
	-fsanitize=undefined
endif
endif

ULP_LDFLAGS :=										\
	-march=rv32imac_zicsr_zifencei	\
	--specs=nano.specs							\
	--specs=nosys.specs 						\
	-nostartfiles										\
	-Wl,--no-warn-rwx-segments			\
	-Wl,--gc-sections								\
	-Xlinker -Map=$(ULP_MAP_FILE)		\
	$(ULP_LDINCLUDES)

# Build rules

.PHONY: context depend

checkpython3:
	$(Q) if [ -z "$$(which python3)" ]; then \
		$(Q) echo "ERROR: python3 not found in PATH"; \
		$(Q) echo "       Please install python3 or fix the PATH"; \
		exit 1; \
	fi

%_ulp.o: %.c $(ULP_NUTTX_CONFIG)
	$(Q) echo "Compiling $< for ULP"
	$(Q) $(CC) $(ULP_CFLAGS) -c $< -o $@
	$(Q) $(CC) $(ULP_INCLUDES) -E -P -xc -o $(ULP_FOLDER)$(DELIM)ulp_sections.ld board$(DELIM)scripts$(DELIM)${CHIP_SERIES}_lpcore_sections.ld

%_ulp.o: %.S $(ULP_NUTTX_CONFIG)
	$(Q) echo "Compiling $< for ULP"
	$(Q) $(CC) $(ULP_ASFLAGS) -c $< -o $@

$(ULP_NUTTX_CONFIG): $(ULP_FOLDER)
	$(Q) echo "Copying nuttx$(DELIM)config.h into $(ULP_FOLDER)$(DELIM)nuttx"
	$(Q) cp $(TOPDIR)$(DELIM)include$(DELIM)nuttx$(DELIM)config.h $(ULP_FOLDER)$(DELIM)nuttx

$(ULP_ELF_FILE): $(ULP_OBJS)
	$(Q) echo "Linking for ULP"
	$(Q) $(CC) $(ULP_LDFLAGS) $(ULP_OBJS) -o $@

$(ULP_BIN_FILE): $(ULP_ELF_FILE) checkpython3
ifneq ($(suffix $(ULP_PROJECT_PATH)),.bin)
	$(Q) echo "Creating bin for ULP"
	$(Q) $(OBJCOPY) -O binary $(ULP_ELF_FILE) $(ULP_BIN_FILE)
	$(Q) $(ULP_READELF) -sW $(ULP_ELF_FILE) > $(ULP_SYM_FILE)
	$(Q) python3 $(ULP_MAPGEN_TOOL_PATH) -s $(ULP_SYM_FILE) -o $(ULP_FOLDER)$(DELIM)ulp_main --base $(ULP_BASE) --prefix $(ULP_PREFIX)
endif
	$(Q) echo "Converting bin for ULP into header file"
	$(Q) xxd -i $(ULP_BIN_FILE_PATH) >$(ULP_CODE_HEADER) || { echo "xxd of $< failed" ; exit 1 ; }
	$(Q) sed -i 's/unsigned char[^[]*\[[^]]*\]/unsigned char esp_ulp_bin[]/g' $(ULP_CODE_HEADER)
	$(Q) sed -i 's/unsigned int[^=]* =/unsigned int esp_ulp_bin_len =/g' $(ULP_CODE_HEADER)

$(ULP_FOLDER):
	$(Q) echo "Creating $(ULP_FOLDER) folder"
	$(Q) mkdir $(ULP_FOLDER)
	$(Q) mkdir $(ULP_FOLDER)$(DELIM)nuttx

context:: $(ULP_FOLDER)
	$(Q) touch $(ULP_CODE_HEADER)
	$(Q) touch $(ULP_FOLDER)$(DELIM)ulp_main.h

depend: $(ULP_BIN_FILE)

distclean::
ifneq ($(suffix $(ULP_PROJECT_PATH)),.bin)
	$(Q) rm $(ULP_APP_OBJS) || true
endif

endif # CONFIG_ESPRESSIF_USE_LP_CORE
