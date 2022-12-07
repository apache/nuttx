############################################################################
# boards/risc-v/esp32c3/esp32c3-devkit/kernel/Makefile
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

include $(TOPDIR)/Make.defs

# The entry point name (if none is provided in the .config file)

CONFIG_INIT_ENTRYPOINT ?= user_start
ENTRYPT = $(patsubst "%",%,$(CONFIG_INIT_ENTRYPOINT))

# Get the paths to the libraries and the links script path in format that
# is appropriate for the host OS

USER_LIBPATHS = $(addprefix -L,$(call CONVERT_PATH,$(addprefix $(TOPDIR)$(DELIM),$(dir $(USERLIBS)))))
USER_LDSCRIPT = $(call CONVERT_PATH,$(BOARD_COMMON_DIR)$(DELIM)scripts$(DELIM)protected_memory.ld)
USER_LDSCRIPT += $(call CONVERT_PATH,$(BOARD_COMMON_DIR)$(DELIM)scripts$(DELIM)user-space.ld)
USER_LDSCRIPT += $(call CONVERT_PATH,$(BOARD_COMMON_DIR)$(DELIM)scripts$(DELIM)esp32c3_rom.ld)
USER_HEXFILE += $(call CONVERT_PATH,$(TOPDIR)$(DELIM)nuttx_user.hex)
USER_BINFILE += $(call CONVERT_PATH,$(TOPDIR)$(DELIM)nuttx_user.bin)

USER_LDFLAGS = --undefined=$(ENTRYPT) --entry=$(ENTRYPT) $(addprefix -T,$(addsuffix .tmp,$(USER_LDSCRIPT))) -melf32lriscv

ifeq ($(CONFIG_DEBUG_LINK_MAP),y)
USER_LDFLAGS += --cref -Map="$(TOPDIR)$(DELIM)User.map"
endif

USER_LDLIBS = $(patsubst lib%,-l%,$(basename $(notdir $(USERLIBS))))
USER_LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"

# Source files

CSRCS = esp32c3_userspace.c
COBJS = $(CSRCS:.c=$(OBJEXT))
OBJS  = $(COBJS)

ifeq ($(LD),$(CC))
  LDSTARTGROUP ?= -Wl,--start-group
  LDENDGROUP   ?= -Wl,--end-group
  USER_LDFLAGS := $(addprefix -Xlinker ,$(USER_LDFLAGS))
  USER_LDFLAGS += $(CFLAGS)
else
  LDSTARTGROUP ?= --start-group
  LDENDGROUP   ?= --end-group
endif

# Targets:

all: $(TOPDIR)$(DELIM)nuttx_user.elf
.PHONY: nuttx_user.elf depend clean distclean

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(addsuffix .tmp,$(USER_LDSCRIPT)): $(USER_LDSCRIPT)
	$(call PREPROCESS,$(patsubst %.tmp,%,$@),$@)

# Create the nuttx_user.elf file containing all of the user-mode code

nuttx_user.elf: $(OBJS) $(addsuffix .tmp,$(USER_LDSCRIPT))
	$(Q) $(LD) -o $@ $(USER_LDFLAGS) $(USER_LIBPATHS) $(OBJS) $(LDSTARTGROUP) $(USER_LDLIBS) $(LDENDGROUP) $(USER_LIBGCC)

$(TOPDIR)$(DELIM)nuttx_user.elf: nuttx_user.elf
	$(Q) echo "LD: nuttx_user.elf"
	$(Q) cp -a nuttx_user.elf $(TOPDIR)$(DELIM)nuttx_user.elf
ifeq ($(CONFIG_INTELHEX_BINARY),y)
	$(Q) echo "CP: nuttx_user.hex"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O ihex nuttx_user.elf $(USER_HEXFILE)
endif
ifeq ($(CONFIG_RAW_BINARY),y)
	$(Q) echo "CP: nuttx_user.bin"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O binary nuttx_user.elf $(USER_BINFILE)
endif
	$(Q) $(call DELFILE,$(addsuffix .tmp,$(USER_LDSCRIPT)))

.depend:

depend: .depend

clean:
	$(call DELFILE, nuttx_user.elf)
	$(call DELFILE, "$(TOPDIR)$(DELIM)nuttx_user.*")
	$(call DELFILE, "$(TOPDIR)$(DELIM)User.map")
	$(call CLEAN)

distclean: clean
