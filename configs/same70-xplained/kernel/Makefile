############################################################################
# configs/same70-xplainged/kernel/Makefile
#
#   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

# This is the directory for the board-specific header files

BOARD_INCLUDE = $(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)include

# The entry point name (if none is provided in the .config file)

CONFIG_USER_ENTRYPOINT ?= user_start
ENTRYPT = $(patsubst "%",%,$(CONFIG_USER_ENTRYPOINT))

# Get the paths to the libraries and the links script path in format that
# is appropriate for the host OS

ifeq ($(WINTOOL),y)
  # Windows-native toolchains
  USER_LIBPATHS = ${shell for path in $(USERLIBS); do dir=`dirname $(TOPDIR)$(DELIM)$$path`;echo "-L\"`cygpath -w $$dir`\"";done}
  USER_LDSCRIPT = -T "${shell cygpath -w $(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts$(DELIM)memory.ld}"
  USER_LDSCRIPT += -T "${shell cygpath -w $(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts$(DELIM)user-space.ld}"
  USER_HEXFILE += "${shell cygpath -w $(TOPDIR)$(DELIM)nuttx_user.hex}"
  USER_SRECFILE += "${shell cygpath -w $(TOPDIR)$(DELIM)nuttx_user.srec}"
  USER_BINFILE += "${shell cygpath -w $(TOPDIR)$(DELIM)nuttx_user.bin}"
else
  # Linux/Cygwin-native toolchain
  USER_LIBPATHS = $(addprefix -L$(TOPDIR)$(DELIM),$(dir $(USERLIBS)))
  USER_LDSCRIPT = -T$(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts$(DELIM)memory.ld
  USER_LDSCRIPT += -T$(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts$(DELIM)user-space.ld
  USER_HEXFILE += "$(TOPDIR)$(DELIM)nuttx_user.hex"
  USER_SRECFILE += "$(TOPDIR)$(DELIM)nuttx_user.srec"
  USER_BINFILE += "$(TOPDIR)$(DELIM)nuttx_user.bin"
endif

USER_LDFLAGS = --undefined=$(ENTRYPT) --entry=$(ENTRYPT) $(USER_LDSCRIPT)
USER_LDLIBS = $(patsubst lib%,-l%,$(basename $(notdir $(USERLIBS))))
USER_LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"

# Source files

CSRCS = sam_userspace.c
COBJS = $(CSRCS:.c=$(OBJEXT))
OBJS  = $(COBJS)

# Targets:

all: $(TOPDIR)$(DELIM)nuttx_user.elf $(TOPDIR)$(DELIM)User.map
.PHONY: nuttx_user.elf depend clean distclean

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# Create the nuttx_user.elf file containing all of the user-mode code

nuttx_user.elf: $(OBJS)
	$(Q) $(LD) -o $@ $(USER_LDFLAGS) $(USER_LIBPATHS) $(OBJS) --start-group $(USER_LDLIBS) --end-group $(USER_LIBGCC)

$(TOPDIR)$(DELIM)nuttx_user.elf: nuttx_user.elf
	@echo "LD: nuttx_user.elf"
	$(Q) cp -a nuttx_user.elf $(TOPDIR)$(DELIM)nuttx_user.elf
ifeq ($(CONFIG_INTELHEX_BINARY),y)
	@echo "CP: nuttx_user.hex"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O ihex nuttx_user.elf $(USER_HEXFILE)
endif
ifeq ($(CONFIG_MOTOROLA_SREC),y)
	@echo "CP: nuttx_user.srec"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O srec nuttx_user.elf $(USER_SRECFILE)
endif
ifeq ($(CONFIG_RAW_BINARY),y)
	@echo "CP: nuttx_user.bin"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O binary nuttx_user.elf $(USER_BINFILE)
endif

$(TOPDIR)$(DELIM)User.map: nuttx_user.elf
	@echo "MK: User.map"
	$(Q) $(NM) nuttx_user.elf >$(TOPDIR)$(DELIM)User.map
	$(Q) $(CROSSDEV)size nuttx_user.elf

.depend:

depend: .depend

clean:
	$(call DELFILE, nuttx_user.elf)
	$(call DELFILE, "$(TOPDIR)$(DELIM)nuttx_user.*")
	$(call DELFILE, "$(TOPDIR)$(DELIM)User.map")
	$(call CLEAN)

distclean: clean
