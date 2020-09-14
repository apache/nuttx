############################################################################
# boards/Makefile
#
#   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

include $(TOPDIR)/Make.defs

# Determine if there is a Kconfig file for any custom board configuration

ifeq ($(CONFIG_ARCH_BOARD_CUSTOM),y)
  CUSTOM_DIR = $(patsubst "%",%,$(CONFIG_ARCH_BOARD_CUSTOM_DIR))
  ifeq ($(CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH),y)
    CUSTOM_KPATH = $(TOPDIR)$(DELIM)$(CUSTOM_DIR)$(DELIM)Kconfig
  else
    CUSTOM_KPATH = $(CUSTOM_DIR)$(DELIM)Kconfig
  endif
  CUSTOM_KCONFIG = $(if $(wildcard $(CUSTOM_KPATH)),y,)
endif

ifeq ($(CUSTOM_KCONFIG),y)
  BOARD_KCONFIG = $(CUSTOM_KPATH)
else
  BOARD_KCONFIG = $(TOPDIR)$(DELIM)boards$(DELIM)dummy$(DELIM)dummy_kconfig
endif

DUMMY_KCONFIG = $(TOPDIR)$(DELIM)boards$(DELIM)dummy$(DELIM)Kconfig

# The board configuration should be installed in the arch/ directory

BOARDDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)board
BOARD_INSTALLED = $(if $(wildcard $(BOARDDIR)$(DELIM)Makefile),y,)

# Basic

CONFIG_CSRCS = dummy.c

# boardctl support

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CONFIG_CSRCS += boardctl.c
endif

ASRCS = $(CONFIG_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CONFIG_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

CXXSRCS = $(CONFIG_CXXSRCS)
CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS)

BIN = libboards$(LIBEXT)

all: $(BIN)
.PHONY: depend context clean_context clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
ifneq ($(SRCS),)
	$(Q) $(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
ifneq ($(CXXSRCS),)
	$(Q) $(MKDEP) "$(CXX)" -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
endif
	$(Q) touch $@

depend: .depend

$(DUMMY_KCONFIG): $(BOARD_KCONFIG)
	$(call DELFILE, $(DUMMY_KCONFIG))
	$(call COPYFILE, $(BOARD_KCONFIG), $(DUMMY_KCONFIG))

dirlinks: $(DUMMY_KCONFIG)

context: $(DUMMY_KCONFIG)
ifeq ($(BOARD_INSTALLED),y)
	$(Q) $(MAKE) -C $(BOARDDIR) context
endif

clean_context:
	$(call DELFILE, $(DUMMY_KCONFIG))

clean: clean_context
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
