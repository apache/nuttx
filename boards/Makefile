############################################################################
# boards/Makefile
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

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds) $(CXXSRCS:.cxx=.ddx)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
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
