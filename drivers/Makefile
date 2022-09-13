############################################################################
# drivers/Makefile
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

CSRCS = drivers_initialize.c

# Include support for various drivers.  Each Make.defs file will add its
# files to the source file list, add its DEPPATH info, and will add
# the appropriate paths to the VPATH variable

include analog/Make.defs
include audio/Make.defs
include bch/Make.defs
include can/Make.defs
include clk/Make.defs
include crypto/Make.defs
include math/Make.defs
include motor/Make.defs
include i2c/Make.defs
include i2s/Make.defs
include ipcc/Make.defs
include input/Make.defs
include ioexpander/Make.defs
include lcd/Make.defs
include leds/Make.defs
include loop/Make.defs
include misc/Make.defs
include mmcsd/Make.defs
include modem/Make.defs
include mtd/Make.defs
include eeprom/Make.defs
include efuse/Make.defs
include net/Make.defs
include note/Make.defs
include pipes/Make.defs
include power/Make.defs
include rptun/Make.defs
include sensors/Make.defs
include serial/Make.defs
include spi/Make.defs
include syslog/Make.defs
include timers/Make.defs
include usbdev/Make.defs
include usbhost/Make.defs
include usbmisc/Make.defs
include usbmonitor/Make.defs
include video/Make.defs
include wireless/Make.defs
include contactless/Make.defs
include 1wire/Make.defs
include rf/Make.defs
include rc/Make.defs
include segger/Make.defs
include usrsock/Make.defs

ifeq ($(CONFIG_SPECIFIC_DRIVERS),y)
-include platform/Make.defs
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libdrivers$(LIBEXT)

all: $(BIN)
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

context::

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
