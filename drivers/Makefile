############################################################################
# drivers/Makefile
#
#   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif

ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
include serial/Make.defs
ROOTDEPPATH = --dep-path .
SERIALDEPPATH = --dep-path serial
endif

ifeq ($(CONFIG_NET),y)
include net/Make.defs
ROOTDEPPATH = --dep-path .
NETDEPPATH = --dep-path net
endif

ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
include pipes/Make.defs
ROOTDEPPATH = --dep-path .
PIPEDEPPATH = --dep-path pipes
endif

ifeq ($(CONFIG_USBDEV),y)
include usbdev/Make.defs
ROOTDEPPATH = --dep-path .
USBDEVDEPPATH = --dep-path usbdev
CFLAGS += ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/drivers/usbdev}
endif

ifeq ($(CONFIG_USBHOST),y)
include usbhost/Make.defs
ROOTDEPPATH = --dep-path .
USBHOSTDEPPATH = --dep-path usbhost
CFLAGS += ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/drivers/usbhost}
endif

include mmcsd/Make.defs
ROOTDEPPATH = --dep-path .
MMCSDDEPPATH = --dep-path mmcsd
CFLAGS += ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/drivers/mmcsd}

include lcd/Make.defs
ROOTDEPPATH = --dep-path .
LCDDEPPATH = --dep-path lcd
CFLAGS += ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/drivers/lcd}

ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
include bch/Make.defs
ROOTDEPPATH = --dep-path .
BCHDEPPATH = --dep-path bch
CFLAGS += ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/drivers/bch}
endif
endif

include mtd/Make.defs
ROOTDEPPATH = --dep-path .
MTDDEPPATH = --dep-path mtd

include sensors/Make.defs
ROOTDEPPATH = --dep-path .
SENSORDEPPATH = --dep-path sensors
CFLAGS += ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/drivers/sensors}

ASRCS		= $(SERIAL_ASRCS) $(NET_ASRCS) $(PIPE_ASRCS) $(USBDEV_ASRCS) \
			  $(USBHOST_ASRCS) $(MMCSD_ASRCS) $(LCD_ASRCS) $(BCH_ASRCS) \
			  $(MTD_ASRCS) $(SENSOR_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		=
ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
CSRCS		+= dev_null.c dev_zero.c loop.c  can.c
ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
CSRCS		+= ramdisk.c rwbuffer.c
endif
endif
CSRCS		+= $(SERIAL_CSRCS) $(NET_CSRCS) $(PIPE_CSRCS) $(USBDEV_CSRCS) \
			   $(USBHOST_CSRCS) $(MMCSD_CSRCS) $(LCD_CSRCS) $(BCH_CSRCS) \
			   $(MTD_CSRCS) $(SENSOR_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN			= libdrivers$(LIBEXT)

VPATH		= serial:net:pipes:usbdev:usbhost:mmcsd:lcd:bch:mtd:sensors

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) $(SERIALDEPPATH) $(NETDEPPATH) $(PIPEDEPPATH) \
	  $(USBDEVDEPPATH) $(USBHOSTDEPPATH) $(MMCSDDEPPATH) $(LCDDEPPATH) \
	  $(BCHDEPPATH) $(MTDDEPPATH) $(SENSORDEPPATH) \
	  $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f $(BIN) *~ .*.swp
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
