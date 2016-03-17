############################################################################
# net/Makefile
#
#   Copyright (C) 2007, 2008, 2011-2015 Gregory Nutt. All rights reserved.
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

ifeq ($(CONFIG_NET),y)

# Basic networking support

NET_ASRCS   =
NET_CSRCS   = net_initialize.c

# Socket support

SOCK_ASRCS  =
SOCK_CSRCS  =

# Support for operations on network devices

NETDEV_ASRCS =
NETDEV_CSRCS =

VPATH =
DEPPATH = --dep-path .

include socket/Make.defs
include netdev/Make.defs
include iob/Make.defs
include arp/Make.defs
include icmp/Make.defs
include icmpv6/Make.defs
include neighbor/Make.defs
include igmp/Make.defs
include pkt/Make.defs
include local/Make.defs
include tcp/Make.defs
include udp/Make.defs
include 6lowpan/Make.defs
include devif/Make.defs
include loopback/Make.defs
include route/Make.defs
include procfs/Make.defs
include utils/Make.defs
endif

ASRCS		= $(SOCK_ASRCS) $(NETDEV_ASRCS) $(NET_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(SOCK_CSRCS) $(NETDEV_CSRCS) $(NET_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libnet$(LIBEXT)

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_NET),y)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
