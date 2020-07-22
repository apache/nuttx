############################################################################
# net/Makefile
#
#   Copyright (C) 2007, 2008, 2011-2018 Gregory Nutt. All rights reserved.
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

ifeq ($(CONFIG_NET),y)

# Basic networking support

NET_CSRCS = net_initialize.c

include socket/Make.defs
include inet/Make.defs
include netdev/Make.defs
include arp/Make.defs
include icmp/Make.defs
include icmpv6/Make.defs
include neighbor/Make.defs
include igmp/Make.defs
include pkt/Make.defs
include local/Make.defs
include mld/Make.defs
include can/Make.defs
include netlink/Make.defs
include tcp/Make.defs
include udp/Make.defs
include sixlowpan/Make.defs
include bluetooth/Make.defs
include ieee802154/Make.defs
include devif/Make.defs
include ipforward/Make.defs
include route/Make.defs
include procfs/Make.defs
include usrsock/Make.defs
include utils/Make.defs
endif

CFLAGS += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)net}

ASRCS = $(SOCK_ASRCS) $(NETDEV_ASRCS) $(NET_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(SOCK_CSRCS) $(NETDEV_CSRCS) $(NET_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libnet$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
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
