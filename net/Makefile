############################################################################
# net/Makefile
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
include rpmsg/Make.defs
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
include nat/Make.defs
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
	$(call ARCHIVE, $@, $^)

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
ifeq ($(CONFIG_NET),y)
	$(Q) $(MAKE) makedepfile
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
