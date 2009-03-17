############################################################################
# netutils/Makefile
#
#   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

ifeq ($(CONFIG_NET),y)
include uiplib/Make.defs
ifeq ($(CONFIG_NET_TCP),y)
include smtp/Make.defs
include telnetd/Make.defs
include webclient/Make.defs
include webserver/Make.defs
endif
ifeq ($(CONFIG_NET_UDP),y)
include dhcpc/Make.defs
include dhcpd/Make.defs
include resolv/Make.defs
ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
include tftpc/Make.defs
endif
endif
endif

SUBDIRS		= uiplib dhcpc dhcpd resolv smtp telnetd webclient webserver tftpc

ASRCS		= $(UIPLIB_ASRCS) $(DHCPC_ASRCS) $(DHCPD_ASRCS) $(RESOLV_ASRCS) \
		  $(SMTP_ASRCS) $(TELNETD_ASRCS) $(WEBCLIENT_ASRCS) $(WEBSERVER_ASRCS) \
		  $(TFTPC_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(UIPLIB_CSRCS) $(DHCPC_CSRCS) $(DHCPD_CSRCS) $(RESOLV_CSRCS) \
		  $(SMTP_CSRCS) $(TELNETD_CSRCS) $(WEBCLIENT_CSRCS) $(WEBSERVER_CSRCS) \
		  $(TFTPC_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libnetutils$(LIBEXT)

VPATH		= uiplib:dhcpc:dhcpd:resolv:smtp:telnetd:webclient:webserver:tftpc

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_NET),y)
	@$(MKDEP) --dep-path . --dep-path uiplib --dep-path dhcpc --dep-path dhcpd \
		  --dep-path smtp --dep-path webclient --dep-path resolv \
		  --dep-path telnetd --dep-path webserver --dep-path tftpc \
	$(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
	@touch $@

depend: .depend

clean:
	@rm -f $(BIN) *~ .*.swp
	$(call CLEAN)
	@( for dir in $(SUBDIRS); do \
		rm -f $${dir}/*~ $${dir}/.*.swp; \
	done ; )

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
