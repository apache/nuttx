#############################################################################
# netutils/thttpd/Makefile
#
#   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#############################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs

CGIBINDIR	= $(TOPDIR)/netutils/thttpd/cgi-bin

SUBDIRS		=
ifeq ($(CONFIG_NXFLAT),y)
SUBDIRS		= cgi-src
SUBDIR_BIN1	= phf
SUBDIR_BIN2	= redirect
SUBDIR_BIN3	= ssi
SUBDIR_BIN	+= cgi-bin/$(SUBDIR_BIN1) cgi-bin/$(SUBDIR_BIN2)  cgi-bin/$(SUBDIR_BIN3)
endif

ASRCS		=
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= thttpd.c libhttpd.c timers.c fdwatch.c tdate_parse.c
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libthttpd$(LIBEXT)

all:	$(BIN) $(SUBDIR_BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

ifeq ($(CONFIG_NXFLAT),y)
cgi-bin:
	@mkdir cgi-bin

cgi-src/$(SUBDIR_BIN1):
	@$(MAKE) -C cgi-src $(SUBDIR_BIN1)

cgi-bin/$(SUBDIR_BIN1): cgi-bin cgi-src/$(SUBDIR_BIN1)
	@cp -a cgi-src/$(SUBDIR_BIN1) $@

cgi-src/$(SUBDIR_BIN2):
	@$(MAKE) -C cgi-src $(SUBDIR_BIN2)

cgi-bin/$(SUBDIR_BIN2): cgi-bin cgi-src/$(SUBDIR_BIN2)
	@cp -a cgi-src/$(SUBDIR_BIN2) $@

cgi-src/$(SUBDIR_BIN3):
	@$(MAKE) -C cgi-src $(SUBDIR_BIN3)

cgi-bin/$(SUBDIR_BIN3): cgi-bin cgi-src/$(SUBDIR_BIN3)
	@cp -a cgi-src/$(SUBDIR_BIN3) $@
endif

.depend: Makefile $(SRCS)
	@$(MKDEP) --dep-path .  $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f $(BIN) *~ .*.swp
	@rm -rf cgi-bin
	$(call CLEAN)
	@( for dir in $(SUBDIRS); do \
		rm -f $${dir}/*~ $${dir}/.*.swp; \
	done ; )

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep


