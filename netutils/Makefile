############################################################################
# Makefile
#
#   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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

MKDEP		= $(TOPDIR)/tools/mkdeps.sh

ifeq ($(CONFIG_NET),y)
include uiplib/Make.defs
include dhcpc/Make.defs
include resolv/Make.defs
include smtp/Make.defs
include telnetd/Make.defs
include webclient/Make.defs
include webserver/Make.defs
include Make.str
endif

ASRCS		= $(UIPLIB_ASRCS) $(DHCPC_ASRCS) $(RESOLV_ASRCS) $(SMTP_ASRCS) \
		  $(TELNETD_ASRCS) $(WEBCLIENT_ASRCS) $(WEBSERVER_ASRCS) $(STRNG_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(UIPLIB_CSRCS) $(DHCPC_CSRCS) $(RESOLV_CSRCS) $(SMTP_CSRCS) \
		  $(TELNETD_CSRCS) $(WEBCLIENT_CSRCS) $(WEBSERVER_CSRCS) $(STRNG_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libnetutils$(LIBEXT)

VPATH		= uiplib:dhcpc:resolv:smtp:telnetd:webclient:webserver

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(CC) -c $(CFLAGS) -D__ASSEMBLY__ $< -o $@

$(COBJS): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BIN): $(OBJS)
	( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $obj FAILED!" ; exit 1 ; } ; \
	done ; )

makestrings$(EXEEXT): uiplib/makestrings.c
	gcc -O2 -Wall -o makestrings$(EXEEXT) uiplib/makestrings.c

.strings: makestrings$(EXEEXT) uiplib/netutil-strings
	./makestrings uiplib/netutil-strings

Make.str: makestrings$(EXEEXT) uiplib/netutil-strings
	./makestrings -s uiplib/netutil-strings >Make.str

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_NET),y)
	$(MKDEP) --dep-path . --dep-path uiplib --dep-path dhcpc --dep-path smtp --dep-path webclient \
		 --dep-path resolv --dep-path telnetd --dep-path webserver \
	$(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
	touch $@

depend: .strings Make.str .depend

clean:
	rm -f $(BIN) *.o *.rel *.asm *.lst *.sym *.adb *~
	rm -f uiplib/*~ dhcpc/*~ resolv/*~ smtp/*~ telnetd/*~ webclient/*~ webserver/*~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	rm -f Make.dep .depend
	rm -f $(STRNG_CSRCS) $(STRNG_ASRCS)
	rm -f Make.str netutil-strings.h makestrings$(EXEEXT)

-include Make.dep
