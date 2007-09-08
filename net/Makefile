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
CFLAGS		+= -I./uip

MKDEP		= $(TOPDIR)/tools/mkdeps.sh

ifeq ($(CONFIG_NET),y)
STD_ASRCS	=
STD_CSRCS	= socket.c bind.c connect.c send.c sendto.c recv.c recvfrom.c \
		  net-sockets.c net-close.c

ifeq ($(CONFIG_NET_SOCKOPTS),y)
STD_CSRCS       += setsockopt.c getsockopt.c 
ifneq ($(CONFIG_DISABLE_CLOCK),y)
STD_CSRCS	+= net-timeout.c net-dsec2timeval.c net-timeval2dsec.c
endif
endif

include uip/Make.defs
endif

ASRCS		= $(STD_ASRCS) $(UIP_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(STD_CSRCS) $(UIP_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libnet$(LIBEXT)

VPATH		= uip

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

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_NET),y)
	$(MKDEP) --dep-path . --dep-path uip $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
	touch $@

depend: .depend

clean:
	rm -f $(BIN) *.o *.rel *.asm *.lst *.sym *.adb *~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
