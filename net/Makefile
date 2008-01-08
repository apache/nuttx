############################################################################
# net/Makefile
#
#   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

ifeq ($(CONFIG_NET),y)
SOCK_ASRCS	=
SOCK_CSRCS	= socket.c bind.c connect.c sendto.c recv.c recvfrom.c \
		  net-sockets.c net-close.c

ifeq ($(CONFIG_NET_TCP),y)
SOCK_CSRCS	+= send.c listen.c accept.c 
endif

ifeq ($(CONFIG_NET_SOCKOPTS),y)
SOCK_CSRCS      += setsockopt.c getsockopt.c 
ifneq ($(CONFIG_DISABLE_CLOCK),y)
SOCK_CSRCS	+= net-timeo.c net-dsec2timeval.c net-timeval2dsec.c net-arptimer.c
endif
endif

NETDEV_ASRCS	=
NETDEV_CSRCS	= netdev-register.c netdev-ioctl.c netdev-txnotify.c \
		  netdev-findbyname.c netdev-findbyaddr.c netdev-count.c \
		  netdev-foreach.c

include uip/Make.defs
endif

ASRCS		= $(SOCK_ASRCS) $(NETDEV_ASRCS) $(UIP_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(SOCK_CSRCS) $(NETDEV_CSRCS) $(UIP_CSRCS)
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
	rm -f uip/*~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
