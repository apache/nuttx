############################################################
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
# 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
############################################################

-include $(TOPDIR)/Make.defs

MKDEP 	= $(TOPDIR)/tools/mkdeps.sh
CFLAGS	+= -I$(TOPDIR)/sched

ASRCS	= up_vectors.S up_saveusercontext.S up_fullcontextrestore.S
ifeq ($(CONFIG_DEBUG),y)
ASRCS	+= up_lowputc.S
endif
AOBJS	= $(ASRCS:.S=.o)

CSRCS	= up_initialize.c up_initialstate.c up_idle.c \
	  up_irq.c up_syscall.c up_dataabort.c up_prefetchabort.c \
	  up_undefinedinsn.c up_interruptcontext.c up_timerisr.c \
	  up_createstack.c up_usestack.c up_releasestack.c \
	  up_exit.c up_assert.c up_blocktask.c up_unblocktask.c \
	  up_releasepending.c up_reprioritizertr.c up_copystate.c \
	  up_schedulesigaction.c up_sigdeliver.c up_serial.c
COBJS	= $(CSRCS:.c=.o)

SRCS	= $(ASRCS) $(CSRCS)
OBJS	= $(AOBJS) $(COBJS)

all: up_head.o libarch.a

$(AOBJS) up_head.o: %.o: %.S
	$(CC) -c $(CFLAGS) -D__ASSEMBLY__ $< -o $@

$(COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

libarch.a: $(OBJS)
	$(AR) rcs $@ $(OBJS)

.depend: Makefile $(SRCS)
	$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	touch $@

depend: .depend

clean:
	rm -f libarch.a *.o *~

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
