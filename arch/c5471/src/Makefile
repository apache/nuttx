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

MKDEP 		= $(TOPDIR)/tools/mkdeps.sh
CFLAGS		+= -I$(TOPDIR)/sched

ASRCS		= up_vectors.S up_saveusercontext.S up_fullcontextrestore.S
ifeq ($(CONFIG_DEBUG),y)
ASRCS		+= up_lowputc.S
endif
AOBJS		= $(ASRCS:.S=.o)

CSRCS		= up_initialize.c up_initialstate.c up_idle.c up_doirq.c \
		  up_irq.c up_syscall.c up_dataabort.c up_prefetchabort.c \
		  up_undefinedinsn.c up_interruptcontext.c up_timerisr.c \
		  up_createstack.c up_usestack.c up_releasestack.c \
		  up_exit.c up_assert.c up_blocktask.c up_unblocktask.c \
		  up_releasepending.c up_reprioritizertr.c up_copystate.c \
		  up_schedulesigaction.c up_sigdeliver.c up_serial.c \
		  up_delay.c up_allocateheap.c up_watchdog.c
COBJS		= $(CSRCS:.c=.o)

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

LDFLAGS		= $(ARCHSCRIPT)
EXTRA_LIBS	=

LINKOBJS	= up_head$(OBJEXT)
LINKLIBS	=
LDPATHES	= $(addprefix -L$(TOPDIR)/,$(dir $(LINKLIBS)))
LDLIBS		= $(patsubst lib%,-l%,$(basename $(notdir $(LINKLIBS))))

BOARDDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src/board

LIBGCC		= ${shell $(CC) -print-libgcc-file-name}

all: up_head.o libarch$(LIBEXT)

$(AOBJS) $(LINKOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) -D__ASSEMBLY__ $< -o $@

$(COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

libarch$(LIBEXT): $(OBJS)
	( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $obj FAILED!" ; exit 1 ; } ; \
	done ; )

board/libboard$(LIBEXT):
	$(MAKE) -C board TOPDIR=$(TOPDIR) libboard$(LIBEXT)

nuttx: $(LINKOBJS) board/libboard$(LIBEXT)
	$(LD) --entry=__start $(LDFLAGS) $(LDPATHES) -L$(BOARDDIR) -o $(TOPDIR)/$@ $(LINKOBJS) \
		--start-group $(LDLIBS) -lboard --end-group $(EXTRA_LIBS) $(LIBGCC) 
	@$(NM) $(TOPDIR)/$@ | \
	grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map
	@export vflashstart=`$(OBJDUMP) --all-headers $(TOPDIR)/$@ | grep _vflashstart | cut -d' ' -f1`;  \
	if [ ! -z "$$vflashstart" ]; then \
		$(OBJCOPY) --adjust-section-vma=.vector=0x$$vflashstart $(TOPDIR)/$@ $(TOPDIR)/$@.flashimage; \
		mv $(TOPDIR)/$@.flashimage $(TOPDIR)/$@; \
	fi
ifeq ($(CONFIG_RRLOAD_BINARY),y)
	@$(TOPDIR)/tools/mkimage.sh $(TOPDIR)/$@ $(TOPDIR)/$@.rr
	@if [ -w /tftpboot ] ; then \
		cp -f $(TOPDIR)/$@.rr /tftpboot/$@.rr.${CONFIG_ARCH}; \
	fi
endif

.depend: Makefile $(SRCS)
	echo $(SRCS)
	$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(MAKE) -C board TOPDIR=$(TOPDIR) depend
	touch $@

depend: .depend

clean:
	rm -f libarch$(LIBEXT) *.o *~
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR=$(TOPDIR) clean ; \
	fi

distclean: clean
	rm -f Make.dep .depend
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR=$(TOPDIR) distclean ; \
	fi

-include Make.dep
