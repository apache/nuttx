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
ASFLAGS		= -x -j -g -l -s -p
CPPFLAGS	= $(ARCHINCLUDES) $(ARCHDEFINES) -D__ASSEMBLY__

SSRCS		= 
ASRCS		= $(SSRCS:.S=$(ASMEXT))
AOBJS		= $(ASRCS:$(ASMEXT)=$(OBJEXT))
CSRCS		= up_initialize.c up_idle.c up_interruptcontext.c \
		  up_initialstate.c up_unblocktask.c up_blocktask.c \
		  up_releasepending.c up_reprioritizertr.c \
		  up_exit.c up_assert.c up_allocateheap.c \
		  up_irq.c up_savecontext.c up_restorecontext.c \
		  up_timerisr.c up_putc.c
COBJS		= $(CSRCS:.c=$(OBJEXT))
SRCS		= $(SSRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

SDCCLIBDIR	= /usr/local/share/sdcc/lib/large
SDCCPATH	= -L$(SDCCLIBDIR)
SDCCLIBS	= -llibfloat.lib -llibint.lib -lliblong.lib -llibmysdcc.lib -lmcs51.lib

LINKSSRCS	= up_head.S
LINKASRCS	= $(LINKSSRCS:.S=$(ASMEXT))
LINKOBJS	= $(LINKASRCS:$(ASMEXT)=$(OBJEXT))
LINKLIBS	=
LDPATHES	= $(addprefix -L$(TOPDIR)/,$(dir $(LINKLIBS)))
LDLIBS		= $(addprefix -l,$(notdir $(LINKLIBS)))

LDFLAGS		= --model-large --nostdlib \
		  --data-loc 0x30 --iram-size 0x100 \
		  --code-loc 0x20c0 --code-size 0x5f40 \
		  --xram-loc 0x0100 --xram-size 0x1f00

DEPSRCS		= $(SRCS) $(LINKSSRCS)

HEAP1_BASE	= ${shell cat pass1.mem | grep "EXTERNAL RAM" | sed -e "s/[ ][ ]*/ /g" | cut -d' ' -f5}
HEAP2_BASE	= ${shell cat pass1.mem | grep "ROM/EPROM/FLASH" | sed -e "s/[ ][ ]*/ /g" | cut -d' ' -f4}
STACK_BASE	= ${shell cat pass1.mem | grep "Stack starts" | cut -d' ' -f4}

all: up_head$(OBJEXT) libarch$(LIBEXT)

$(ASRCS) $(LINKASRCS): %$(ASMEXT): %.S
	$(CPP) -P $(CPPFLAGS) -D__ASSEMBLY__ $< -o $@

$(AOBJS) $(LINKOBJS): $(ASRCS) $(LINKASRCS)
	$(AS) $(ASFLAGS) $<

$(COBJS): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) $< -o $@

# Create a header file that contains addressing information needed by the code

pass1.mem:

up_mem.h: pass1.mem
	@echo "#ifndef __ARCH_MEM_H" >up_mem.h
	@echo "#define __ARCH_MEM_H" >>up_mem.h
	@if [ -e pass1.mem ]; then \
		echo "#define UP_STACK_BASE $(STACK_BASE)" >>up_mem.h ; \
		echo "#define UP_HEAP1_BASE $(HEAP1_BASE)" >> up_mem.h ; \
		echo "#define UP_HEAP1_END  0x2000" >> up_mem.h ; \
		echo "#define UP_HEAP2_BASE $(HEAP2_BASE)" >> up_mem.h ; \
		echo "#define UP_HEAP2_END  0x8000" >> up_mem.h ; \
	else \
		echo "#define UP_STACK_BASE 0x30" >>up_mem.h ; \
		echo "#define UP_HEAP1_BASE 0x0100" >> up_mem.h ; \
		echo "#define UP_HEAP1_END  0x2000" >> up_mem.h ; \
		echo "#define UP_HEAP2_BASE 0x6000" >> up_mem.h ; \
		echo "#define UP_HEAP2_END  0x8000" >> up_mem.h ; \
	fi
	@echo "#endif /* __ARCH_MEM_H */" >>up_mem.h

# Combine all objects in this directory into a library

libarch$(LIBEXT): up_mem.h $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $${obj} FAILED!" ; exit 1 ; } ; \
	done ; )

# This is a kludge to work around some conflicting symbols in libsdcc.liXqueb

$(SDCCLIBDIR)/libmysdcc.lib: $(SDCCLIBDIR)/libsdcc.lib
	@cat $(SDCCLIBDIR)/libsdcc.lib | \
		grep -v calloc  | grep -v malloc  | grep -v realloc  | \
		grep -v free    | grep -v vprintf | grep -v _strncpy | \
		grep -v _strchr | grep -v _strlen | grep -v _strcmp  | \
		grep -v _strcpy | grep -v _memcmp | grep -v _memcpy  | \
		grep -v _memset \
		> libmysdcc.lib
	@sudo mv -f libmysdcc.lib $(SDCCLIBDIR)/libmysdcc.lib

# This target builds the final executable

pass1.ihx: up_mem.h $(SDCCLIBDIR)/libmysdcc.lib $(LINKOBJS)
	$(CC) $(LDFLAGS) $(LDPATHES) $(SDCCPATH) $(LINKOBJS) $(LDLIBS) $(SDCCLIBS) -o $@
	@rm -f up_mem.h
	@rm -f up_allocateheap$(OBJEXT) libarch$(LIBEXT)
	@$(MAKE) TOPDIR=$(TOPDIR) libarch$(LIBEXT)

nuttx.ihx: up_mem.h $(SDCCLIBDIR)/libmysdcc.lib $(LINKOBJS)
	$(CC) $(LDFLAGS) $(LDPATHES) $(SDCCPATH) $(LINKOBJS) $(LDLIBS) $(SDCCLIBS) -o $@

nuttx$(EXEEXT): pass1.ihx nuttx.ihx
	@rm -f pass1.*
	packihx nuttx.ihx > $(TOPDIR)/nuttx$(EXEEXT)
	@cp -f nuttx.map $(TOPDIR)/.

# Build dependencies

.depend: Makefile up_mem.h $(DEPSRCS)
	$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	touch $@

depend: .depend

clean:
	rm -f libarch$(LIBEXT) up_mem.h
	rm -f *.asm *.rel *.lst *.rst *.sym *.adb *.lnk *.map *.mem *.ihx *.hex *~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
