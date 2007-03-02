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
		  --code-loc 0x2040 --code-size 0x5fc0 \
		  --xram-loc 0x0100 --xram-size 0x1f00

HEAP_START	= ${shell cat pass1.mem | grep "EXTERNAL RAM" | sed -e "s/[ ][ ]*/ /g" | cut -d' ' -f5}

DEPSRCS		= $(SRCS) $(LINKSSRCS)

all: up_head$(OBJEXT) libarch$(LIBEXT)

$(ASRCS) $(LINKASRCS): %$(ASMEXT): %.S
	$(CPP) -P $(CPPFLAGS) -D__ASSEMBLY__ -DUP_HEAP_START=0x0100 -DUP_HEAP_END=0x2000 $< -o $@

$(AOBJS) $(LINKOBJS): $(ASRCS) $(LINKASRCS)
	$(AS) $(ASFLAGS) $<

$(COBJS): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) -DUP_HEAP_START=0x0100 -DUP_HEAP_END=0x2000 $< -o $@

# Combine all objects in this directory into a library

libarch$(LIBEXT): $(OBJS)
	( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $${obj} FAILED!" ; exit 1 ; } ; \
	done ; )

# This is a kludge to work around some conflicting symbols in libsdcc.lib

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

pass1$(EXEEXT): $(SDCCLIBDIR)/libmysdcc.lib $(LINKOBJS)
	$(CC) $(LDFLAGS) $(LDPATHES) $(SDCCPATH) $(LINKOBJS) $(LDLIBS) $(SDCCLIBS) -o pass1.ihx

nuttx$(EXEEXT): pass1$(EXEEXT)
	$(CC) -c $(CFLAGS) -DUP_HEAP_START=$(HEAP_START) -DUP_HEAP_END=0x2000 \
		up_allocateheap.c -o up_allocateheap$(OBJEXT)
	@rm -f pass1.*
	$(CC) $(LDFLAGS) $(LDPATHES) $(SDCCPATH) $(LINKOBJS) $(LDLIBS) $(SDCCLIBS) -o nuttx.ihx
	packihx nuttx.ihx > $(TOPDIR)/nuttx$(EXEEXT)
	@cp -f nuttx.map $(TOPDIR)/.

# Build dependencies

.depend: Makefile $(DEPSRCS)
	$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	touch $@

depend: .depend

clean:
	rm -f libarch$(LIBEXT) *.asm *.rel *.lst *.rst *.sym *.adb *.lnk *.map *.mem *.ihx *.hex *~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
