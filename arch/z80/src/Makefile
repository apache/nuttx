############################################################################
# arch/z80/src/Makefile
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

############################################################################
# Makefile fragments

-include $(TOPDIR)/Make.defs
-include chip/Make.defs

############################################################################
# Tools

MKDEP 		= $(TOPDIR)/tools/mkdeps.sh

CFLAGS		+= -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(TOPDIR)/sched
LDFLAGS		= $(ARCHSCRIPT)

LDPATHES	= $(addprefix -L$(TOPDIR)/,$(dir $(LINKLIBS)))
LDLIBS		= $(patsubst lib%,-l%,$(basename $(notdir $(LINKLIBS))))

CPPFLAGS	= $(ARCHINCLUDES) $(ARCHDEFINES) -D__ASSEMBLY__

############################################################################
# Files and directories

# There should be one head source (.asm file)

HEAD_AOBJ	= $(HEAD_ASRC:$(ASMEXT)=$(OBJEXT))

# Assembly sources and objects

ASRCS		= $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

# C sources and objects

CSRCS		= $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

# All sources and objcts

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

# Sources and objects for the test target (plus HEAD_AOBJ

TEST_AOBJS	= $(TEST_ASRCS:$(ASMEXT)=$(OBJEXT))
TEST_COBJS	= $(TEST_CSRCS:.c=$(OBJEXT))

TESTSRCS	= up_irqtest.c
TESTOBJS	= $(TESTSRCS:.c=$(OBJEXT))
TESTEXTRAOBJS	= up_savecontext$(OBJEXT) up_restorecontext$(OBJEXT)

# Sources that can have dependencies (no .asm files)

DEPSRCS		= $(SRCS)

# Directories

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
BOARDDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src/board

VPATH		= chip:common

# Libraries

LIBGCC		= ${shell $(CC) -print-libgcc-file-name}

# Supports dynamic sizing of HEAP

HEAP_BASE	= ${shell \
			if [ -e pass1.mem ]; then \
				cat pass1.mem | grep "ROM/EPROM/FLASH" | \
				sed -e "s/[ ][ ]*/ /g" | cut -d' ' -f4 ; \
			else \
				echo $(DEF_HEAP_BASE) ; \
			fi \
		   }

STACK_BASE	= ${shell \
			if [ -e pass1.mem ]; then \
				cat pass1.mem | grep "Stack starts" | \
				cut -d' ' -f4 ; \
			else \
				echo $(DEF_STACK_BASE) ; \
			fi \
		   }

############################################################################
# Targets

all: $(HEAD_AOBJ) libarch$(LIBEXT)

$(AOBJS) $(HEAD_AOBJ) $(TEST_AOBJS): %$(OBJEXT): %$(ASMEXT)
	$(AS) $(ASFLAGS) $@ $<

$(COBJS) $(TEST_COBJS): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) $< -o $@

# Create a header file that contains addressing information needed by the code

pass1.mem:

up_mem.h: pass1.mem
	@echo "#ifndef __ARCH_MEM_H" >up_mem.h
	@echo "#define __ARCH_MEM_H" >>up_mem.h
	@echo "" >>up_mem.h
	@echo "#define UP_DEFAULT_STACK_BASE $(DEF_STACK_BASE)" >>up_mem.h
	@echo "#define UP_DEFAULT_HEAP_BASE $(DEF_HEAP_BASE)" >> up_mem.h
	@echo "" >>up_mem.h
	@echo "#define UP_STACK_BASE         $(STACK_BASE)" >>up_mem.h
	@echo "#if UP_STACK_BASE > UP_DEFAULT_STACK_BASE" >>up_mem.h
	@echo "#  error \"Stack overlap:  $(DEF_STACK_BASE) < $(STACK_BASE)\""  >>up_mem.h
	@echo "#elif UP_STACK_BASE < UP_DEFAULT_STACK_BASE" >>up_mem.h
	@echo "#  warning \"Wasted stack: $(DEF_STACK_BASE) > $(STACK_BASE)\""  >>up_mem.h
	@echo "#endif" >>up_mem.h
	@echo "" >>up_mem.h
	@echo "#define UP_HEAP_BASE          $(HEAP_BASE)" >> up_mem.h
	@echo "#define UP_HEAP_END           $(CONFIG_DRAM_END)" >> up_mem.h
	@echo "" >>up_mem.h
	@echo "#endif /* __ARCH_MEM_H */" >>up_mem.h

# Combine all objects in this directory into a library

libarch$(LIBEXT): up_mem.h $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $${obj} FAILED!" ; exit 1 ; } ; \
	done ; )

# This builds the libboard library in the board/ subdirectory 

board/libboard$(LIBEXT):
	$(MAKE) -C board TOPDIR=$(TOPDIR) libboard$(LIBEXT)

# This target builds the final executable

pass1.ihx: up_mem.h $(HEAD_AOBJ) board/libboard$(LIBEXT)
	$(CC) $(LDFLAGS) -L$(BOARDDIR) $(SDCCPATH) $(HEAD_AOBJ) \
		-llibboard$(LIBEXT) $(SDCCLIBS) -o $@
	@rm -f up_mem.h
	@rm -f up_allocateheap$(OBJEXT) libarch$(LIBEXT)
	@$(MAKE) TOPDIR=$(TOPDIR) libarch$(LIBEXT)

nuttx.ihx: up_mem.h $(HEAD_AOBJ)
	$(CC) $(LDFLAGS) -L$(BOARDDIR) $(SDCCPATH) $(HEAD_AOBJ) \
		-llibboard$(LIBEXT) $(SDCCLIBS) -o $@

nuttx$(EXEEXT): pass1.ihx nuttx.ihx
	@rm -f pass1.*
	packihx nuttx.ihx > $(TOPDIR)/nuttx$(EXEEXT)
	@cp -f nuttx.map $(TOPDIR)/.

# This target builds a test program to verify interrupt context switching.  irqtest is
# a PHONY target that just sets upt the up_irqtest build correctly

up_irqtest.ihx: $(TEST_COBJS)
	$(CC) $(LDFLAGS) -L. $(SDCCPATH) $(HEAD_AOBJ) $(TEST_COBJS) $(TESTEXTRAOBJS) $(SDCCLIBS) -o $@

irqtest:
	$(MAKE) TOPDIR=../../.. up_irqtest.ihx

# Build dependencies

.depend: Makefile up_mem.h chip/Make.defs $(DEPSRCS)
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR=$(TOPDIR) depend ; \
	fi
	$(MKDEP) --dep-path chip --dep-path common $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR=$(TOPDIR) clean ; \
	fi
	rm -f libarch$(LIBEXT) up_mem.h
	rm -f *.asm *.rel *.lst *.rst *.sym *.adb *.lnk *.map *.mem *.ihx *.hex *~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR=$(TOPDIR) distclean ; \
	fi
	rm -f Make.dep .depend

-include Make.dep
