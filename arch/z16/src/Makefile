############################################################################
# arch/z16/src/Makefile
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
-include chip/Make.defs

ARCHSRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
WARCHSRCDIR	= ${shell cygpath -w $(ARCHSRCDIR)}
USRINCLUDES	= -usrinc:'.;$(WTOPDIR)\sched;$(WARCHSRCDIR)\chip;$(WARCHSRCDIR)\common'
INCLUDES	= $(ARCHSTDINCLUDES) $(USRINCLUDES)
CFLAGS		= $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES)
WLINKLIBS       =
LDFLAGS	        = @${shell cypath -w $(ARCHSRCDIR)/nuttx.linkcmd}

HEAD_AOBJ	= $(HEAD_ASRC:.S=$(OBJEXT))

ASRCS		= $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BOARDDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src/board

VPATH		= chip:common

all: $(HEAD_OBJ) libarch$(LIBEXT)

$(AOBJS) $(HEAD_AOBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

board/libboard$(LIBEXT):
	@$(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT)

wlinklibs:
        $(foreach LIB, $(LINKLIBS), $(eval WLINKLIBS += ${shell cygpath -w $(LIB)}))

nuttx.linkcmd: $(LINKCMDTEMPLATE) wlinklibs
	@cp -f $(LINKCMDTEMPLATE) nuttx.linkcmd
	@echo "\"${shell cygpath -w $(TOPDIR)/nuttx}\"= \\" >>nuttx.linkcmd
	@echo " ${shell cygpath -w $(ARCHSRCDIR)/$(HEAD_AOBJ)}, \\" >>nuttx.linkcmd
	@( for lib in $(WLINKLIBS); do \
		echo " $${lib}, \\" >>nuttx.linkcmd; \
	done ; )
	@echo " ${shell cygpath -w $(ARCHSRCDIR)/board/libboard$(LIBEXT)}"  >>nuttx.linkcmd

nuttx$(EXEEXT): $(HEAD_AOBJ) board/libboard$(LIBEXT) nuttx.linkcmd
        @echo "LD:  nuttx.hex"
	@$(LD) $(LDFLAGS)

.depend: Makefile chip/Make.defs $(SRCS)
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" depend ; \
	fi
	@$(MKDEP) --dep-path chip --dep-path common $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" clean ; \
	fi
	@rm -f libarch$(LIBEXT) nuttx.linkcmd *~ .*.swp
	$(call CLEAN)

distclean: clean
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean ; \
	fi
	@rm -f Make.dep .depend

-include Make.dep
