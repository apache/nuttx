############################################################################
# arch/sim/src/Makefile
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

CFLAGS		+= -I$(TOPDIR)/sched

ASRCS		= up_setjmp.S
AOBJS		= $(ASRCS:.S=$(OBJEXT))
CSRCS		= up_initialize.c up_idle.c up_interruptcontext.c \
		  up_initialstate.c up_createstack.c up_usestack.c \
		  up_releasestack.c  up_unblocktask.c up_blocktask.c \
		  up_releasepending.c up_reprioritizertr.c \
		  up_exit.c up_schedulesigaction.c up_allocateheap.c \
		  up_devconsole.c
HOSTSRCS	= up_stdio.c
ifeq ($(CONFIG_FS_FAT),y)
CSRCS		+= up_blockdevice.c up_deviceimage.c
endif
ifeq ($(CONFIG_NET),y)
CSRCS		+= up_uipdriver.c
HOSTSRCS	+= up_tapdev.c
endif

COBJS		= $(CSRCS:.c=$(OBJEXT))

NXOBJS		= $(AOBJS) $(COBJS)
HOSTOBJS	= $(HOSTSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS) $(HOSTSRCS)
OBJS		= $(AOBJS) $(COBJS) $(HOSTOBJS)

LDFLAGS		= $(ARCHSCRIPT)
STDLIBS		= -lc
ifeq ($(CONFIG_FS_FAT),y)
STDLIBS		+= -lz
endif

LINKOBJS	= up_head$(OBJEXT)
LINKLIBS	=
LDPATHES	= $(addprefix -L$(TOPDIR)/,$(dir $(LINKLIBS)))
LDLIBS		= $(patsubst lib%,-l%,$(basename $(notdir $(LINKLIBS))))

all: up_head$(OBJEXT) libarch$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(HOSTOBJS): %$(OBJEXT): %.c
	@echo "CC:  $<"
	@$(CC) -c $(HOSTCFLAGS) $< -o $@

# The architecture-specific library
libarch$(LIBEXT): $(NXOBJS)
	@( for obj in $(NXOBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

# A partially linked object containing only NuttX code (no interface to host OS)
# Change the names of most symbols that conflict with libc symbols.

Linux-names.dat: nuttx-names.dat
	@cp $^ $@

Cygwin-names.dat: nuttx-names.dat
	@cat $^ | sed -e "s/^/_/g" >$@

nuttx.rel : libarch$(LIBEXT) $(HOSTOS)-names.dat $(LINKOBJS)
	@echo "LD:  nuttx.rel"
	@$(LD) -r $(LDFLAGS) $(LDPATHES) -o $@ $(LINKOBJS) --start-group $(LDLIBS) --end-group $(EXTRA_LIBS)
	@$(OBJCOPY) --redefine-syms=$(HOSTOS)-names.dat $@

# Generate the final NuttX binary by linking the host-specific objects with the NuttX
# specific objects (with munged names)

nuttx$(EXEEXT): nuttx.rel $(HOSTOBJS)
	@echo "LD:  nuttx$(EXEEXT)"
	@$(CC) $(LDFLAGS) $(LDPATHES) -o $(TOPDIR)/$@ nuttx.rel $(HOSTOBJS) $(STDLIBS)
	@$(NM) $(TOPDIR)/$@ | \
		grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
		sort > $(TOPDIR)/System.map

.depend: Makefile $(SRCS)
	@$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f nuttx.rel libarch$(LIBEXT) *~ .*.swp
	@rm -f nuttx.rel Linux-names.dat Cygwin-names.dat
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
