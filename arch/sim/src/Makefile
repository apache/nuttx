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

ASRCS		= up_setjmp.S
AOBJS		= $(ASRCS:.S=$(OBJEXT))
CSRCS		= up_initialize.c up_idle.c up_interruptcontext.c \
		  up_initialstate.c up_createstack.c up_usestack.c \
		  up_releasestack.c  up_unblocktask.c up_blocktask.c \
		  up_releasepending.c up_reprioritizertr.c \
		  up_exit.c up_schedulesigaction.c up_allocateheap.c \
		  up_devconsole.c
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

LDFLAGS			= $(ARCHSCRIPT)
EXTRA_LIBS		= -lc

LINKOBJS	=
LINKLIBS	=
LDPATHES	= $(addprefix -L$(TOPDIR)/,$(dir $(LINKLIBS)))
LDLIBS		= $(patsubst lib%,-l%,$(basename $(notdir $(LINKLIBS))))

all: up_head$(OBJEXT) libarch$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(CC) -c $(CFLAGS) -D__ASSEMBLY__ $< -o $@

$(COBJS) up_head$(OBJEXT): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) $< -o $@

libarch$(LIBEXT): $(OBJS)
	( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $obj FAILED!" ; exit 1 ; } ; \
	done ; )

nuttx:
	$(CC) $(LDFLAGS) $(LDPATHES) -o $(TOPDIR)/$@ \
		$(addprefix $(TOPDIR)/,$(LINKOBJS)) \
		-Wl,--start-group $(LDLIBS) $(EXTRA_LIBS) -Wl,--end-group
	@$(NM) $(TOPDIR)/$@ | \
	grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map


.depend: Makefile $(SRCS)
	$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	touch $@

depend: .depend

clean:
	rm -f libarch$(LIBEXT) *~
	if [ ! -z "$(OBJEXT)" ]; then rm -f *$(OBJEXT); fi

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
