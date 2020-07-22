############################################################################
# syscall/Makefile
#
#   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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
###########################################################################

include $(TOPDIR)/Make.defs

include proxies/Make.defs
include stubs/Make.defs
include wraps/Make.defs

MKSYSCALL = "$(TOPDIR)$(DELIM)tools$(DELIM)mksyscall$(HOSTEXEEXT)"
CSVFILE = "$(TOPDIR)$(DELIM)syscall$(DELIM)syscall.csv"

ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
ifeq ($(CONFIG_LIB_SYSCALL),y)
PROXY_SRCS += syscall_names.c
else
WRAP_SRCS += syscall_names.c
endif
endif
STUB_SRCS += syscall_stublookup.c

AOBJS = $(ASRCS:.S=$(OBJEXT))

PROXY_OBJS = $(PROXY_SRCS:.c=$(OBJEXT))
STUB_OBJS = $(STUB_SRCS:.c=$(OBJEXT))
WRAP_OBJS = $(WRAP_SRCS:.c=$(OBJEXT))

CSRCS = $(PROXY_SRCS) $(STUB_SRCS) $(WRAP_SRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

PROXYDEPPATH = --dep-path proxies
STUBDEPPATH = --dep-path stubs
WRAPDEPPATH = --dep-path wraps
VPATH = proxies:stubs:wraps

BIN1 = libproxies$(LIBEXT)
BIN2 = libstubs$(LIBEXT)
BIN3 = libwraps$(LIBEXT)
SYSCALLWRAPS = syscall_wraps.ldcmd

all: $(BIN1) $(BIN2) $(BIN3) $(SYSCALLWRAPS)
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN1): $(PROXY_OBJS)
	$(call ARCHIVE, $@, $(PROXY_OBJS))

$(BIN2): $(STUB_OBJS)
	$(call ARCHIVE, $@, $(STUB_OBJS))

$(BIN3): $(WRAP_OBJS)
	$(call ARCHIVE, $@, $(WRAP_OBJS))

$(SYSCALLWRAPS): .context

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MKDEP) $(PROXYDEPPATH) $(STUBDEPPATH) $(WRAPDEPPATH) \
	  "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

.context: syscall.csv
	$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools -f Makefile.host mksyscall
ifeq ($(CONFIG_LIB_SYSCALL),y)
	$(Q) (cd proxies; $(MKSYSCALL) -p $(CSVFILE);)
	$(Q) (cd stubs; $(MKSYSCALL) -s $(CSVFILE);)
endif
ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
	$(Q) (cd wraps; $(MKSYSCALL) -w $(CSVFILE);)
	$(Q) $(CPP) $(CPPFLAGS) $(SYSCALLWRAPS:.ldcmd=.h) | \
	  sed -e '1,/WRAPOPTSTARTS/d' -e '/^#/d' > $(SYSCALLWRAPS)
endif
	$(Q) touch $@

context: .context

clean:
	$(call DELFILE, $(BIN1))
	$(call DELFILE, $(BIN2))
	$(call DELFILE, $(BIN3))
ifneq ($(OBJEXT),)
	$(call DELFILE, proxies$(DELIM)*$(OBJEXT))
	$(call DELFILE, stubs$(DELIM)*$(OBJEXT))
	$(call DELFILE, wraps$(DELIM)*$(OBJEXT))
endif
	$(call CLEAN)

distclean: clean
	$(call DELFILE, .context)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, proxies$(DELIM)*.c)
	$(call DELFILE, stubs$(DELIM)*.c)
	$(call DELFILE, wraps$(DELIM)*.c)
	$(call DELFILE, $(SYSCALLWRAPS))

-include Make.dep
