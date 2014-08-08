############################################################################
# sched/Makefile
#
#   Copyright (C) 2007-2014 Gregory Nutt. All rights reserved.
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
############################################################################

-include $(TOPDIR)/Make.defs

VPATH =
DEPPATH = --dep-path .

ASRCS =

SCHED_SRCS  = sched_garbage.c sched_getfiles.c
SCHED_SRCS += sched_addreadytorun.c sched_removereadytorun.c sched_addprioritized.c
SCHED_SRCS += sched_mergepending.c sched_addblocked.c sched_removeblocked.c
SCHED_SRCS += sched_free.c sched_gettcb.c sched_verifytcb.c sched_releasetcb.c
SCHED_SRCS += sched_getsockets.c sched_getstreams.c
SCHED_SRCS += sched_setparam.c sched_setpriority.c sched_getparam.c
SCHED_SRCS += sched_setscheduler.c sched_getscheduler.c
SCHED_SRCS += sched_yield.c sched_rrgetinterval.c sched_foreach.c
SCHED_SRCS += sched_lock.c sched_unlock.c sched_lockcount.c sched_self.c

ifeq ($(CONFIG_SCHED_ATEXIT),y)
SCHED_SRCS += atexit.c
endif

ifeq ($(CONFIG_SCHED_ONEXIT),y)
SCHED_SRCS += on_exit.c
endif

ifeq ($(CONFIG_PRIORITY_INHERITANCE),y)
SCHED_SRCS += sched_reprioritize.c
endif

ifeq ($(CONFIG_SCHED_WAITPID),y)
SCHED_SRCS += sched_waitpid.c
ifeq ($(CONFIG_SCHED_HAVE_PARENT),y)
SCHED_SRCS += sched_waitid.c sched_wait.c
endif
endif

ifeq ($(CONFIG_SCHED_CPULOAD),y)
SCHED_SRCS += sched_cpuload.c
endif

ifeq ($(CONFIG_SCHED_TICKLESS),y)
SCHED_SRCS += sched_timerexpiration.c
else
SCHED_SRCS += sched_processtimer.c
endif

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
SCHED_SRCS += nanosleep.c
endif

include init/Make.defs
include irq/Make.defs
include paging/Make.defs
include group/Make.defs
include task/Make.defs
include errno/Make.defs
include wdog/Make.defs
include semaphore/Make.defs
include signal/Make.defs
include pthread/Make.defs
include mqueue/Make.defs
include clock/Make.defs
include timer/Make.defs
include environ/Make.defs

# REVISIT
TSK_SRCS += prctl.c exit.c getpid.c

CSRCS  = $(INIT_SRCS) $(IRQ_SRCS) $(PG_SRCS) $(GRP_SRCS) $(SCHED_SRCS)
CSRCS += $(TSK_SRCS) $(ERRNO_SRCS) $(WDOG_SRCS) $(SEM_SRCS)
CSRCS += $(SIGNAL_SRCS) $(PTHREAD_SRCS) $(MQUEUE_SRCS) $(CLOCK_SRCS)
CSRCS += $(TIMER_SRCS) $(ENV_SRCS)

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libsched$(LIBEXT)

all: $(BIN)
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
