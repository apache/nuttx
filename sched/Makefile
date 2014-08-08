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
AOBJS = $(ASRCS:.S=$(OBJEXT))

MISC_SRCS  = os_start.c os_bringup.c errno_getptr.c sched_garbage.c
MISC_SRCS += sched_getfiles.c sched_getsockets.c sched_getstreams.c

ifeq ($(CONFIG_NUTTX_KERNEL),y)
MISC_SRCS += errno_get.c errno_set.c
endif

TSK_SRCS  = prctl.c exit.c getpid.c
TSK_SRCS += task_create.c task_init.c task_setup.c task_activate.c task_start.c
TSK_SRCS += task_delete.c task_exit.c task_exithook.c task_recover.c
TSK_SRCS += task_restart.c task_spawn.c task_spawnparms.c task_terminate.c
TSK_SRCS += sched_addreadytorun.c sched_removereadytorun.c sched_addprioritized.c
TSK_SRCS += sched_mergepending.c sched_addblocked.c sched_removeblocked.c
TSK_SRCS += sched_free.c sched_gettcb.c sched_verifytcb.c sched_releasetcb.c

ifeq ($(CONFIG_ARCH_HAVE_VFORK),y)
ifeq ($(CONFIG_SCHED_WAITPID),y)
TSK_SRCS += task_vfork.c
endif
endif

ifneq ($(CONFIG_BINFMT_DISABLE),y)
ifeq ($(CONFIG_LIBC_EXECFUNCS),y)
TSK_SRCS += task_posixspawn.c
endif
endif

ifeq ($(CONFIG_SCHED_STARTHOOK),y)
TSK_SRCS += task_starthook.c
endif

SCHED_SRCS = sched_setparam.c sched_setpriority.c sched_getparam.c
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

WDOG_SRCS = wd_initialize.c wd_create.c wd_start.c wd_cancel.c wd_delete.c
WDOG_SRCS += wd_gettime.c

ifeq ($(CONFIG_SCHED_TICKLESS),y)
TIME_SRCS += sched_timerexpiration.c
else
TIME_SRCS += sched_processtimer.c
endif

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
TIME_SRCS += nanosleep.c
endif

CLOCK_SRCS  = clock_initialize.c clock_settime.c clock_gettime.c clock_getres.c
CLOCK_SRCS += clock_time2ticks.c clock_abstime2ticks.c clock_ticks2time.c
CLOCK_SRCS += clock_gettimeofday.c clock_systimer.c

ifneq ($(CONFIG_DISABLE_POSIX_TIMERS),y)
TIMER_SRCS += timer_initialize.c timer_create.c timer_delete.c timer_getoverrun.c
TIMER_SRCS += timer_gettime.c timer_settime.c timer_release.c
endif

ifeq ($(CONFIG_PAGING),y)
PGFILL_SRCS = pg_miss.c pg_worker.c
endif

IRQ_SRCS = irq_initialize.c irq_attach.c irq_dispatch.c irq_unexpectedisr.c

include group/Make.defs
include semaphore/Make.defs
include signal/Make.defs
include pthread/Make.defs
include mqueue/Make.defs
include environ/Make.defs

CSRCS  = $(MISC_SRCS) $(TSK_SRCS) $(SCHED_SRCS) $(WDOG_SRCS)
CSRCS += $(TIME_SRCS)  $(TIMER_SRCS) $(PGFILL_SRCS)
CSRCS += $(IRQ_SRCS) $(CLOCK_SRCS)
CSRCS += $(GRP_SRCS) $(SEM_SRCS) $(SIGNAL_SRCS) $(PTHREAD_SRCS) $(MQUEUE_SRCS) $(ENV_SRCS)

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
