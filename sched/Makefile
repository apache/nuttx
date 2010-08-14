############################################################################
# sched/Makefile
#
#   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

ASRCS		= 
AOBJS		= $(ASRCS:.S=$(OBJEXT))

MISC_SRCS	= os_start.c get_errno_ptr.c sched_garbage.c \
		  sched_setupstreams.c sched_getfiles.c sched_getsockets.c sched_getstreams.c \
		  sched_setupidlefiles.c sched_setuptaskfiles.c sched_setuppthreadfiles.c \
		  sched_releasefiles.c

TSK_SRCS	= task_create.c task_init.c task_setup.c task_activate.c \
		  task_start.c task_delete.c task_deletecurrent.c task_restart.c \
		  exit.c abort.c atexit.c getpid.c  \
		  sched_addreadytorun.c sched_removereadytorun.c sched_addprioritized.c \
		  sched_mergepending.c sched_addblocked.c sched_removeblocked.c \
		  sched_free.c sched_gettcb.c sched_verifytcb.c sched_releasetcb.c

SCHED_SRCS	= sched_setparam.c sched_setpriority.c sched_getparam.c \
		  sched_setscheduler.c sched_getscheduler.c \
		  sched_yield.c sched_rrgetinterval.c sched_foreach.c \
		  sched_getprioritymax.c sched_getprioritymin.c \
		  sched_lock.c sched_unlock.c sched_lockcount.c
ifeq ($(CONFIG_PRIORITY_INHERITANCE),y)
SCHED_SRCS	+= sched_reprioritize.c 
endif

ENV_SRCS	= env_getenvironptr.c env_dup.c env_share.c env_release.c \
		  env_findvar.c env_removevar.c \
		  env_clearenv.c env_getenv.c env_putenv.c env_setenv.c env_unsetenv.c

WDOG_SRCS	= wd_initialize.c wd_create.c wd_start.c wd_cancel.c wd_delete.c \
		  wd_gettime.c

TIME_SRCS	= sched_processtimer.c

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
TIME_SRCS	+= sleep.c usleep.c
endif
CLOCK_SRCS	= clock_initialize.c clock_settime.c clock_gettime.c clock_getres.c \
		  clock_time2ticks.c clock_abstime2ticks.c clock_ticks2time.c \
		  clock_gettimeofday.c

SIGNAL_SRCS	= sig_initialize.c \
		  sig_action.c sig_procmask.c sig_pending.c sig_suspend.c \
		  sig_kill.c sig_queue.c sig_waitinfo.c sig_timedwait.c \
		  sig_emptyset.c sig_fillset.c sig_addset.c sig_delset.c \
		  sig_ismember.c sig_findaction.c \
		  sig_allocatependingsigaction.c sig_releasependingsigaction.c \
                  sig_unmaskpendingsignal.c sig_removependingsignal.c \
		  sig_releasependingsignal.c sig_lowest.c sig_mqnotempty.c \
		  sig_cleanup.c sig_received.c sig_deliver.c

MQUEUE_SRCS	= mq_open.c mq_close.c mq_unlink.c \
		  mq_send.c mq_timedsend.c mq_sndinternal.c \
		  mq_receive.c mq_timedreceive.c mq_rcvinternal.c \
		  mq_setattr.c mq_getattr.c mq_initialize.c mq_descreate.c \
		  mq_findnamed.c mq_msgfree.c mq_msgqfree.c mq_waitirq.c
ifneq ($(CONFIG_DISABLE_SIGNALS),y)
MQUEUE_SRCS	+= mq_notify.c
endif

PTHREAD_SRCS	= pthread_attrinit.c pthread_attrdestroy.c \
		  pthread_attrsetschedpolicy.c pthread_attrgetschedpolicy.c \
		  pthread_attrsetinheritsched.c pthread_attrgetinheritsched.c \
		  pthread_attrsetstacksize.c pthread_attrgetstacksize.c \
		  pthread_attrsetschedparam.c pthread_attrgetschedparam.c \
		  pthread_create.c pthread_exit.c pthread_join.c pthread_detach.c \
		  pthread_yield.c pthread_getschedparam.c pthread_setschedparam.c \
		  pthread_mutexattrinit.c pthread_mutexattrdestroy.c \
		  pthread_mutexattrgetpshared.c pthread_mutexattrsetpshared.c \
		  pthread_mutexinit.c pthread_mutexdestroy.c \
		  pthread_mutexlock.c pthread_mutextrylock.c pthread_mutexunlock.c \
		  pthread_condinit.c pthread_conddestroy.c \
		  pthread_condattrinit.c pthread_condattrdestroy.c \
		  pthread_condwait.c pthread_condsignal.c pthread_condbroadcast.c \
		  pthread_barrierattrinit.c pthread_barrierattrdestroy.c \
		  pthread_barrierattrgetpshared.c pthread_barrierattrsetpshared.c \
		  pthread_barrierinit.c pthread_barrierdestroy.c pthread_barrierwait.c \
		  pthread_cancel.c pthread_setcancelstate.c \
		  pthread_keycreate.c pthread_setspecific.c pthread_getspecific.c pthread_keydelete.c \
		  pthread_initialize.c pthread_completejoin.c pthread_findjoininfo.c \
		  pthread_removejoininfo.c pthread_once.c pthread_setschedprio.c
ifneq ($(CONFIG_DISABLE_SIGNALS),y)
PTHREAD_SRCS	+= pthread_condtimedwait.c pthread_kill.c pthread_sigmask.c
endif
ifeq ($(CONFIG_MUTEX_TYPES),y)
PTHREAD_SRCS	+= pthread_mutexattrsettype.c pthread_mutexattrgettype.c
endif

SEM_SRCS	= sem_initialize.c sem_init.c sem_destroy.c\
		  sem_open.c sem_close.c sem_unlink.c \
		  sem_wait.c sem_trywait.c sem_post.c sem_getvalue.c \
                  sem_waitirq.c sem_findnamed.c
ifeq ($(CONFIG_PRIORITY_INHERITANCE),y)
SEM_SRCS	+= sem_holder.c
endif

ifneq ($(CONFIG_DISABLE_POSIX_TIMERS),y)
TIMER_SRCS	= timer_initialize.c timer_create.c timer_delete.c timer_getoverrun.c \
		  timer_gettime.c timer_settime.c timer_release.c
endif

ifeq ($(CONFIG_SCHED_WORKQUEUE),y)
WORK_SRCS	= work_thread.c work_queue.c work_cancel.c
endif

ifeq ($(CONFIG_PAGING),y)
PGFILL_SRCS	= pg_miss.c
endif

IRQ_SRCS	= irq_initialize.c irq_attach.c irq_dispatch.c irq_unexpectedisr.c

CSRCS		= $(MISC_SRCS) $(TSK_SRCS) $(SCHED_SRCS) $(WDOG_SRCS) $(TIME_SRCS) \
		  $(SEM_SRCS) $(TIMER_SRCS) $(WORK_SRCS) $(PGFILL_SRCS) $(IRQ_SRCS)

ifneq ($(CONFIG_DISABLE_CLOCK),y)
CSRCS		+= $(CLOCK_SRCS)
endif
ifneq ($(CONFIG_DISABLE_PTHREAD),y)
CSRCS		+= $(PTHREAD_SRCS)
endif
ifneq ($(CONFIG_DISABLE_SIGNALS),y)
CSRCS		+= $(SIGNAL_SRCS)
endif
ifneq ($(CONFIG_DISABLE_MQUEUE),y)
CSRCS		+= $(MQUEUE_SRCS)
endif
ifneq ($(CONFIG_DISABLE_ENVIRON),y)
CSRCS		+= $(ENV_SRCS)
endif
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libsched$(LIBEXT)

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

.depend: Makefile $(SRCS)
	@$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f $(BIN) *~ .*.swp
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
