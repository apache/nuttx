/****************************************************************************
 * sched/sched/sched_note.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
struct note_filter_s
{
  struct note_filter_mode_s mode;
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  struct note_filter_irq_s irq_mask;
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  struct note_filter_syscall_s syscall_mask;
#endif
};
#endif

struct note_startalloc_s
{
  struct note_common_s nsa_cmn; /* Common note parameters */
#if CONFIG_TASK_NAME_SIZE > 0
  char nsa_name[CONFIG_TASK_NAME_SIZE + 1];
#endif
};

#if CONFIG_TASK_NAME_SIZE > 0
#  define SIZEOF_NOTE_START(n) (sizeof(struct note_start_s) + (n) - 1)
#else
#  define SIZEOF_NOTE_START(n) (sizeof(struct note_start_s))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
static struct note_filter_s g_note_filter =
{
  .mode =
    {
      .flag = CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#ifdef CONFIG_SMP
      .cpuset = CONFIG_SCHED_INSTRUMENTATION_CPUSET,
#endif
    }
};

#ifdef CONFIG_SMP
static unsigned int g_note_disabled_irq_nest[CONFIG_SMP_NCPUS];
#else
static unsigned int g_note_disabled_irq_nest[1];
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_common
 *
 * Description:
 *   Fill in some of the common fields in the note structure.
 *
 * Input Parameters:
 *   tcb    - The TCB containing the information
 *   note   - The common note structure to use
 *   length - The total lengthof the note structure
 *   type   - The type of the note
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void note_common(FAR struct tcb_s *tcb,
                        FAR struct note_common_s *note,
                        uint8_t length, uint8_t type)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_HIRES
  struct timespec ts;

  clock_systime_timespec(&ts);
#else
  uint32_t systime    = (uint32_t)clock_systime_ticks();
#endif

  /* Save all of the common fields */

  note->nc_length     = length;
  note->nc_type       = type;
  note->nc_priority   = tcb->sched_priority;
#ifdef CONFIG_SMP
  note->nc_cpu        = tcb->cpu;
#endif
  note->nc_pid[0]     = (uint8_t)(tcb->pid & 0xff);
  note->nc_pid[1]     = (uint8_t)((tcb->pid >> 8) & 0xff);

#ifdef CONFIG_SCHED_INSTRUMENTATION_HIRES
  note->nc_systime_nsec[0] = (uint8_t)(ts.tv_nsec         & 0xff);
  note->nc_systime_nsec[1] = (uint8_t)((ts.tv_nsec >> 8)  & 0xff);
  note->nc_systime_nsec[2] = (uint8_t)((ts.tv_nsec >> 16) & 0xff);
  note->nc_systime_nsec[3] = (uint8_t)((ts.tv_nsec >> 24) & 0xff);
  note->nc_systime_sec[0] = (uint8_t)(ts.tv_sec         & 0xff);
  note->nc_systime_sec[1] = (uint8_t)((ts.tv_sec >> 8)  & 0xff);
  note->nc_systime_sec[2] = (uint8_t)((ts.tv_sec >> 16) & 0xff);
  note->nc_systime_sec[3] = (uint8_t)((ts.tv_sec >> 24) & 0xff);
#else
  /* Save the LS 32-bits of the system timer in little endian order */

  note->nc_systime[0] = (uint8_t)(systime         & 0xff);
  note->nc_systime[1] = (uint8_t)((systime >> 8)  & 0xff);
  note->nc_systime[2] = (uint8_t)((systime >> 16) & 0xff);
  note->nc_systime[3] = (uint8_t)((systime >> 24) & 0xff);
#endif
}

/****************************************************************************
 * Name: note_isenabled
 *
 * Description:
 *   Check whether the instrumentation is enabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned if the instrumentation is enabled.
 *
 ****************************************************************************/

static inline int note_isenabled(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!(g_note_filter.mode.flag & NOTE_FILTER_MODE_FLAG_ENABLE))
    {
      return false;
    }

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((g_note_filter.mode.cpuset & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return false;
    }
#endif
#endif

  return true;
}

/****************************************************************************
 * Name: note_isenabled_syscall
 *
 * Description:
 *   Check whether the syscall instrumentation is enabled.
 *
 * Input Parameters:
 *   nr - syscall number
 *
 * Returned Value:
 *   True is returned if the instrumentation is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
static inline int note_isenabled_syscall(int nr)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!note_isenabled())
    {
      return false;
    }

  /* Exclude the case of syscall called by the interrupt handler which is
   * not traced.
   */

  if (up_interrupt_context())
    {
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
#ifdef CONFIG_SMP
      int cpu = this_cpu();
#else
      int cpu = 0;
#endif

      if (g_note_disabled_irq_nest[cpu] > 0)
        {
          return false;
        }
#else
      return false;
#endif
    }

  /* If the syscall trace is disabled or the syscall number is masked,
   * do nothing.
   */

  if (!(g_note_filter.mode.flag & NOTE_FILTER_MODE_FLAG_SYSCALL) ||
      NOTE_FILTER_SYSCALLMASK_ISSET(nr - CONFIG_SYS_RESERVED,
                                    &g_note_filter.syscall_mask))
    {
      return false;
    }
#endif

  return true;
}
#endif

/****************************************************************************
 * Name: note_isenabled_irqhandler
 *
 * Description:
 *   Check whether the interrupt handler instrumentation is enabled.
 *
 * Input Parameters:
 *   irq   - IRQ number
 *   enter - interrupt enter/leave flag
 *
 * Returned Value:
 *   True is returned if the instrumentation is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static inline int note_isenabled_irq(int irq, bool enter)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!note_isenabled())
    {
      return false;
    }

  /* If the IRQ trace is disabled or the IRQ number is masked, disable
   * subsequent syscall traces until leaving the interrupt handler
   */

  if (!(g_note_filter.mode.flag & NOTE_FILTER_MODE_FLAG_IRQ) ||
      NOTE_FILTER_IRQMASK_ISSET(irq, &g_note_filter.irq_mask))
    {
#ifdef CONFIG_SMP
      int cpu = this_cpu();
#else
      int cpu = 0;
#endif

      if (enter)
        {
          g_note_disabled_irq_nest[cpu]++;
        }
      else
        {
          g_note_disabled_irq_nest[cpu]--;
        }

      return false;
    }
#endif

  return true;
}
#endif

/****************************************************************************
 * Name: note_spincommon
 *
 * Description:
 *   Common logic for NOTE_SPINLOCK, NOTE_SPINLOCKED, and NOTE_SPINUNLOCK
 *
 * Input Parameters:
 *   tcb  - The TCB containing the information
 *   note - The common note structure to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
static void note_spincommon(FAR struct tcb_s *tcb,
                            FAR volatile spinlock_t *spinlock,
                            int type)
{
  struct note_spinlock_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_spinlock_s), type);

  note.nsp_spinlock[0] = (uint8_t)((uintptr_t)spinlock & 0xff);
  note.nsp_spinlock[1] = (uint8_t)(((uintptr_t)spinlock >> 8)  & 0xff);
#if UINTPTR_MAX > UINT16_MAX
  note.nsp_spinlock[2] = (uint8_t)(((uintptr_t)spinlock >> 16) & 0xff);
  note.nsp_spinlock[3] = (uint8_t)(((uintptr_t)spinlock >> 24) & 0xff);
#if UINTPTR_MAX > UINT32_MAX
  note.nsp_spinlock[4] = (uint8_t)(((uintptr_t)spinlock >> 32) & 0xff);
  note.nsp_spinlock[5] = (uint8_t)(((uintptr_t)spinlock >> 40) & 0xff);
  note.nsp_spinlock[6] = (uint8_t)(((uintptr_t)spinlock >> 48) & 0xff);
  note.nsp_spinlock[7] = (uint8_t)(((uintptr_t)spinlock >> 56) & 0xff);
#endif
#endif

  note.nsp_value    = (uint8_t)*spinlock;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_spinlock_s));
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_*
 *
 * Description:
 *   These are the hooks into the scheduling instrumentation logic.  Each
 *   simply formats the note associated with the schedule event and adds
 *   that note to the circular buffer.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

void sched_note_start(FAR struct tcb_s *tcb)
{
  struct note_startalloc_s note;
  unsigned int length;
#if CONFIG_TASK_NAME_SIZE > 0
  int namelen;
#endif

  if (!note_isenabled())
    {
      return;
    }

  /* Copy the task name (if possible) and get the length of the note */

#if CONFIG_TASK_NAME_SIZE > 0
  namelen = strlen(tcb->name);

  DEBUGASSERT(namelen <= CONFIG_TASK_NAME_SIZE);
  strncpy(note.nsa_name, tcb->name, CONFIG_TASK_NAME_SIZE + 1);

  length = SIZEOF_NOTE_START(namelen + 1);
#else
  length = SIZEOF_NOTE_START(0)
#endif

  /* Finish formatting the note */

  note_common(tcb, &note.nsa_cmn, length, NOTE_START);

  /* Add the note to circular buffer */

  sched_note_add(&note, length);
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  struct note_stop_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_stop_s), NOTE_STOP);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_stop_s));
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{
  struct note_suspend_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.nsu_cmn, sizeof(struct note_suspend_s),
              NOTE_SUSPEND);
  note.nsu_state           = tcb->task_state;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_suspend_s));
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
  struct note_resume_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.nre_cmn, sizeof(struct note_resume_s), NOTE_RESUME);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_resume_s));
}

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_start_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_start_s),
              NOTE_CPU_START);
  note.ncs_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_start_s));
}

void sched_note_cpu_started(FAR struct tcb_s *tcb)
{
  struct note_cpu_started_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_started_s),
              NOTE_CPU_STARTED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_started_s));
}

void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_pause_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_pause_s),
              NOTE_CPU_PAUSE);
  note.ncp_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_pause_s));
}

void sched_note_cpu_paused(FAR struct tcb_s *tcb)
{
  struct note_cpu_paused_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_paused_s),
              NOTE_CPU_PAUSED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_paused_s));
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_resume_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resume_s),
              NOTE_CPU_RESUME);
  note.ncr_target = (uint8_t)cpu;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_resume_s));
}

void sched_note_cpu_resumed(FAR struct tcb_s *tcb)
{
  struct note_cpu_resumed_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resumed_s),
              NOTE_CPU_RESUMED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_resumed_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_note_premption(FAR struct tcb_s *tcb, bool locked)
{
  struct note_preempt_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.npr_cmn, sizeof(struct note_preempt_s),
              locked ? NOTE_PREEMPT_LOCK : NOTE_PREEMPT_UNLOCK);
  note.npr_count[0] = (uint8_t)(tcb->lockcount & 0xff);
  note.npr_count[1] = (uint8_t)((tcb->lockcount >> 8) & 0xff);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_preempt_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
void sched_note_csection(FAR struct tcb_s *tcb, bool enter)
{
  struct note_csection_s note;

  if (!note_isenabled())
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_csection_s),
              enter ? NOTE_CSECTION_ENTER : NOTE_CSECTION_LEAVE);
#ifdef CONFIG_SMP
  note.ncs_count[0] = (uint8_t)(tcb->irqcount & 0xff);
  note.ncs_count[1] = (uint8_t)((tcb->irqcount >> 8) & 0xff);
#endif

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_csection_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_note_spinlock(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCK);
}

void sched_note_spinlocked(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCKED);
}

void sched_note_spinunlock(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_UNLOCK);
}

void sched_note_spinabort(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_ABORT);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr, int argc, ...)
{
  struct note_syscall_enter_s note;
  FAR struct tcb_s *tcb = this_task();
  unsigned int length;
  va_list ap;
  uintptr_t arg;
  int i;
  uint8_t *args;

  if (!note_isenabled_syscall(nr))
    {
      return;
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!(g_note_filter.mode.flag & NOTE_FILTER_MODE_FLAG_SYSCALL_ARGS))
    {
      argc = 0;
    }
#endif

  /* Format the note */

  length = SIZEOF_NOTE_SYSCALL_ENTER(argc);
  note_common(tcb, &note.nsc_cmn, length, NOTE_SYSCALL_ENTER);
  DEBUGASSERT(nr <= UCHAR_MAX);
  note.nsc_nr = nr;
  DEBUGASSERT(argc <= MAX_SYSCALL_ARGS);
  note.nsc_argc = argc;

  /* If needed, retrieve the given syscall arguments */

  va_start(ap, argc);

  args = note.nsc_args;
  for (i = 0; i < argc; i++)
    {
      arg = (uintptr_t)va_arg(ap, uintptr_t);
      *args++ = (uint8_t)(arg & 0xff);
      *args++ = (uint8_t)((arg >> 8)  & 0xff);
#if UINTPTR_MAX > UINT16_MAX
      *args++ = (uint8_t)((arg >> 16) & 0xff);
      *args++ = (uint8_t)((arg >> 24) & 0xff);
#if UINTPTR_MAX > UINT32_MAX
      *args++ = (uint8_t)((arg >> 32) & 0xff);
      *args++ = (uint8_t)((arg >> 40) & 0xff);
      *args++ = (uint8_t)((arg >> 48) & 0xff);
      *args++ = (uint8_t)((arg >> 56) & 0xff);
#endif
#endif
    }

  va_end(ap);

  /* Add the note to circular buffer */

  sched_note_add((FAR const uint8_t *)&note, length);
}

void sched_note_syscall_leave(int nr, uintptr_t result)
{
  struct note_syscall_leave_s note;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_syscall(nr))
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.nsc_cmn, sizeof(struct note_syscall_leave_s),
              NOTE_SYSCALL_LEAVE);
  DEBUGASSERT(nr <= UCHAR_MAX);
  note.nsc_nr     = nr;

  note.nsc_result[0] = (uint8_t)(result & 0xff);
  note.nsc_result[1] = (uint8_t)((result >> 8)  & 0xff);
#if UINTPTR_MAX > UINT16_MAX
  note.nsc_result[2] = (uint8_t)((result >> 16) & 0xff);
  note.nsc_result[3] = (uint8_t)((result >> 24) & 0xff);
#if UINTPTR_MAX > UINT32_MAX
  note.nsc_result[4] = (uint8_t)((result >> 32) & 0xff);
  note.nsc_result[5] = (uint8_t)((result >> 40) & 0xff);
  note.nsc_result[6] = (uint8_t)((result >> 48) & 0xff);
  note.nsc_result[7] = (uint8_t)((result >> 56) & 0xff);
#endif
#endif

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_syscall_leave_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  struct note_irqhandler_s note;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_irq(irq, enter))
    {
      return;
    }

  /* Format the note */

  note_common(tcb, &note.nih_cmn, sizeof(struct note_irqhandler_s),
              enter ? NOTE_IRQ_ENTER : NOTE_IRQ_LEAVE);
  DEBUGASSERT(irq <= UCHAR_MAX);
  note.nih_irq = irq;

  /* Add the note to circular buffer */

  sched_note_add((FAR const uint8_t *)&note,
                 sizeof(struct note_irqhandler_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER

/****************************************************************************
 * Name: sched_note_filter_mode
 *
 * Description:
 *   Set and get note filter mode.
 *   (Same as NOTECTL_GETMODE / NOTECTL_SETMODE ioctls)
 *
 * Input Parameters:
 *   oldm - A writable pointer to struct note_filter_mode_s to get current
 *          filter mode
 *          If 0, no data is written.
 *   newm - A read-only pointer to struct note_filter_mode_s which holds the
 *          new filter mode
 *          If 0, the filter mode is not updated.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_note_filter_mode(struct note_filter_mode_s *oldm,
                            struct note_filter_mode_s *newm)
{
  irqstate_t irq_mask;

  irq_mask = enter_critical_section();

  if (oldm != NULL)
    {
      *oldm = g_note_filter.mode;
    }

  if (newm != NULL)
    {
      g_note_filter.mode = *newm;
    }

  leave_critical_section(irq_mask);
}

/****************************************************************************
 * Name: sched_note_filter_syscall
 *
 * Description:
 *   Set and get syscall filter setting
 *   (Same as NOTECTL_GETSYSCALLFILTER / NOTECTL_SETSYSCALLFILTER ioctls)
 *
 * Input Parameters:
 *   oldf - A writable pointer to struct note_filter_syscall_s to get
 *          current syscall filter setting
 *          If 0, no data is written.
 *   newf - A read-only pointer to struct note_filter_syscall_s of the
 *          new syscall filter setting
 *          If 0, the setting is not updated.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_filter_syscall(struct note_filter_syscall_s *oldf,
                               struct note_filter_syscall_s *newf)
{
  irqstate_t irq_mask;

  irq_mask = enter_critical_section();

  if (oldf != NULL)
    {
      /* Return the current filter setting */

      *oldf = g_note_filter.syscall_mask;
    }

  if (newf != NULL)
    {
      /* Replace the syscall filter mask by the provided setting */

      g_note_filter.syscall_mask = *newf;
    }

  leave_critical_section(irq_mask);
}
#endif

/****************************************************************************
 * Name: sched_note_filter_irq
 *
 * Description:
 *   Set and get IRQ filter setting
 *   (Same as NOTECTL_GETIRQFILTER / NOTECTL_SETIRQFILTER ioctls)
 *
 * Input Parameters:
 *   oldf - A writable pointer to struct note_filter_irq_s to get
 *          current IRQ filter setting
 *          If 0, no data is written.
 *   newf - A read-only pointer to struct note_filter_irq_s of the new
 *          IRQ filter setting
 *          If 0, the setting is not updated.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_filter_irq(struct note_filter_irq_s *oldf,
                           struct note_filter_irq_s *newf)
{
  irqstate_t irq_mask;

  irq_mask = enter_critical_section();

  if (oldf != NULL)
    {
      /* Return the current filter setting */

      *oldf = g_note_filter.irq_mask;
    }

  if (newf != NULL)
    {
      /* Replace the syscall filter mask by the provided setting */

      g_note_filter.irq_mask = *newf;
    }

  leave_critical_section(irq_mask);
}
#endif

#endif /* CONFIG_SCHED_INSTRUMENTATION_FILTER */
