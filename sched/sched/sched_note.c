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
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

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
  uint32_t systime    = (uint32_t)clock_systime_ticks();

  /* Save all of the common fields */

  note->nc_length     = length;
  note->nc_type       = type;
  note->nc_priority   = tcb->sched_priority;
#ifdef CONFIG_SMP
  note->nc_cpu        = tcb->cpu;
#endif
  note->nc_pid[0]     = (uint8_t)(tcb->pid & 0xff);
  note->nc_pid[1]     = (uint8_t)((tcb->pid >> 8) & 0xff);

  /* Save the LS 32-bits of the system timer in little endian order */

  note->nc_systime[0] = (uint8_t)(systime         & 0xff);
  note->nc_systime[1] = (uint8_t)((systime >> 8)  & 0xff);
  note->nc_systime[2] = (uint8_t)((systime >> 16) & 0xff);
  note->nc_systime[3] = (uint8_t)((systime >> 24) & 0xff);
}

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_spinlock_s), type);
  note.nsp_spinlock = (FAR void *)spinlock;
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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.nsp_cmn, sizeof(struct note_stop_s), NOTE_STOP);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_stop_s));
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{
  struct note_suspend_s note;

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.nre_cmn, sizeof(struct note_resume_s), NOTE_RESUME);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_resume_s));
}

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_start_s note;

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_started_s),
              NOTE_CPU_STARTED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_started_s));
}

void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_pause_s note;

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_paused_s),
              NOTE_CPU_PAUSED);

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_cpu_paused_s));
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_resume_s note;

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.nsc_cmn, sizeof(struct note_syscall_enter_s),
              NOTE_SYSCALL_ENTER);
  DEBUGASSERT(nr <= UCHAR_MAX);
  note.nsc_nr = nr;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_syscall_enter_s));
}

void sched_note_syscall_leave(int nr, uintptr_t result)
{
  struct note_syscall_leave_s note;
  FAR struct tcb_s *tcb = this_task();

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

  /* Format the note */

  note_common(tcb, &note.nsc_cmn, sizeof(struct note_syscall_leave_s),
              NOTE_SYSCALL_LEAVE);
  note.nsc_result = result;
  DEBUGASSERT(nr <= UCHAR_MAX);
  note.nsc_nr     = nr;

  /* Add the note to circular buffer */

  sched_note_add(&note, sizeof(struct note_syscall_leave_s));
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  struct note_irqhandler_s note;
  FAR struct tcb_s *tcb = this_task();

#ifdef CONFIG_SMP
  /* Ignore notes that are not in the set of monitored CPUs */

  if ((CONFIG_SCHED_INSTRUMENTATION_CPUSET & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return;
    }
#endif

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
