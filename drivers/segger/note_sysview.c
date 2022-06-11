/****************************************************************************
 * drivers/segger/note_sysview.c
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
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include <SEGGER_RTT.h>
#include <SEGGER_SYSVIEW.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sysview_s
{
  unsigned int                 irq[CONFIG_SMP_NCPUS];
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  struct note_filter_mode_s    mode;
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  struct note_filter_irq_s     irq_mask;
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  struct note_filter_syscall_s syscall_mask;
  struct note_filter_syscall_s syscall_marker;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sysview_s g_sysview =
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  .mode =
    {
      .flag = CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#ifdef CONFIG_SMP
      .cpuset = CONFIG_SCHED_INSTRUMENTATION_CPUSET,
#endif
    }
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sysview_send_taskinfo
 ****************************************************************************/

static void sysview_send_taskinfo(FAR struct tcb_s *tcb)
{
  SEGGER_SYSVIEW_TASKINFO info;

  info.TaskID     = tcb->pid;
#if CONFIG_TASK_NAME_SIZE > 0
  info.sName      = tcb->name;
#else
  info.sName      = "<noname>";
#endif
  info.Prio       = tcb->sched_priority;
  info.StackBase  = (uintptr_t)tcb->stack_base_ptr;
  info.StackSize  = tcb->adj_stack_size;

  SEGGER_SYSVIEW_SendTaskInfo(&info);
}

/****************************************************************************
 * Name: sysview_get_time
 ****************************************************************************/

static uint64_t sysview_get_time(void)
{
  return TICK2USEC(clock_systime_ticks());
}

/****************************************************************************
 * Name: sysview_send_tasklist
 ****************************************************************************/

static void sysview_send_tasklist(void)
{
  int i;

  for (i = 0; i < g_npidhash; i++)
    {
      if (g_pidhash[i] != NULL)
        {
          sysview_send_taskinfo(g_pidhash[i]);
        }
    }
}

/****************************************************************************
 * Name: sysview_send_description
 ****************************************************************************/

static void sysview_send_description(void)
{
  SEGGER_SYSVIEW_SendSysDesc("N="SEGGER_SYSVIEW_APP_NAME);
  SEGGER_SYSVIEW_SendSysDesc("D="CONFIG_LIBC_HOSTNAME);
  SEGGER_SYSVIEW_SendSysDesc("O=NuttX");
}

/****************************************************************************
 * Name: sysview_isenabled
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

static bool sysview_isenabled(void)
{
  bool enable;

  enable = SEGGER_SYSVIEW_IsStarted();

#if defined(CONFIG_SCHED_INSTRUMENTATION_FILTER) && defined(CONFIG_SMP)
  /* Ignore notes that are not in the set of monitored CPUs */

  if (enable && (g_sysview.mode.cpuset & (1 << this_cpu())) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return false;
    }
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  /* Use the Syscall "0" to identify whether the syscall is enabled,
   * if the host tool is closed abnormally, use this bit to clear
   * the active set.
   */

  if (!enable &&
      NOTE_FILTER_SYSCALLMASK_ISSET(0, &g_sysview.syscall_marker))
    {
      NOTE_FILTER_SYSCALLMASK_ZERO(&g_sysview.syscall_marker);
    }
#endif

  return enable;
}

/****************************************************************************
 * Name: sysview_isenabled_irq
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
static bool sysview_isenabled_irq(int irq, bool enter)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!sysview_isenabled())
    {
      return false;
    }

  /* If the IRQ trace is disabled or the IRQ number is masked, disable
   * subsequent syscall traces until leaving the interrupt handler
   */

  if ((g_sysview.mode.flag & NOTE_FILTER_MODE_FLAG_IRQ) == 0 ||
      NOTE_FILTER_IRQMASK_ISSET(irq, &g_sysview.irq_mask))
    {
      return false;
    }
#endif

  return true;
}
#endif

/****************************************************************************
 * Name: sysview_isenabled_syscall
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
static inline int sysview_isenabled_syscall(int nr)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!sysview_isenabled())
    {
      return false;
    }

  /* If the syscall trace is disabled or the syscall number is masked,
   * do nothing.
   */

  if ((g_sysview.mode.flag & NOTE_FILTER_MODE_FLAG_SYSCALL) == 0 ||
      NOTE_FILTER_SYSCALLMASK_ISSET(nr, &g_sysview.syscall_mask))
    {
      return false;
    }
#endif

  return true;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_start, sched_note_stop, sched_note_switch,
 *       sched_note_premption
 *
 * Description:
 *   Hooks to scheduler monitor
 *
 * Input Parameters:
 *   Varies
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_note_start(FAR struct tcb_s *tcb)
{
  if (!sysview_isenabled())
    {
      return;
    }

  SEGGER_SYSVIEW_OnTaskCreate(tcb->pid);
  sysview_send_taskinfo(tcb);
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  if (!sysview_isenabled())
    {
      return;
    }

  SEGGER_SYSVIEW_OnTaskTerminate(tcb->pid);
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_note_suspend(FAR struct tcb_s *tcb)
{
  if (!sysview_isenabled())
    {
      return;
    }

  if (!up_interrupt_context())
    {
      SEGGER_SYSVIEW_OnTaskStopExec();
    }
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
  if (!sysview_isenabled())
    {
      return;
    }

  if (!up_interrupt_context())
    {
      if (tcb->flink == NULL)
        {
          SEGGER_SYSVIEW_OnIdle();
        }
      else
        {
          SEGGER_SYSVIEW_OnTaskStartExec(tcb->pid);
        }
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  if (!sysview_isenabled_irq(irq, enter))
    {
      return;
    }

  if (enter)
    {
      g_sysview.irq[up_cpu_index()] = irq;

      SEGGER_SYSVIEW_OnTaskStopExec();
      SEGGER_SYSVIEW_RecordEnterISR();
    }
  else
    {
      SEGGER_SYSVIEW_RecordExitISR();

      if (up_interrupt_context())
        {
          FAR struct tcb_s *tcb = this_task();

          if (tcb && tcb->flink != NULL)
            {
              SEGGER_SYSVIEW_OnTaskStartExec(tcb->pid);
            }
          else
            {
              SEGGER_SYSVIEW_OnIdle();
            }
        }

      g_sysview.irq[up_cpu_index()] = 0;
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr, int argc, ...)
{
  nr -= CONFIG_SYS_RESERVED;

  if (sysview_isenabled_syscall(nr) == 0)
    {
      return;
    }

  /* Set the name marker if the current syscall nr is not active */

  if (NOTE_FILTER_SYSCALLMASK_ISSET(nr, &g_sysview.syscall_marker) == 0)
    {
      /* Set the name marker */

      SEGGER_SYSVIEW_NameMarker(nr, g_funcnames[nr]);

      /* Mark the syscall active */

      NOTE_FILTER_SYSCALLMASK_SET(nr, &g_sysview.syscall_marker);

      /* Use the Syscall "0" to identify whether the syscall is enabled,
       * if the host tool is closed abnormally, use this bit to clear
       * the active set.
       */

      if (NOTE_FILTER_SYSCALLMASK_ISSET(0, &g_sysview.syscall_marker) == 0)
        {
          NOTE_FILTER_SYSCALLMASK_SET(0, &g_sysview.syscall_marker);
        }
    }

  SEGGER_SYSVIEW_MarkStart(nr);
}

void sched_note_syscall_leave(int nr, uintptr_t result)
{
  nr -= CONFIG_SYS_RESERVED;

  if (sysview_isenabled_syscall(nr) == 0)
    {
      return;
    }

  if (NOTE_FILTER_SYSCALLMASK_ISSET(nr, &g_sysview.syscall_marker) != 0)
    {
      SEGGER_SYSVIEW_MarkStop(nr);
    }
}
#endif

/****************************************************************************
 * Name: sysview_get_interrupt_id
 *
 * Description:
 *   Retrieve the Id of the currently active interrupt.
 *
 ****************************************************************************/

unsigned int sysview_get_interrupt_id(void)
{
  return g_sysview.irq[up_cpu_index()];
}

/****************************************************************************
 * Name: sysview_get_timestamp
 *
 * Description:
 *   Retrieve a system timestamp for SYSVIEW events.
 *
 ****************************************************************************/

unsigned int sysview_get_timestamp(void)
{
  return up_perf_gettime();
}

/****************************************************************************
 * Name: sysview_initialize
 *
 * Description:
 *   Initializes the SYSVIEW module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int sysview_initialize(void)
{
  uint32_t freq = up_perf_getfreq();

  static const SEGGER_SYSVIEW_OS_API g_sysview_trace_api =
    {
      sysview_get_time,
      sysview_send_tasklist,
    };

  SEGGER_SYSVIEW_Init(freq, freq, &g_sysview_trace_api,
                      sysview_send_description);

#if CONFIG_SEGGER_SYSVIEW_RAM_BASE != 0
  SEGGER_SYSVIEW_SetRAMBase(CONFIG_SEGGER_SYSVIEW_RAM_BASE);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if ((g_sysview.mode.flag & NOTE_FILTER_MODE_FLAG_ENABLE) != 0)
#endif
    {
      SEGGER_SYSVIEW_Start();
    }

  syslog(LOG_NOTICE, "SEGGER RTT Control Block Address: %#" PRIxPTR "\n",
                      (uintptr_t)&_SEGGER_RTT +
                      CONFIG_SEGGER_RTT_UNCACHED_OFF);
  return 0;
}

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
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (oldm != NULL)
    {
      if (SEGGER_SYSVIEW_IsStarted())
        {
          g_sysview.mode.flag |= NOTE_FILTER_MODE_FLAG_ENABLE;
        }
      else
        {
          g_sysview.mode.flag &= ~NOTE_FILTER_MODE_FLAG_ENABLE;
        }

      *oldm = g_sysview.mode;
    }

  if (newm != NULL)
    {
      g_sysview.mode = *newm;

      if ((g_sysview.mode.flag & NOTE_FILTER_MODE_FLAG_ENABLE) != 0)
        {
          if (!SEGGER_SYSVIEW_IsStarted())
            {
              SEGGER_SYSVIEW_Start();
            }
        }
      else
        {
          if (SEGGER_SYSVIEW_IsStarted())
            {
              SEGGER_SYSVIEW_Stop();
            }
        }
    }

  spin_unlock_irqrestore(NULL, flags);
}

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
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (oldf != NULL)
    {
      /* Return the current filter setting */

      *oldf = g_sysview.irq_mask;
    }

  if (newf != NULL)
    {
      /* Replace the syscall filter mask by the provided setting */

      g_sysview.irq_mask = *newf;
    }

  spin_unlock_irqrestore(NULL, flags);
}
#endif

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
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (oldf != NULL)
    {
      /* Return the current filter setting */

      *oldf = g_sysview.syscall_mask;
    }

  if (newf != NULL)
    {
      /* Replace the syscall filter mask by the provided setting */

      g_sysview.syscall_mask = *newf;
    }

  spin_unlock_irqrestore(NULL, flags);
}
#endif

#endif /* CONFIG_SCHED_INSTRUMENTATION_FILTER */
