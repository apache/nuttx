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

#include <stddef.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/segger/sysview.h>

#include <SEGGER_RTT.h>
#include <SEGGER_SYSVIEW.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct note_sysview_driver_s
{
  struct note_driver_s driver;
  unsigned int irq[CONFIG_SMP_NCPUS];
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  struct note_filter_syscall_s syscall_marker;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void note_sysview_start(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb);
static void note_sysview_stop(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb);
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void note_sysview_suspend(FAR struct note_driver_s *drv,
                                 FAR struct tcb_s *tcb);
static void note_sysview_resume(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static void note_sysview_irqhandler(FAR struct note_driver_s *drv, int irq,
                                    FAR void *handler, bool enter);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
static void note_sysview_syscall_enter(FAR struct note_driver_s *drv,
                                       int nr, int argc, va_list *ap);
static void note_sysview_syscall_leave(FAR struct note_driver_s *drv,
                                       int nr, uintptr_t result);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_note_sysview_ops =
{
  NULL,                       /* add */
  note_sysview_start,         /* start */
  note_sysview_stop,          /* stop */
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  note_sysview_suspend,       /* suspend */
  note_sysview_resume,        /* resume */
#endif
#ifdef CONFIG_SMP
  NULL,                       /* cpu_start */
  NULL,                       /* cpu_started */
#  ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  NULL,                       /* cpu_pause */
  NULL,                       /* cpu_paused */
  NULL,                       /* cpu_resume */
  NULL,                       /* cpu_resumed */
#  endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
  NULL,                       /* premption */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
  NULL,                       /* csection */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  NULL,                       /* spinlock */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  note_sysview_syscall_enter, /* syscall_enter */
  note_sysview_syscall_leave, /* syscall_leave */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  note_sysview_irqhandler,    /* irqhandler */
#endif
};

static struct note_sysview_driver_s g_note_sysview_driver =
{
  {
    &g_note_sysview_ops
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_sysview_send_taskinfo
 ****************************************************************************/

static void note_sysview_send_taskinfo(FAR struct tcb_s *tcb)
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
 * Name: note_sysview_get_time
 ****************************************************************************/

static uint64_t note_sysview_get_time(void)
{
  return TICK2USEC(clock_systime_ticks());
}

/****************************************************************************
 * Name: note_sysview_send_tasklist
 ****************************************************************************/

static void note_sysview_send_tasklist(void)
{
  int i;

  for (i = 0; i < g_npidhash; i++)
    {
      if (g_pidhash[i] != NULL)
        {
          note_sysview_send_taskinfo(g_pidhash[i]);
        }
    }
}

/****************************************************************************
 * Name: note_sysview_send_description
 ****************************************************************************/

static void note_sysview_send_description(void)
{
  SEGGER_SYSVIEW_SendSysDesc("N="SEGGER_SYSVIEW_APP_NAME);
  SEGGER_SYSVIEW_SendSysDesc("D="CONFIG_LIBC_HOSTNAME);
  SEGGER_SYSVIEW_SendSysDesc("O=NuttX");
}

/****************************************************************************
 * Name: note_sysview_*
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

static void note_sysview_start(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb)
{
  SEGGER_SYSVIEW_OnTaskCreate(tcb->pid);
  note_sysview_send_taskinfo(tcb);
}

static void note_sysview_stop(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb)
{
  SEGGER_SYSVIEW_OnTaskTerminate(tcb->pid);
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void note_sysview_suspend(FAR struct note_driver_s *drv,
                                 FAR struct tcb_s *tcb)
{
  if (!up_interrupt_context())
    {
      SEGGER_SYSVIEW_OnTaskStopExec();
    }
}

static void note_sysview_resume(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb)
{
  if (!up_interrupt_context())
    {
      if (is_idle_task(tcb))
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
static void note_sysview_irqhandler(FAR struct note_driver_s *drv, int irq,
                                    FAR void *handler, bool enter)
{
  FAR struct note_sysview_driver_s *driver =
      (FAR struct note_sysview_driver_s *)drv;

  if (enter)
    {
      driver->irq[up_cpu_index()] = irq;

      SEGGER_SYSVIEW_OnTaskStopExec();
      SEGGER_SYSVIEW_RecordEnterISR();
    }
  else
    {
      SEGGER_SYSVIEW_RecordExitISR();

      if (up_interrupt_context())
        {
          FAR struct tcb_s *tcb = this_task();

          if (tcb && !is_idle_task(tcb))
            {
              SEGGER_SYSVIEW_OnTaskStartExec(tcb->pid);
            }
          else
            {
              SEGGER_SYSVIEW_OnIdle();
            }
        }

      driver->irq[up_cpu_index()] = 0;
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
static void note_sysview_syscall_enter(FAR struct note_driver_s *drv, int nr,
                                       int argc, va_list *ap)
{
  FAR struct note_sysview_driver_s *driver =
      (FAR struct note_sysview_driver_s *)drv;
  nr -= CONFIG_SYS_RESERVED;

  /* Set the name marker if the current syscall nr is not active */

  if (NOTE_FILTER_SYSCALLMASK_ISSET(nr, &driver->syscall_marker) == 0)
    {
      /* Set the name marker */

      SEGGER_SYSVIEW_NameMarker(nr, g_funcnames[nr]);

      /* Mark the syscall active */

      NOTE_FILTER_SYSCALLMASK_SET(nr, &driver->syscall_marker);

      /* Use the Syscall "0" to identify whether the syscall is enabled,
       * if the host tool is closed abnormally, use this bit to clear
       * the active set.
       */

      if (NOTE_FILTER_SYSCALLMASK_ISSET(0, &driver->syscall_marker) == 0)
        {
          NOTE_FILTER_SYSCALLMASK_SET(0, &driver->syscall_marker);
        }
    }

  SEGGER_SYSVIEW_MarkStart(nr);
}

static void note_sysview_syscall_leave(FAR struct note_driver_s *drv,
                                       int nr, uintptr_t result)
{
  FAR struct note_sysview_driver_s *driver =
      (FAR struct note_sysview_driver_s *)drv;
  nr -= CONFIG_SYS_RESERVED;

  if (NOTE_FILTER_SYSCALLMASK_ISSET(nr, &driver->syscall_marker) != 0)
    {
      SEGGER_SYSVIEW_MarkStop(nr);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_sysview_get_interrupt_id
 *
 * Description:
 *   Retrieve the Id of the currently active interrupt.
 *
 ****************************************************************************/

unsigned int note_sysview_get_interrupt_id(void)
{
  return g_note_sysview_driver.irq[up_cpu_index()];
}

/****************************************************************************
 * Name: note_sysview_get_timestamp
 *
 * Description:
 *   Retrieve a system timestamp for SYSVIEW events.
 *
 ****************************************************************************/

unsigned int note_sysview_get_timestamp(void)
{
  return up_perf_gettime();
}

/****************************************************************************
 * Name: note_sysview_initialize
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

int note_sysview_initialize(void)
{
  uint32_t freq = up_perf_getfreq();

  static const SEGGER_SYSVIEW_OS_API g_sysview_trace_api =
    {
      note_sysview_get_time,
      note_sysview_send_tasklist,
    };

  if (freq == 0)
    {
      syslog(LOG_ERR, "up_perf isn't initialized, sysview isn't available");
      PANIC();
    }

  SEGGER_SYSVIEW_Init(freq, freq, &g_sysview_trace_api,
                      note_sysview_send_description);

#if CONFIG_SEGGER_SYSVIEW_RAM_BASE != 0
  SEGGER_SYSVIEW_SetRAMBase(CONFIG_SEGGER_SYSVIEW_RAM_BASE);
#endif

  SEGGER_SYSVIEW_Start();
  note_driver_register(&g_note_sysview_driver.driver);
  syslog(LOG_NOTICE, "SEGGER RTT Control Block Address: %#" PRIxPTR "\n",
                      (uintptr_t)&_SEGGER_RTT +
                      CONFIG_SEGGER_RTT_UNCACHED_OFF);
  return 0;
}
