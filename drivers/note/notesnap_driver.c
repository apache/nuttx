/****************************************************************************
 * drivers/note/notesnap_driver.c
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

#include <stdatomic.h>

#include <nuttx/note/note_driver.h>
#include <nuttx/note/notesnap_driver.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/sched_note.h>
#include <sched/sched.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct notesnap_chunk_s
{
  uint8_t type;
#ifdef CONFIG_SMP
  uint8_t cpu;
#endif
  pid_t pid;
#ifdef CONFIG_SCHED_INSTRUMENTATION_PERFCOUNT
  uint32_t count;
#else
  struct timespec time;
#endif
  uintptr_t args;
};

struct notesnap_s
{
  struct note_driver_s driver;
  struct notifier_block nb;
  size_t index;
  bool dumping;
  struct notesnap_chunk_s buffer[CONFIG_DRIVERS_NOTESNAP_NBUFFERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void notesnap_start(FAR struct note_driver_s *drv,
                           FAR struct tcb_s *tcb);
static void notesnap_stop(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb);
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void notesnap_suspend(FAR struct note_driver_s *drv,
                             FAR struct tcb_s *tcb);
static void notesnap_resume(FAR struct note_driver_s *drv,
                            FAR struct tcb_s *tcb);
#endif
#ifdef CONFIG_SMP
static void notesnap_cpu_start(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, int cpu);
static void notesnap_cpu_started(FAR struct note_driver_s *drv,
                                 FAR struct tcb_s *tcb);
#  ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void notesnap_cpu_pause(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, int cpu);
static void notesnap_cpu_paused(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb);
static void notesnap_cpu_resume(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb, int cpu);
static void notesnap_cpu_resumed(FAR struct note_driver_s *drv,
                                 FAR struct tcb_s *tcb);
#  endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
static void notesnap_premption(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, bool locked);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
static void notesnap_csection(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, bool enter);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
static void notesnap_spinlock(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb,
                              FAR volatile void *spinlock, int type);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
static void notesnap_syscall_enter(FAR struct note_driver_s *drv, int nr,
                                   int argc, FAR va_list *ap);
static void notesnap_syscall_leave(FAR struct note_driver_s *drv, int nr,
                                   uintptr_t result);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static void notesnap_irqhandler(FAR struct note_driver_s *drv, int irq,
                                FAR void *handler, bool enter);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_notesnap_ops =
{
  NULL,
  notesnap_start,
  notesnap_stop,
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  notesnap_suspend,
  notesnap_resume,
#endif
#ifdef CONFIG_SMP
  notesnap_cpu_start,
  notesnap_cpu_started,
#  ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  notesnap_cpu_pause,
  notesnap_cpu_paused,
  notesnap_cpu_resume,
  notesnap_cpu_resumed,
#  endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
  notesnap_premption,
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
  notesnap_csection,
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  notesnap_spinlock,
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  notesnap_syscall_enter,
  notesnap_syscall_leave,
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  notesnap_irqhandler,
#endif
};

static struct notesnap_s g_notesnap =
{
  {&g_notesnap_ops}
};

static FAR const char *g_notesnap_type[] =
{
  "NOTE_START",
  "NOTE_STOP",
  "NOTE_SUSPEND",
  "NOTE_RESUME",
  "NOTE_CPU_START",
  "NOTE_CPU_STARTED",
  "NOTE_CPU_PAUSE",
  "NOTE_CPU_PAUSED",
  "NOTE_CPU_RESUME",
  "NOTE_CPU_RESUMED",
  "NOTE_PREEMPT_LOCK",
  "NOTE_PREEMPT_UNLOCK",
  "NOTE_CSECTION_ENTER",
  "NOTE_CSECTION_LEAVE",
  "NOTE_SPINLOCK_LOCK",
  "NOTE_SPINLOCK_LOCKED",
  "NOTE_SPINLOCK_UNLOCK",
  "NOTE_SPINLOCK_ABORT",
  "NOTE_SYSCALL_ENTER",
  "NOTE_SYSCALL_LEAVE",
  "NOTE_IRQ_ENTER",
  "NOTE_IRQ_LEAVE",
  "NOTE_DUMP_STRING",
  "NOTE_DUMP_BINARY"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notesnap_common
 ****************************************************************************/

static inline void notesnap_common(FAR struct note_driver_s *drv,
                                   FAR struct tcb_s *tcb, uint8_t type,
                                   uintptr_t args)
{
  FAR struct notesnap_s *snap = (FAR struct notesnap_s *)drv;
  FAR struct notesnap_chunk_s *note;
  size_t index;

  if (snap->dumping)
    {
      return;
    }

  /* Atomic operation, equivalent to snap.index++; */

  index = atomic_fetch_add(&snap->index, 1);
  note = &snap->buffer[index % CONFIG_DRIVERS_NOTESNAP_NBUFFERS];

  note->type = type;
#ifdef CONFIG_SMP
  note->cpu = tcb->cpu;
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PERFCOUNT
  note->count = up_perf_gettime();
#else
  clock_systime_timespec(&note->time);
#endif
  note->pid = tcb->pid;
  note->args = args;
}

/****************************************************************************
 * Name: notesnap_*
 ****************************************************************************/

static void notesnap_start(FAR struct note_driver_s *drv,
                           FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_START, 0);
}

static void notesnap_stop(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_STOP, 0);
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void notesnap_suspend(FAR struct note_driver_s *drv,
                             FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_SUSPEND, 0);
}

static void notesnap_resume(FAR struct note_driver_s *drv,
                            FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_RESUME, 0);
}
#endif

#ifdef CONFIG_SMP
static void notesnap_cpu_start(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, int cpu)
{
  notesnap_common(drv, tcb, NOTE_CPU_START, cpu);
}

static void notesnap_cpu_started(FAR struct note_driver_s *drv,
                                 FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_CPU_STARTED, 0);
}

#  ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void notesnap_cpu_pause(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, int cpu)
{
  notesnap_common(drv, tcb, NOTE_CPU_PAUSE, cpu);
}

static void notesnap_cpu_paused(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_CPU_PAUSED, 0);
}

static void notesnap_cpu_resume(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb, int cpu)
{
  notesnap_common(drv, tcb, NOTE_CPU_RESUME, cpu);
}

static void notesnap_cpu_resumed(FAR struct note_driver_s *drv,
                                 FAR struct tcb_s *tcb)
{
  notesnap_common(drv, tcb, NOTE_CPU_RESUMED, 0);
}
#  endif
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
static void notesnap_premption(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, bool locked)
{
  notesnap_common(drv, tcb, locked ? NOTE_PREEMPT_LOCK :
                  NOTE_PREEMPT_UNLOCK, 0);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
static void notesnap_csection(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, bool enter)
{
  notesnap_common(drv, tcb, enter ? NOTE_CSECTION_ENTER :
                  NOTE_CSECTION_LEAVE, 0);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
static void notesnap_spinlock(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb,
                              FAR volatile void *spinlock, int type)
{
  notesnap_common(drv, tcb, type, (uintptr_t)spinlock);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static void notesnap_irqhandler(FAR struct note_driver_s *drv, int irq,
                                FAR void *handler, bool enter)
{
  notesnap_common(drv, this_task(), enter ? NOTE_IRQ_ENTER :
                  NOTE_IRQ_LEAVE, irq);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
static void notesnap_syscall_enter(FAR struct note_driver_s *drv, int nr,
                                   int argc, FAR va_list *ap)
{
  notesnap_common(drv, this_task(), NOTE_SYSCALL_ENTER, nr);
}

static void notesnap_syscall_leave(FAR struct note_driver_s *drv, int nr,
                                   uintptr_t result)
{
  notesnap_common(drv, this_task(), NOTE_SYSCALL_LEAVE, nr);
}
#endif

static int notesnap_notifier(FAR struct notifier_block *nb,
                             unsigned long action, FAR void *data)
{
  if (action == PANIC_KERNEL)
    {
      notesnap_dump();
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notesnap_register
 ****************************************************************************/

int notesnap_register(void)
{
  g_notesnap.nb.notifier_call = notesnap_notifier;
  panic_notifier_chain_register(&g_notesnap.nb);
  return note_driver_register(&g_notesnap.driver);
}

/****************************************************************************
 * Name: notesnap_dump_with_stream
 ****************************************************************************/

void notesnap_dump_with_stream(FAR struct lib_outstream_s *stream)
{
  size_t i;
  size_t index = g_notesnap.index % CONFIG_DRIVERS_NOTESNAP_NBUFFERS;

#ifdef CONFIG_SCHED_INSTRUMENTATION_PERFCOUNT
  uint32_t lastcount = g_notesnap.buffer[index].count;
  struct timespec lasttime =
  {
    0
  };
#endif

  /* Stop recording while dumping */

  atomic_store(&g_notesnap.dumping, true);

  for (i = index; i != index - 1;
       i == CONFIG_DRIVERS_NOTESNAP_NBUFFERS - 1 ? i = 0 : i++)
    {
      FAR struct notesnap_chunk_s *note = &g_notesnap.buffer[i];

#ifdef CONFIG_SCHED_INSTRUMENTATION_PERFCOUNT
      struct timespec time;
      uint32_t elapsed = note->count < lastcount ?
                         note->count + UINT32_MAX - lastcount :
                         note->count - lastcount;
      up_perf_convert(elapsed, &time);
      clock_timespec_add(&lasttime, &time, &lasttime);
      lastcount = note->count;
#endif

      lib_sprintf(stream,
                  "snapshoot: [%u.%09u] "
#ifdef CONFIG_SMP
                  "[CPU%d] "
#endif
                  "[%d] %-16s %#" PRIxPTR "\n",
#ifdef CONFIG_SCHED_INSTRUMENTATION_PERFCOUNT
                  (unsigned)lasttime.tv_sec,
                  (unsigned)lasttime.tv_nsec,
#else
                  (unsigned)note->time.tv_sec, (unsigned)note->time.tv_nsec,
#endif
#ifdef CONFIG_SMP
                  note->cpu,
#endif
                  note->pid, g_notesnap_type[note->type], note->args);
    }

  atomic_store(&g_notesnap.dumping, false);
}

/****************************************************************************
 * Name: notesnap_dump
 ****************************************************************************/

void notesnap_dump(void)
{
  struct lib_syslogstream_s stream;
  lib_syslogstream_open(&stream);
  notesnap_dump_with_stream(&stream.public);
  lib_syslogstream_close(stream);
}
