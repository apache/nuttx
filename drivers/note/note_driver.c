/****************************************************************************
 * drivers/note/note_driver.c
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

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/note/notelog_driver.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

#define note_add(drv, note, notelen)                                         \
  ((drv)->ops->add(drv, note, notelen))
#define note_start(drv, tcb)                                                 \
  ((drv)->ops->start && ((drv)->ops->start(drv, tcb), true))
#define note_stop(drv, tcb)                                                  \
  ((drv)->ops->stop && ((drv)->ops->stop(drv, tcb), true))
#define note_suspend(drv, tcb)                                               \
  ((drv)->ops->suspend && ((drv)->ops->suspend(drv, tcb), true))
#define note_resume(drv, tcb)                                                \
  ((drv)->ops->resume && ((drv)->ops->resume(drv, tcb), true))
#define note_cpu_start(drv, tcb, cpu)                                        \
  ((drv)->ops->cpu_start && ((drv)->ops->cpu_start(drv, tcb, cpu), true))
#define note_cpu_started(drv, tcb)                                           \
  ((drv)->ops->cpu_started && ((drv)->ops->cpu_started(drv, tcb), true))
#define note_cpu_pause(drv, tcb, cpu)                                        \
  ((drv)->ops->cpu_pause && ((drv)->ops->cpu_pause(drv, tcb, cpu), true))
#define note_cpu_paused(drv, tcb)                                            \
  ((drv)->ops->cpu_paused && ((drv)->ops->cpu_paused(drv, tcb), true))
#define note_cpu_resume(drv, tcb, cpu)                                       \
  ((drv)->ops->cpu_resume && ((drv)->ops->cpu_resume(drv, tcb, cpu), true))
#define note_cpu_resumed(drv, tcb)                                           \
  ((drv)->ops->cpu_resumed && ((drv)->ops->cpu_resumed(drv, tcb), true))
#define note_premption(drv, tcb, locked)                                     \
  ((drv)->ops->premption && ((drv)->ops->premption(drv, tcb, locked), true))
#define note_csection(drv, tcb, enter)                                       \
  ((drv)->ops->csection && ((drv)->ops->csection(drv, tcb, enter), true))
#define note_spinlock(drv, tcb, spinlock, type)                              \
  ((drv)->ops->spinlock &&                                                   \
  ((drv)->ops->spinlock(drv, tcb, spinlock, type), true))
#define note_syscall_enter(drv, nr, argc, ap)                                \
  ((drv)->ops->syscall_enter &&                                              \
  ((drv)->ops->syscall_enter(drv, nr, argc, ap), true))
#define note_syscall_leave(drv, nr, result)                                  \
  ((drv)->ops->syscall_leave &&                                              \
  ((drv)->ops->syscall_leave(drv, nr, result), true))
#define note_irqhandler(drv, irq, handler, enter)                            \
  ((drv)->ops->irqhandler &&                                                 \
  ((drv)->ops->irqhandler(drv, irq, handler, enter), true))
#define note_string(drv, ip, buf)                                            \
  ((drv)->ops->string && ((drv)->ops->string(drv, ip, buf), true))
#define note_dump(drv, ip, buf, len)                                         \
  ((drv)->ops->dump && ((drv)->ops->dump(drv, ip, event, buf, len), true))
#define note_vprintf(drv, ip, fmt, va)                                       \
  ((drv)->ops->vprintf && ((drv)->ops->vprintf(drv, ip, fmt, va), true))
#define note_vbprintf(drv, ip, event, fmt, va)                               \
  ((drv)->ops->vbprintf &&                                                   \
  ((drv)->ops->vbprintf(drv, ip, event, fmt, va), true))

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

#if CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0
struct note_taskname_info_s
{
  uint8_t size;
  uint8_t pid[2];
  char name[1];
};

struct note_taskname_s
{
  size_t head;
  size_t tail;
  char buffer[CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE];
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
static struct note_filter_s g_note_filter =
{
  {
     CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE
#ifdef CONFIG_SMP
     , CONFIG_SCHED_INSTRUMENTATION_CPUSET
#endif
  }
};

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static unsigned int g_note_disabled_irq_nest[CONFIG_SMP_NCPUS];
#endif
#endif

FAR static struct note_driver_s *g_note_drivers[CONFIG_DRIVER_NOTE_MAX + 1] =
{
#ifdef CONFIG_DRIVER_NOTERAM
  &g_noteram_driver,
#endif
#ifdef CONFIG_DRIVER_NOTELOG
  &g_notelog_driver,
#endif
  NULL
};

#if CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0
static struct note_taskname_s g_note_taskname;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_flatten
 *
 * Description:
 *   Copy the data in the little endian layout
 *
 ****************************************************************************/

static inline void sched_note_flatten(FAR uint8_t *dst,
                                      FAR void *src, size_t len)
{
#ifdef CONFIG_ENDIAN_BIG
  FAR uint8_t *end = (FAR uint8_t *)src + len - 1;
  while (len-- > 0)
    {
      *dst++ = *end--;
    }
#else
  memcpy(dst, src, len);
#endif
}

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
  clock_t systime = clock_systime_ticks();
#endif

  /* Save all of the common fields */

  note->nc_length   = length;
  note->nc_type     = type;
  note->nc_priority = tcb->sched_priority;
#ifdef CONFIG_SMP
  note->nc_cpu      = tcb->cpu;
#endif
  sched_note_flatten(note->nc_pid, &tcb->pid, sizeof(tcb->pid));

#ifdef CONFIG_SCHED_INSTRUMENTATION_HIRES
  sched_note_flatten(note->nc_systime_nsec, &ts.tv_nsec, sizeof(ts.tv_nsec));
  sched_note_flatten(note->nc_systime_sec, &ts.tv_sec, sizeof(ts.tv_sec));
#else
  /* Save the LS 32-bits of the system timer in little endian order */

  sched_note_flatten(note->nc_systime, &systime, sizeof(systime));
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

  if (CPU_ISSET(&g_note_filter.mode.cpuset, this_cpu()) == 0)
    {
      /* Not in the set of monitored CPUs.  Do not log the note. */

      return false;
    }
#endif
#endif

  return true;
}

/****************************************************************************
 * Name: note_isenabled_switch
 *
 * Description:
 *   Check whether the switch instrumentation is enabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned if the instrumentation is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static inline int note_isenabled_switch(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!note_isenabled())
    {
      return false;
    }

  /* If the switch trace is disabled, do nothing. */

  if ((g_note_filter.mode.flag & NOTE_FILTER_MODE_FLAG_SWITCH) == 0)
    {
      return false;
    }
#endif

  return true;
}
#endif

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
      int cpu = this_cpu();

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
      int cpu = this_cpu();

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
 * Name: note_isenabled_dump
 *
 * Description:
 *   Check whether the dump instrumentation is enabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned if the instrumentation is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
static inline int note_isenabled_dump(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  if (!note_isenabled())
    {
      return false;
    }

  /* If the dump trace is disabled, do nothing. */

  if ((g_note_filter.mode.flag & NOTE_FILTER_MODE_FLAG_DUMP) == 0)
    {
      return false;
    }
#endif

  return true;
}
#endif

#if CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0

/****************************************************************************
 * Name: note_find_taskname
 *
 * Description:
 *   Find task name info corresponding to the specified PID
 *
 * Input Parameters:
 *   PID - Task ID
 *
 * Returned Value:
 *   Pointer to the task name info
 *   If the corresponding info doesn't exist in the buffer, NULL is returned.
 *
 ****************************************************************************/

static FAR struct note_taskname_info_s *note_find_taskname(pid_t pid)
{
  FAR struct note_taskname_info_s *ti;
  int n = g_note_taskname.tail;

  while (n != g_note_taskname.head)
    {
      ti = (FAR struct note_taskname_info_s *)
            &g_note_taskname.buffer[n];
      if (ti->pid[0] + (ti->pid[1] << 8) == pid)
        {
          return ti;
        }

      n += ti->size;
      if (n >= CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE)
        {
          n -= CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: note_record_taskname
 *
 * Description:
 *   Record the task name info of the specified task
 *
 * Input Parameters:
 *   PID - Task ID
 *   name - task name
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void note_record_taskname(pid_t pid, FAR const char *name)
{
  FAR struct note_taskname_info_s *ti;
  size_t tilen;
  size_t namelen;
  size_t skiplen;
  size_t remain;

  namelen = strlen(name);
  DEBUGASSERT(namelen <= CONFIG_TASK_NAME_SIZE);
  tilen = sizeof(struct note_taskname_info_s) + namelen;
  DEBUGASSERT(tilen <= UCHAR_MAX);

  skiplen = CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE - g_note_taskname.head;
  if (skiplen >= tilen + sizeof(struct note_taskname_info_s))
    {
      skiplen = 0; /* Have enough space at the tail - needn't skip */
    }

  if (g_note_taskname.head >= g_note_taskname.tail)
    {
      remain = CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE -
               (g_note_taskname.head - g_note_taskname.tail);
    }
  else
    {
      remain = g_note_taskname.tail - g_note_taskname.head;
    }

  while (skiplen + tilen >= remain)
    {
      /* No enough space, drop the old info */

      ti = (FAR struct note_taskname_info_s *)
            &g_note_taskname.buffer[g_note_taskname.tail];
      g_note_taskname.tail = (g_note_taskname.tail + ti->size) %
                             CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE;
      remain += ti->size;
    }

  if (skiplen)
    {
      /* Fill the skipped region with an invalid info */

      ti = (FAR struct note_taskname_info_s *)
            &g_note_taskname.buffer[g_note_taskname.head];
      ti->size = skiplen;
      ti->pid[0] = 0xff;
      ti->pid[1] = 0xff;
      ti->name[0] = '\0';

      /* Move to the begin of circle buffer */

      g_note_taskname.head = 0;
    }

  ti = (FAR struct note_taskname_info_s *)
        &g_note_taskname.buffer[g_note_taskname.head];
  ti->size = tilen;
  ti->pid[0] = pid & 0xff;
  ti->pid[1] = (pid >> 8) & 0xff;
  strlcpy(ti->name, name, namelen + 1);
  g_note_taskname.head += tilen;
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
  FAR struct note_driver_s **driver;
  bool formatted = false;

#if CONFIG_TASK_NAME_SIZE > 0
  int namelen;
#endif

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_start(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      if (!formatted)
        {
          formatted = true;

          /* Copy the task name (if possible) and
           * get the length of the note
           */

#if CONFIG_TASK_NAME_SIZE > 0
          namelen = strlen(tcb->name);

          DEBUGASSERT(namelen <= CONFIG_TASK_NAME_SIZE);
          strlcpy(note.nsa_name, tcb->name, sizeof(note.nsa_name));

          length = SIZEOF_NOTE_START(namelen + 1);
#else
          length = SIZEOF_NOTE_START(0);
#endif

          /* Finish formatting the note */

          note_common(tcb, &note.nsa_cmn, length, NOTE_START);
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, length);
    }
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  struct note_stop_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

#if CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0
  note_record_taskname(tcb->pid, tcb->name);
#endif

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_stop(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.nsp_cmn, sizeof(struct note_stop_s),
                      NOTE_STOP);
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_stop_s));
    }
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_note_suspend(FAR struct tcb_s *tcb)
{
  struct note_suspend_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled_switch())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (!note_suspend(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.nsu_cmn, sizeof(struct note_suspend_s),
                      NOTE_SUSPEND);
          note.nsu_state = tcb->task_state;
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_suspend_s));
    }
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
  struct note_resume_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled_switch())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_resume(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.nre_cmn, sizeof(struct note_resume_s),
                      NOTE_RESUME);
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_resume_s));
    }
}
#endif

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_start_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_cpu_start(*driver, tcb, cpu))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_start_s),
                      NOTE_CPU_START);
          note.ncs_target = (uint8_t)cpu;
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_cpu_start_s));
    }
}

void sched_note_cpu_started(FAR struct tcb_s *tcb)
{
  struct note_cpu_started_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_cpu_started(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncs_cmn, sizeof(struct note_cpu_started_s),
                      NOTE_CPU_STARTED);
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_cpu_started_s));
    }
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_pause_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled_switch())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_cpu_pause(*driver, tcb, cpu))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_pause_s),
                      NOTE_CPU_PAUSE);
          note.ncp_target = (uint8_t)cpu;
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_cpu_pause_s));
    }
}

void sched_note_cpu_paused(FAR struct tcb_s *tcb)
{
  struct note_cpu_paused_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled_switch())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_cpu_paused(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncp_cmn, sizeof(struct note_cpu_paused_s),
                      NOTE_CPU_PAUSED);
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_cpu_paused_s));
    }
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
  struct note_cpu_resume_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled_switch())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_cpu_resume(*driver, tcb, cpu))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resume_s),
                      NOTE_CPU_RESUME);
          note.ncr_target = (uint8_t)cpu;
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_cpu_resume_s));
    }
}

void sched_note_cpu_resumed(FAR struct tcb_s *tcb)
{
  struct note_cpu_resumed_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled_switch())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_cpu_resumed(*driver, tcb))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncr_cmn, sizeof(struct note_cpu_resumed_s),
                      NOTE_CPU_RESUMED);
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_cpu_resumed_s));
    }
}
#endif
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_note_premption(FAR struct tcb_s *tcb, bool locked)
{
  struct note_preempt_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_premption(*driver, tcb, locked))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.npr_cmn, sizeof(struct note_preempt_s),
                      locked ? NOTE_PREEMPT_LOCK : NOTE_PREEMPT_UNLOCK);
          sched_note_flatten(note.npr_count, &tcb->lockcount,
                             sizeof(tcb->lockcount));
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_preempt_s));
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
void sched_note_csection(FAR struct tcb_s *tcb, bool enter)
{
  struct note_csection_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_csection(*driver, tcb, enter))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.ncs_cmn, sizeof(struct note_csection_s),
                      enter ? NOTE_CSECTION_ENTER : NOTE_CSECTION_LEAVE);
#ifdef CONFIG_SMP
          sched_note_flatten(note.ncs_count, &tcb->irqcount,
                             sizeof(tcb->irqcount));
#endif
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_csection_s));
    }
}
#endif

/****************************************************************************
 * Name: sched_note_spinlock
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
void sched_note_spinlock(FAR struct tcb_s *tcb,
                         FAR volatile spinlock_t *spinlock,
                         int type)
{
  struct note_spinlock_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;

  if (!note_isenabled())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_spinlock(*driver, tcb, spinlock, type))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.nsp_cmn, sizeof(struct note_spinlock_s),
                      type);
          sched_note_flatten(note.nsp_spinlock, &spinlock, sizeof(spinlock));
          note.nsp_value = *(FAR uint8_t *)spinlock;
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_spinlock_s));
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr, int argc, ...)
{
  struct note_syscall_enter_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  FAR struct tcb_s *tcb = this_task();
  unsigned int length;
  FAR uint8_t *args;
  uintptr_t arg;
  va_list ap;
  int i;

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

  va_start(ap, argc);
  for (driver = g_note_drivers; *driver; driver++)
    {
      va_list copy;

      va_copy(copy, ap);
      if (note_syscall_enter(*driver, nr, argc, &copy))
        {
          va_end(copy);
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          va_end(copy);
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          length = SIZEOF_NOTE_SYSCALL_ENTER(argc);
          note_common(tcb, &note.nsc_cmn, length, NOTE_SYSCALL_ENTER);
          DEBUGASSERT(nr <= UCHAR_MAX);
          note.nsc_nr = nr;
          DEBUGASSERT(argc <= MAX_SYSCALL_ARGS);
          note.nsc_argc = argc;

          /* If needed, retrieve the given syscall arguments */

          args = note.nsc_args;
          for (i = 0; i < argc; i++)
            {
              arg = (uintptr_t)va_arg(copy, uintptr_t);
              sched_note_flatten(args, &arg, sizeof(arg));
              args += sizeof(uintptr_t);
            }
        }

      va_end(copy);

      /* Add the note to circular buffer */

      note_add(*driver, &note, length);
    }

    va_end(ap);
}

void sched_note_syscall_leave(int nr, uintptr_t result)
{
  struct note_syscall_leave_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_syscall(nr))
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_syscall_leave(*driver, nr, result))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.nsc_cmn,
                      sizeof(struct note_syscall_leave_s),
                      NOTE_SYSCALL_LEAVE);
          DEBUGASSERT(nr <= UCHAR_MAX);
          note.nsc_nr = nr;

          sched_note_flatten(note.nsc_result, &result, sizeof(result));
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_syscall_leave_s));
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  struct note_irqhandler_s note;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_irq(irq, enter))
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_irqhandler(*driver, irq, handler, enter))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      if (!formatted)
        {
          formatted = true;
          note_common(tcb, &note.nih_cmn, sizeof(struct note_irqhandler_s),
                      enter ? NOTE_IRQ_ENTER : NOTE_IRQ_LEAVE);
          DEBUGASSERT(irq <= UCHAR_MAX);
          note.nih_irq = irq;
        }

      /* Add the note to circular buffer */

      note_add(*driver, &note, sizeof(struct note_irqhandler_s));
    }
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
void sched_note_string(uintptr_t ip, FAR const char *buf)
{
  FAR struct note_string_s *note;
  uint8_t data[255];
  unsigned int length;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_string(*driver, ip, buf))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note = (FAR struct note_string_s *)data;
          length = SIZEOF_NOTE_STRING(strlen(buf));
          if (length > sizeof(data))
            {
              length = sizeof(data);
            }

          note_common(tcb, &note->nst_cmn, length, NOTE_DUMP_STRING);
          sched_note_flatten(note->nst_ip, &ip, sizeof(uintptr_t));
          memcpy(note->nst_data, buf, length - sizeof(struct note_string_s));
          data[length - 1] = '\0';
        }

      /* Add the note to circular buffer */

      note_add(*driver, note, length);
    }
}

void sched_note_dump(uintptr_t ip, uint8_t event,
                     FAR const void *buf, size_t len)
{
  FAR struct note_binary_s *note;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  char data[255];
  unsigned int length;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_dump(*driver, ip, buf, len))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note = (FAR struct note_binary_s *)data;
          length = SIZEOF_NOTE_BINARY(len);
          if (length > sizeof(data))
            {
              length = sizeof(data);
            }

          note_common(tcb, &note->nbi_cmn, length, NOTE_DUMP_BINARY);
          sched_note_flatten(note->nbi_ip, &ip, sizeof(uintptr_t));
          note->nbi_event = event;
          memcpy(note->nbi_data, buf,
                 length - sizeof(struct note_binary_s) + 1);
        }

      /* Add the note to circular buffer */

      note_add(*driver, note, length);
    }
}

void sched_note_vprintf(uintptr_t ip,
                        FAR const char *fmt, va_list va)
{
  FAR struct note_string_s *note;
  uint8_t data[255];
  unsigned int length;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_vprintf(*driver, ip, fmt, va))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note = (FAR struct note_string_s *)data;
          length = vsnprintf(note->nst_data,
                             sizeof(data) - sizeof(struct note_string_s),
                             fmt,
                             va);
          length = SIZEOF_NOTE_STRING(length);
          if (length > sizeof(data))
            {
              length = sizeof(data);
            }

          note_common(tcb, &note->nst_cmn, length, NOTE_DUMP_STRING);

          sched_note_flatten(note->nst_ip, &ip, sizeof(uintptr_t));
        }

      /* Add the note to circular buffer */

      note_add(*driver, note, length);
    }
}

void sched_note_vbprintf(uintptr_t ip, uint8_t event,
                         FAR const char *fmt, va_list va)
{
  FAR struct note_binary_s *note;
  FAR struct note_driver_s **driver;
  bool formatted = false;
  uint8_t data[255];
  begin_packed_struct union
    {
      char c;
      short s;
      int i;
      long l;
#ifdef CONFIG_HAVE_LONG_LONG
      long long ll;
#endif
      intmax_t im;
      size_t sz;
      ptrdiff_t ptr;
#ifdef CONFIG_HAVE_FLOAT
      float f;
#endif
#ifdef CONFIG_HAVE_DOUBLE
      double d;
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
      long double ld;
#endif
    }

  end_packed_struct *var;

  char c;
  int length;
  bool search_fmt = 0;
  int next = 0;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

  for (driver = g_note_drivers; *driver; driver++)
    {
      if (note_vbprintf(*driver, ip, event, fmt, va))
        {
          continue;
        }

      if ((*driver)->ops->add == NULL)
        {
          continue;
        }

      /* Format the note */

      if (!formatted)
        {
          formatted = true;
          note = (FAR struct note_binary_s *)data;
          length = sizeof(data) - sizeof(struct note_binary_s) + 1;

          while ((c = *fmt++) != '\0')
            {
              if (c != '%' && search_fmt == 0)
                {
                  continue;
                }

              search_fmt = 1;
              var = (FAR void *)&note->nbi_data[next];

              if (c == 'd' || c == 'i' || c == 'u' ||
                  c == 'o' || c == 'x' || c == 'X')
                {
                  if (*(fmt - 2) == 'h' && *(fmt - 3) == 'h')
                    {
                      if (next + sizeof(var->c) > length)
                        {
                          break;
                        }

                      var->c = va_arg(va, int);
                      next += sizeof(var->c);
                    }
                  else if (*(fmt - 2) == 'h')
                    {
                      if (next + sizeof(var->s) > length)
                        {
                          break;
                        }

                      var->s = va_arg(va, int);
                      next += sizeof(var->s);
                    }
                  else if (*(fmt - 2) == 'j')
                    {
                      if (next + sizeof(var->im) > length)
                        {
                          break;
                        }

                      var->im = va_arg(va, intmax_t);
                      next += sizeof(var->im);
                    }
#ifdef CONFIG_HAVE_LONG_LONG
                  else if (*(fmt - 2) == 'l' && *(fmt - 3) == 'l')
                    {
                      if (next + sizeof(var->ll) > length)
                        {
                          break;
                        }

                      var->ll = va_arg(va, long long);
                      next += sizeof(var->ll);
                    }
#endif
                  else if (*(fmt - 2) == 'l')
                    {
                      if (next + sizeof(var->l) > length)
                        {
                          break;
                        }

                      var->l = va_arg(va, long);
                      next += sizeof(var->l);
                    }
                  else if (*(fmt - 2) == 'z')
                    {
                      if (next + sizeof(var->sz) > length)
                        {
                          break;
                        }

                      var->sz = va_arg(va, size_t);
                      next += sizeof(var->sz);
                    }
                  else if (*(fmt - 2) == 't')
                    {
                      if (next + sizeof(var->ptr) > length)
                        {
                          break;
                        }

                      var->ptr = va_arg(va, ptrdiff_t);
                      next += sizeof(var->ptr);
                    }
                  else
                    {
                      if (next + sizeof(var->i) > length)
                        {
                          break;
                        }

                      var->i = va_arg(va, int);
                      next += sizeof(var->i);
                    }

                  search_fmt = 0;
                }

              if (c == 'e' || c == 'f' || c == 'g' ||
                  c == 'E' || c == 'F' || c == 'G')
                {
                  if (*(fmt - 2) == 'L')
                    {
#ifdef CONFIG_HAVE_LONG_DOUBLE
                      if (next + sizeof(var->ld) > length)
                        {
                          break;
                        }

                      var->ld = va_arg(va, long double);
                      next += sizeof(var->ld);
#endif
                    }
                  else if (*(fmt - 2) == 'l')
                    {
#ifdef CONFIG_HAVE_DOUBLE
                      if (next + sizeof(var->d) > length)
                        {
                          break;
                        }

                      var->d = va_arg(va, double);
                      next += sizeof(var->d);
#endif
                    }
                  else
#ifdef CONFIG_HAVE_FLOAT
                    {
                      if (next + sizeof(var->l) > length)
                        {
                          break;
                        }

                      var->l = va_arg(va, double);
                      next += sizeof(var->l);
#endif
                    }

                  search_fmt = 0;
                }
            }

          length = SIZEOF_NOTE_BINARY(next);

          note_common(tcb, &note->nbi_cmn, length, NOTE_DUMP_BINARY);
          sched_note_flatten(note->nbi_ip, &ip, sizeof(uintptr_t));
          note->nbi_event = event;
        }

      /* Add the note to circular buffer */

      note_add(*driver, note, length);
    }
}

void sched_note_printf(uintptr_t ip,
                       FAR const char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  sched_note_vprintf(ip, fmt, va);
  va_end(va);
}

void sched_note_bprintf(uintptr_t ip, uint8_t event,
                        FAR const char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  sched_note_vbprintf(ip, event, fmt, va);
  va_end(va);
}
#endif /* CONFIG_SCHED_INSTRUMENTATION_DUMP */

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

void sched_note_filter_mode(FAR struct note_filter_mode_s *oldm,
                            FAR struct note_filter_mode_s *newm)
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
void sched_note_filter_syscall(FAR struct note_filter_syscall_s *oldf,
                               FAR struct note_filter_syscall_s *newf)
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
void sched_note_filter_irq(FAR struct note_filter_irq_s *oldf,
                           FAR struct note_filter_irq_s *newf)
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

#if CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0

/****************************************************************************
 * Name: note_get_taskname
 *
 * Description:
 *   Get the task name string of the specified PID
 *
 * Input Parameters:
 *   PID - Task ID
 *   name - Task name buffer
 *          this buffer must be greater than CONFIG_TASK_NAME_SIZE + 1
 *
 * Returned Value:
 *   Retrun OK if task name can be retrieved, otherwise -ESRCH
 *
 ****************************************************************************/

int note_get_taskname(pid_t pid, FAR char *buffer)
{
  FAR struct note_taskname_info_s *ti;
  FAR struct tcb_s *tcb;
  irqstate_t irq_mask;

  irq_mask = enter_critical_section();
  tcb = nxsched_get_tcb(pid);
  if (tcb != NULL)
    {
      strlcpy(buffer, tcb->name, CONFIG_TASK_NAME_SIZE + 1);
      leave_critical_section(irq_mask);
      return OK;
    }

  ti = note_find_taskname(pid);
  if (ti != NULL)
    {
      strlcpy(buffer, ti->name, CONFIG_TASK_NAME_SIZE + 1);
      leave_critical_section(irq_mask);
      return OK;
    }

  leave_critical_section(irq_mask);
  return -ESRCH;
}

#endif
