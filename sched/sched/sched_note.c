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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

/****************************************************************************
 * Public Functions
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

void note_common(FAR struct tcb_s *tcb,
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
 * Name: sched_note_flatten
 *
 * Description:
 *   Copy the data in the little endian layout
 *
 ****************************************************************************/

void sched_note_flatten(FAR uint8_t *dst,
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
  if (!note_isenabled())
    {
      return;
    }

    sched_ramnote_start(tcb);
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  if (!note_isenabled())
    {
      return;
    }
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH

void sched_note_suspend(FAR struct tcb_s *tcb)
{
  if (!note_isenabled_switch())
    {
      return;
    }

  sched_ramnote_suspend(tcb);
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
  if (!note_isenabled_switch())
    {
      return;
    }

  sched_ramnote_resume(tcb);
}
#endif

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_cpu_start(tcb, cpu);
}

void sched_note_cpu_started(FAR struct tcb_s *tcb)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_cpu_started(tcb);
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
  if (!note_isenabled_switch())
    {
      return;
    }

  sched_ramnote_cpu_pause(tcb, cpu);
}

void sched_note_cpu_paused(FAR struct tcb_s *tcb)
{
  if (!note_isenabled_switch())
    {
      return;
    }

  sched_ramnote_cpu_paused(tcb);
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
  if (!note_isenabled_switch())
    {
      return;
    }

  sched_ramnote_cpu_resume(tcb, cpu);
}

void sched_note_cpu_resumed(FAR struct tcb_s *tcb)
{
  if (!note_isenabled_switch())
    {
      return;
    }

  sched_ramnote_cpu_resumed(tcb);
}
#endif
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_note_premption(FAR struct tcb_s *tcb, bool locked)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_premption(tcb, locked);
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

  sched_ramnote_csection(tcb, enter);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_note_spinlock(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_spinlock(tcb, spinlock);
}

void sched_note_spinlocked(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_spinlocked(tcb, spinlock);
}

void sched_note_spinunlock(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_spinunlock(tcb, spinlock);
}

void sched_note_spinabort(FAR struct tcb_s *tcb, FAR volatile void *spinlock)
{
  if (!note_isenabled())
    {
      return;
    }

  sched_ramnote_spinabort(tcb, spinlock);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr, int argc, ...)
{
  va_list ap;

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
  sched_ramnote_syscall_enter(nr, argc, ap);
  va_end(ap);
}

void sched_note_syscall_leave(int nr, uintptr_t result)
{
  if (!note_isenabled_syscall(nr))
    {
      return;
    }

  sched_ramnote_syscall_leave(nr, result);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  if (!note_isenabled_irq(irq, enter))
    {
      return;
    }

  sched_ramnote_irqhandler(irq, handler, enter);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
void sched_note_string(uintptr_t ip, FAR const char *buf)
{
  FAR struct note_string_s *note;
  uint8_t data[255];
  unsigned int length;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

  note = (FAR struct note_string_s *)data;
  length = SIZEOF_NOTE_STRING(strlen(buf));
  if (length > sizeof(data))
    {
      length = sizeof(data);
    }

  /* Format the note */

  note_common(tcb, &note->nst_cmn, length,
              NOTE_DUMP_STRING);

  sched_note_flatten(note->nst_ip, &ip, sizeof(uintptr_t));
  memcpy(note->nst_data, buf, length - sizeof(struct note_string_s));
  data[length - 1] = '\0';

  /* Add the note to circular buffer */

  sched_ramnote_write(note, length);
}

void sched_note_dump(uintptr_t ip, uint8_t event,
                                 FAR const void *buf, size_t len)
{
  FAR struct note_binary_s *note;
  char data[255];
  unsigned int length;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

  note = (FAR struct note_binary_s *)data;
  length = SIZEOF_NOTE_BINARY(len);
  if (length > sizeof(data))
    {
      length = sizeof(data);
    }

  /* Format the note */

  note_common(tcb, &note->nbi_cmn, length,
              NOTE_DUMP_BINARY);

  sched_note_flatten(note->nbi_ip, &ip, sizeof(uintptr_t));
  note->nbi_event = event;
  memcpy(note->nbi_data, buf, length - sizeof(struct note_binary_s) + 1);

  /* Add the note to circular buffer */

  sched_ramnote_write(note, length);
}

void sched_note_vprintf(uintptr_t ip,
                        FAR const char *fmt, va_list va)
{
  FAR struct note_string_s *note;
  uint8_t data[255];
  unsigned int length;
  FAR struct tcb_s *tcb = this_task();

  if (!note_isenabled_dump())
    {
      return;
    }

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

  /* Format the note */

  note_common(tcb, &note->nst_cmn, length,
              NOTE_DUMP_STRING);

  sched_note_flatten(note->nst_ip, &ip, sizeof(uintptr_t));

  /* Add the note to circular buffer */

  sched_ramnote_write(note, length);
}

void sched_note_vbprintf(uintptr_t ip, uint8_t event,
                         FAR const char *fmt, va_list va)
{
  FAR struct note_binary_s *note;
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

  /* Format the note */

  note_common(tcb, &note->nbi_cmn, length,
              NOTE_DUMP_BINARY);

  sched_note_flatten(note->nbi_ip, &ip, sizeof(uintptr_t));
  note->nbi_event = event;

  /* Add the note to circular buffer */

  sched_ramnote_write(note, length);
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

void sched_note_begin(uintptr_t ip)
{
  sched_note_string(ip, "B");
}

void sched_note_end(uintptr_t ip)
{
  sched_note_string(ip, "E");
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
