/****************************************************************************
 * sched/misc/assert.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/tls.h>

#include <assert.h>
#include <debug.h>
#include <stdlib.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BOARD_RESET_ON_ASSERT
#  define CONFIG_BOARD_RESET_ON_ASSERT 0
#endif

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/* USB trace dumping */

#ifndef CONFIG_USBDEV_TRACE
#  undef CONFIG_ARCH_USBDUMP
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(FAR const char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  vsyslog(LOG_EMERG, fmt, ap);
  va_end(ap);
  return OK;
}

static int assert_tracecallback(FAR struct usbtrace_s *trace, FAR void *arg)
{
  usbtrace_trprintf(usbtrace_syslog, trace->event, trace->value);
  return 0;
}
#endif

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Name: stackdump
 ****************************************************************************/

static void stackdump(uintptr_t sp, uintptr_t stack_top)
{
  uintptr_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      FAR uint32_t *ptr = (FAR uint32_t *)stack;
      _alert("%" PRIxPTR ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 "\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: dump_stack
 ****************************************************************************/

static void dump_stack(FAR const char *tag, uintptr_t sp,
                       uintptr_t base, size_t size,
                       size_t used, bool force)
{
  uintptr_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("sp:     %08" PRIxPTR "\n", sp);
  _alert("  base: %08" PRIxPTR "\n", base);
  _alert("  size: %08zu\n", size);

  if (sp >= base && sp < top)
    {
      stackdump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
#ifdef CONFIG_STACK_COLORATION
          size_t remain = size - used;

          base  += remain;
          size  -= remain;
#endif

#if CONFIG_ARCH_STACKDUMP_MAX_LENGTH > 0
          if (size > CONFIG_ARCH_STACKDUMP_MAX_LENGTH)
            {
              size = CONFIG_ARCH_STACKDUMP_MAX_LENGTH;
            }
#endif

          stackdump(base, base + size);
        }
    }
}

/****************************************************************************
 * Name: showstacks
 ****************************************************************************/

static void showstacks(void)
{
  FAR struct tcb_s *rtcb = running_task();
  uintptr_t sp = up_getsp();

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  dump_stack("IRQ", sp,
             up_get_intstackbase(),
             CONFIG_ARCH_INTERRUPTSTACK,
#  ifdef CONFIG_STACK_COLORATION
             up_check_intstack(),
#  else
             0,
#  endif
             up_interrupt_context());
  if (up_interrupt_context())
    {
      sp = up_getusrsp();
    }
#endif

  dump_stack("User", sp,
             (uintptr_t)rtcb->stack_base_ptr,
             (size_t)rtcb->adj_stack_size,
#ifdef CONFIG_STACK_COLORATION
             up_check_tcbstack(rtcb),
#else
             0,
#endif
#ifdef CONFIG_ARCH_KERNEL_STACK
             false
#else
             true
#endif
            );

#ifdef CONFIG_ARCH_KERNEL_STACK
  dump_stack("Kernel", sp,
             (uintptr_t)rtcb->xcp.kstack,
             CONFIG_ARCH_KERNEL_STACKSIZE,
#  ifdef CONFIG_STACK_COLORATION
             up_check_tcbstack(rtcb),
#  else
             0,
#  endif
             false);
#endif
}

#endif

/****************************************************************************
 * Name: dump_task
 ****************************************************************************/

static void dump_task(struct tcb_s *tcb, void *arg)
{
  char args[64] = "";
#ifdef CONFIG_STACK_COLORATION
  size_t stack_filled = 0;
  size_t stack_used;
#endif
#ifdef CONFIG_SCHED_CPULOAD
  struct cpuload_s cpuload;
  size_t fracpart = 0;
  size_t intpart = 0;
  size_t tmp;

  clock_cpuload(tcb->pid, &cpuload);

  if (cpuload.total > 0)
    {
      tmp      = (1000 * cpuload.active) / cpuload.total;
      intpart  = tmp / 10;
      fracpart = tmp - 10 * intpart;
    }
#endif

#ifdef CONFIG_STACK_COLORATION
  stack_used = up_check_tcbstack(tcb);
  if (tcb->adj_stack_size > 0 && stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 * stack_used / tcb->adj_stack_size;
    }
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      struct pthread_tcb_s *ptcb = (struct pthread_tcb_s *)tcb;

      snprintf(args, sizeof(args), " %p %p",
               ptcb->cmn.entry.main, ptcb->arg);
    }
  else
#endif
    {
      char **argv = tcb->group->tg_info->argv + 1;
      size_t npos = 0;

      while (*argv != NULL && npos < sizeof(args))
        {
          npos += snprintf(args + npos, sizeof(args) - npos, " %s", *argv++);
        }
    }

  /* Dump interesting properties of this task */

  _alert("  %4d   %4d"
#ifdef CONFIG_SMP
         "  %4d"
#endif
         "   %7zu"
#ifdef CONFIG_STACK_COLORATION
         "   %7zu   %3zu.%1zu%%%c"
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "   %3zu.%01zu%%"
#endif
         "   %s%s\n"
         , tcb->pid, tcb->sched_priority
#ifdef CONFIG_SMP
         , tcb->cpu
#endif
         , tcb->adj_stack_size
#ifdef CONFIG_STACK_COLORATION
         , up_check_tcbstack(tcb)
         , stack_filled / 10, stack_filled % 10
         , (stack_filled >= 10 * 80 ? '!' : ' ')
#endif
#ifdef CONFIG_SCHED_CPULOAD
         , intpart, fracpart
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         , tcb->name
#else
         , "<noname>"
#endif
         , args
        );
}

/****************************************************************************
 * Name: dump_backtrace
 ****************************************************************************/

#ifdef CONFIG_SCHED_BACKTRACE
static void dump_backtrace(FAR struct tcb_s *tcb, FAR void *arg)
{
  /* Show back trace */

  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: showtasks
 ****************************************************************************/

static void showtasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 0
#  ifdef CONFIG_STACK_COLORATION
  size_t stack_used = up_check_intstack();
  size_t stack_filled = 0;

  if (stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 *
                     stack_used / CONFIG_ARCH_INTERRUPTSTACK;
    }
#  endif
#endif

  /* Dump interesting properties of each task in the crash environment */

  _alert("   PID    PRI"
#ifdef CONFIG_SMP
         "   CPU"
#endif
         "     STACK"
#ifdef CONFIG_STACK_COLORATION
         "      USED   FILLED "
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "      CPU"
#endif
         "   COMMAND\n");

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  _alert("  ----   ----"
#  ifdef CONFIG_SMP
         "  ----"
#  endif
         "   %7u"
#  ifdef CONFIG_STACK_COLORATION
         "   %7zu   %3zu.%1zu%%%c"
#  endif
#  ifdef CONFIG_SCHED_CPULOAD
         "     ----"
#  endif
         "   irq\n"
         , CONFIG_ARCH_INTERRUPTSTACK
#  ifdef CONFIG_STACK_COLORATION
         , stack_used
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
#  endif
        );
#endif

  nxsched_foreach(dump_task, NULL);

#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(dump_backtrace, NULL);
#endif
}

/****************************************************************************
 * Name: assert_end
 ****************************************************************************/

static void assert_end(void)
{
  /* Flush any buffered SYSLOG data */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (up_interrupt_context() || running_task()->flink == NULL)
    {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif

      /* Disable interrupts on this CPU */

      up_irq_save();

#ifdef CONFIG_SMP
      /* Try (again) to stop activity on other CPUs */

      spin_trylock(&g_cpu_irqlock);
#endif

      for (; ; )
        {
          up_mdelay(250);
        }
    }
  else
    {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 2
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void _assert(FAR const char *filename, int linenum)
{
  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#if CONFIG_BOARD_RESET_ON_ASSERT < 2
  if (!up_interrupt_context() && running_task()->flink != NULL)
    {
      panic_notifier_call_chain(PANIC_TASK, NULL);
    }
  else
#endif
    {
      panic_notifier_call_chain(PANIC_KERNEL, NULL);
    }

#ifdef CONFIG_SMP
#  if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed CPU%d at file: %s:%d task: %s %p\n",
         up_cpu_index(), filename, linenum, running_task()->name,
         running_task()->entry.main);
#  else
  _alert("Assertion failed CPU%d at file: %s:%d\n",
         up_cpu_index(), filename, linenum);
#  endif
#else
#  if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file: %s:%d task: %s %p\n",
         filename, linenum, running_task()->name,
         running_task()->entry.main);
#  else
  _alert("Assertion failed at file: %s:%d\n",
         filename, linenum);
#  endif
#endif

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif

#ifdef CONFIG_BOARD_CRASHDUMP
  board_crashdump(up_getsp(), running_task(), filename, linenum);
#endif

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(running_task()->pid);
#endif

  up_assert();

#ifdef CONFIG_ARCH_STACKDUMP
  showstacks();
#endif

  showtasks();

  assert_end();
  exit(EXIT_FAILURE);
}
