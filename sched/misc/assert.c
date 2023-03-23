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
#include <nuttx/tls.h>
#include <nuttx/signal.h>

#include <nuttx/panic_notifier.h>
#include <nuttx/reboot_notifier.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>

#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/utsname.h>

#include "irq/irq.h"
#include "sched/sched.h"
#include "group/group.h"

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
 * Private Data
 ****************************************************************************/

static uint8_t g_last_regs[XCPTCONTEXT_SIZE];

static FAR const char *g_policy[4] =
{
  "FIFO", "RR", "SPORADIC"
};

static FAR const char * const g_ttypenames[4] =
{
  "Task",
  "pthread",
  "Kthread",
  "Invalid"
};

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
 * Name: stack_dump
 ****************************************************************************/

static void stack_dump(uintptr_t sp, uintptr_t stack_top)
{
  uintptr_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      FAR uint32_t *ptr = (FAR uint32_t *)stack;
      _alert("%p: %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 "\n",
             (FAR void *)stack, ptr[0], ptr[1], ptr[2], ptr[3],
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
  _alert("sp:     %p\n", (FAR void *)sp);
  _alert("  base: %p\n", (FAR void *)base);
  _alert("  size: %08zu\n", size);

  if (sp >= base && sp < top)
    {
      stack_dump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
#ifdef CONFIG_STACK_COLORATION
          size_t remain = size - used;

          base += remain;
          size -= remain;
#endif

#if CONFIG_ARCH_STACKDUMP_MAX_LENGTH > 0
          if (size > CONFIG_ARCH_STACKDUMP_MAX_LENGTH)
            {
              size = CONFIG_ARCH_STACKDUMP_MAX_LENGTH;
            }
#endif

          stack_dump(base, base + size);
        }
    }
}

/****************************************************************************
 * Name: showstacks
 ****************************************************************************/

static void show_stacks(FAR struct tcb_s *rtcb)
{
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
             0, false);
#endif
}

#endif

/****************************************************************************
 * Name: dump_task
 ****************************************************************************/

static void dump_task(FAR struct tcb_s *tcb, FAR void *arg)
{
  char args[64] = "";
  char state[32];
  FAR char *s;
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

  /* Stringify the argument vector */

  group_argvstr(tcb, args, sizeof(args));

  /* get the task_state */

  nxsched_get_stateinfo(tcb, state, sizeof(state));
  if ((s = strchr(state, ',')) != NULL)
    {
      *s = ' ';
    }

  /* Dump interesting properties of this task */

  _alert("   %4d %5d"
#ifdef CONFIG_SMP
         "  %4d"
#endif
         " %3d %-8s %-7s %c%c%c"
         " %-18s"
         " " SIGSET_FMT
         " %p"
         "   %7zu"
#ifdef CONFIG_STACK_COLORATION
         "   %7zu   %3zu.%1zu%%%c"
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "   %3zu.%01zu%%"
#endif
         "   %s%s\n"
         , tcb->pid
         , tcb->group ? tcb->group->tg_pid : -1
#ifdef CONFIG_SMP
         , tcb->cpu
#endif
         , tcb->sched_priority
         , g_policy[(tcb->flags & TCB_FLAG_POLICY_MASK) >>
                    TCB_FLAG_POLICY_SHIFT]
         , g_ttypenames[(tcb->flags & TCB_FLAG_TTYPE_MASK)
                        >> TCB_FLAG_TTYPE_SHIFT]
         , tcb->flags & TCB_FLAG_NONCANCELABLE ? 'N' : '-'
         , tcb->flags & TCB_FLAG_CANCEL_PENDING ? 'P' : '-'
         , tcb->flags & TCB_FLAG_EXIT_PROCESSING ? 'P' : '-'
         , state
         , SIGSET_ELEM(&tcb->sigprocmask)
         , tcb->stack_base_ptr
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
  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: showtasks
 ****************************************************************************/

static void show_tasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 0 && defined(CONFIG_STACK_COLORATION)
  size_t stack_used = up_check_intstack();
  size_t stack_filled = 0;

  if (stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 *
                     stack_used / CONFIG_ARCH_INTERRUPTSTACK;
    }
#endif

  /* Dump interesting properties of each task in the crash environment */

  _alert("   PID GROUP"
#ifdef CONFIG_SMP
         "   CPU"
#endif
         " PRI POLICY   TYPE    NPX"
         " STATE   EVENT"
         "      SIGMASK"
         "  STACKBASE"
         "  STACKSIZE"
#ifdef CONFIG_STACK_COLORATION
         "      USED   FILLED "
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "      CPU"
#endif
         "   COMMAND\n");

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  _alert("  ----   ---"
#  ifdef CONFIG_SMP
         "  ----"
#  endif
         " --- --------"
         " ------- ---"
         " ------- ----------"
         " --------"
         " %p"
         "   %7u"
#  ifdef CONFIG_STACK_COLORATION
         "   %7zu   %3zu.%1zu%%%c"
#  endif
#  ifdef CONFIG_SCHED_CPULOAD
         "     ----"
#  endif
         "   irq\n"
         , (FAR void *)up_get_intstackbase()
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
 * Public Functions
 ****************************************************************************/

void _assert(FAR const char *filename, int linenum, FAR const char *msg)
{
  FAR struct tcb_s *rtcb = running_task();
  struct utsname name;
  bool fatal = false;

  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#if CONFIG_BOARD_RESET_ON_ASSERT < 2
  if (up_interrupt_context() ||
      (rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      fatal = true;
    }
#else
  fatal = true;
#endif

  panic_notifier_call_chain(fatal ? PANIC_KERNEL : PANIC_TASK, rtcb);

  uname(&name);
  _alert("Current Version: %s %s %s %s %s\n",
         name.sysname, name.nodename,
         name.release, name.version, name.machine);

  _alert("Assertion failed %s: at file: %s:%d task"
#ifdef CONFIG_SMP
         "(CPU%d)"
#endif
         ": "
#if CONFIG_TASK_NAME_SIZE > 0
         "%s "
#endif
         "%p\n",
         msg ? msg : "",
         filename ? filename : "", linenum,
#ifdef CONFIG_SMP
         up_cpu_index(),
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         rtcb->name,
#endif
         rtcb->entry.main);

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif

  /* Register dump */

  if (up_interrupt_context())
    {
      up_dump_register(NULL);
    }
  else
    {
      up_saveusercontext(g_last_regs);
      up_dump_register(g_last_regs);
    }

#ifdef CONFIG_ARCH_STACKDUMP
  show_stacks(rtcb);
#endif

  /* Flush any buffered SYSLOG data */

  syslog_flush();

  if (fatal)
    {
      show_tasks();

#ifdef CONFIG_ARCH_USBDUMP
      /* Dump USB trace data */

      usbtrace_enumerate(assert_tracecallback, NULL);
#endif

#ifdef CONFIG_BOARD_CRASHDUMP
      board_crashdump(up_getsp(), rtcb, filename, linenum, msg);
#endif

      /* Flush any buffered SYSLOG data */

      syslog_flush();
      panic_notifier_call_chain(PANIC_KERNEL_FINAL, rtcb);

      reboot_notifier_call_chain(SYS_HALT, NULL);

#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#else
      /* Disable interrupts on this CPU */

      up_irq_save();

#  ifdef CONFIG_SMP
      /* Try (again) to stop activity on other CPUs */

      spin_trylock(&g_cpu_irqlock);
#  endif

      for (; ; )
        {
          up_mdelay(250);
        }
#endif
    }
}
