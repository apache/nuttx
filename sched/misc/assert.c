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
#include <nuttx/binfmt/binfmt.h>
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

#define DEADLOCK_MAX 8

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

static uint8_t g_last_regs[XCPTCONTEXT_SIZE] aligned_data(16);

#ifdef CONFIG_BOARD_COREDUMP
static struct lib_syslogstream_s  g_syslogstream;
static struct lib_hexdumpstream_s g_hexstream;
#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
static struct lib_lzfoutstream_s  g_lzfstream;
#  endif
#endif

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
                       uintptr_t base, size_t size, size_t used)
{
  uintptr_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("  base: %p\n", (FAR void *)base);
  _alert("  size: %08zu\n", size);

  if (sp != 0)
    {
      _alert("    sp: %p\n", (FAR void *)sp);
      stack_dump(sp, top);
    }
  else
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

/****************************************************************************
 * Name: dump_stacks
 ****************************************************************************/

static void dump_stacks(FAR struct tcb_s *rtcb, uintptr_t sp)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 0
  uintptr_t intstack_base = up_get_intstackbase();
  size_t intstack_size = CONFIG_ARCH_INTERRUPTSTACK;
  uintptr_t intstack_top = intstack_base + intstack_size;
  uintptr_t intstack_sp = 0;
#endif
#ifdef CONFIG_ARCH_KERNEL_STACK
  uintptr_t kernelstack_base = (uintptr_t)rtcb->xcp.kstack;
  size_t kernelstack_size = CONFIG_ARCH_KERNEL_STACKSIZE;
  uintptr_t kernelstack_top = kernelstack_base + kernelstack_size;
  uintptr_t kernelstack_sp = 0;
#endif
  uintptr_t tcbstack_base = (uintptr_t)rtcb->stack_base_ptr;
  size_t tcbstack_size = (size_t)rtcb->adj_stack_size;
  uintptr_t tcbstack_top = tcbstack_base + tcbstack_size;
  uintptr_t tcbstack_sp = 0;
  bool force = false;

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  if (sp >= intstack_base && sp < intstack_top)
    {
      intstack_sp = sp;
    }
  else
#endif
#ifdef CONFIG_ARCH_KERNEL_STACK
  if (sp >= kernelstack_base && sp < kernelstack_top)
    {
      kernelstack_sp = sp;
    }
  else
#endif
  if (sp >= tcbstack_base && sp < tcbstack_top)
    {
      tcbstack_sp = sp;
    }
  else
    {
      force = true;
      _alert("ERROR: Stack pointer is not within the stack\n");
    }

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  if (intstack_sp != 0 || force)
    {
      dump_stack("IRQ",
                 intstack_sp,
                 intstack_base,
                 intstack_size,
#ifdef CONFIG_STACK_COLORATION
                 up_check_intstack());
#else
                 0);
#endif

      tcbstack_sp = up_getusrsp((FAR void *)CURRENT_REGS);
      if (tcbstack_sp < tcbstack_base || tcbstack_sp >= tcbstack_top)
        {
          tcbstack_sp = 0;
          force = true;
        }
    }
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  if (kernelstack_sp != 0 || force)
    {
      dump_stack("Kernel",
                 kernelstack_sp,
                 kernelstack_base,
                 kernelstack_size,
                 0
                );
    }
#endif

  if (tcbstack_sp != 0 || force)
    {
      dump_stack("User",
                 tcbstack_sp,
                 tcbstack_base,
                 tcbstack_size,
#ifdef CONFIG_STACK_COLORATION
                 up_check_tcbstack(rtcb));
#else
                 0);
#endif
    }
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
 * Name: dump_tasks
 ****************************************************************************/

static void dump_tasks(void)
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
         "      SIGMASK        "
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
 * Name: dump_core
 ****************************************************************************/

#ifdef CONFIG_BOARD_COREDUMP
static void dump_core(pid_t pid)
{
  FAR void *stream;
  int logmask;

  logmask = setlogmask(LOG_ALERT);

  _alert("Start coredump:\n");

  /* Initialize hex output stream */

  lib_syslogstream(&g_syslogstream, LOG_EMERG);

  stream = &g_syslogstream;

  lib_hexdumpstream(&g_hexstream, stream);

  stream = &g_hexstream;

#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION

  /* Initialize LZF compression stream */

  lib_lzfoutstream(&g_lzfstream, stream);
  stream = &g_lzfstream;

#  endif

  /* Do core dump */

  core_dump(NULL, stream, pid);

#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
  _alert("Finish coredump (Compression Enabled).\n");
#  else
  _alert("Finish coredump.\n");
#  endif

  setlogmask(logmask);
}
#endif

/****************************************************************************
 * Name: dump_deadlock
 ****************************************************************************/

#ifdef CONFIG_ARCH_DEADLOCKDUMP
static void dump_deadlock(void)
{
  pid_t deadlock[DEADLOCK_MAX];
  size_t i = nxsched_collect_deadlock(deadlock, DEADLOCK_MAX);

  if (i > 0)
    {
      _alert("Deadlock detected\n");
      while (i-- > 0)
        {
#ifdef CONFIG_SCHED_BACKTRACE
          sched_dumpstack(deadlock[i]);
#else
          _alert("deadlock pid: %d\n", deadlock[i]);
#endif
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _assert
 ****************************************************************************/

void _assert(FAR const char *filename, int linenum,
             FAR const char *msg, FAR void *regs)
{
  FAR struct tcb_s *rtcb = running_task();
  struct utsname name;
  bool fatal = true;

  /* try to save current context if regs is null */

  if (regs == NULL)
    {
      up_saveusercontext(g_last_regs);
      regs = g_last_regs;
    }

  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#if CONFIG_BOARD_RESET_ON_ASSERT < 2
  if (!up_interrupt_context() &&
      (rtcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      fatal = false;
    }
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

  up_dump_register(regs);

#ifdef CONFIG_ARCH_STACKDUMP
  dump_stacks(rtcb, up_getusrsp(regs));
#endif

  /* Flush any buffered SYSLOG data */

  syslog_flush();

  if (fatal)
    {
      dump_tasks();

#ifdef CONFIG_ARCH_DEADLOCKDUMP
      /* Deadlock Dump */

      dump_deadlock();
#endif

#ifdef CONFIG_ARCH_USBDUMP
      /* Dump USB trace data */

      usbtrace_enumerate(assert_tracecallback, NULL);
#endif

#ifdef CONFIG_BOARD_CRASHDUMP
      board_crashdump(up_getsp(), rtcb, filename, linenum, msg, regs);

#elif defined(CONFIG_BOARD_COREDUMP)
      /* Dump core information */

#  ifdef CONFIG_BOARD_COREDUMP_FULL
      dump_core(INVALID_PROCESS_ID);
#  else
      dump_core(rtcb->pid);
#  endif

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
