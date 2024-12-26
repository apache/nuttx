/****************************************************************************
 * sched/misc/assert.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/cache.h>
#include <nuttx/coredump.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/init.h>
#include <nuttx/tls.h>
#include <nuttx/signal.h>
#include <nuttx/sched.h>
#ifdef CONFIG_ARCH_LEDS
#  include <arch/board/board.h>
#endif
#include <nuttx/panic_notifier.h>
#include <nuttx/reboot_notifier.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/mm/kasan.h>

#include <assert.h>
#include <debug.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/utsname.h>

#include "irq/irq.h"
#include "sched/sched.h"
#include "group/group.h"
#include "coredump.h"

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

#define DUMP_PTR(p, x) ((uintptr_t)(&(p)[(x)]) < stack_top ? (p)[(x)] : 0)
#define DUMP_STRIDE    (sizeof(FAR void *) * 8)

#if UINTPTR_MAX <= UINT32_MAX
#  define DUMP_FORMAT " %08" PRIxPTR ""
#elif UINTPTR_MAX <= UINT64_MAX
#  define DUMP_FORMAT " %016" PRIxPTR ""
#endif

/* Architecture can overwrite the default XCPTCONTEXT alignment */

#ifndef XCPTCONTEXT_ALIGN
#  define XCPTCONTEXT_ALIGN 16
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_SMP
static noreturn_function int pause_cpu_handler(FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SMP
static bool g_cpu_paused[CONFIG_SMP_NCPUS];
#endif

static spinlock_t g_assert_lock = SP_UNLOCKED;

static uintptr_t g_last_regs[CONFIG_SMP_NCPUS][XCPTCONTEXT_REGS]
                 aligned_data(XCPTCONTEXT_ALIGN);

#ifdef CONFIG_DEBUG_ALERT
static FAR const char * const g_policy[4] =
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
#endif

#ifdef CONFIG_SMP
static struct smp_call_data_s g_call_data =
SMP_CALL_INITIALIZER(pause_cpu_handler, NULL);
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
 * Name: stack_dump
 ****************************************************************************/

static void stack_dump(uintptr_t sp, uintptr_t stack_top)
{
  uintptr_t stack;

  for (stack = sp; stack <= stack_top; stack += DUMP_STRIDE)
    {
      FAR uintptr_t *ptr = (FAR uintptr_t *)stack;

      _alert("%p:"DUMP_FORMAT DUMP_FORMAT DUMP_FORMAT DUMP_FORMAT
             DUMP_FORMAT DUMP_FORMAT DUMP_FORMAT DUMP_FORMAT "\n",
             (FAR void *)stack, DUMP_PTR(ptr, 0), DUMP_PTR(ptr , 1),
             DUMP_PTR(ptr, 2), DUMP_PTR(ptr, 3), DUMP_PTR(ptr, 4),
             DUMP_PTR(ptr, 5), DUMP_PTR(ptr , 6), DUMP_PTR(ptr, 7));
    }
}

/****************************************************************************
 * Name: dump_stackinfo
 ****************************************************************************/

static void dump_stackinfo(FAR const char *tag, uintptr_t sp,
                           uintptr_t base, size_t size, size_t used)
{
  uintptr_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("  base: %p\n", (FAR void *)base);
  _alert("  size: %08zu\n", size);

  if (sp != 0)
    {
      _alert("    sp: %p\n", (FAR void *)sp);

      /* Get more information */

      if (sp - DUMP_STRIDE >= base)
        {
          sp -= DUMP_STRIDE;
        }

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
#ifdef CONFIG_SMP
  int cpu = rtcb->cpu;
#else
  int cpu = this_cpu();
  UNUSED(cpu);
#endif
#if CONFIG_ARCH_INTERRUPTSTACK > 0
  uintptr_t intstack_base = up_get_intstackbase(cpu);
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
      dump_stackinfo("IRQ",
                     intstack_sp,
                     intstack_base,
                     intstack_size,
#ifdef CONFIG_STACK_COLORATION
                     up_check_intstack(cpu)
#else
                     0
#endif
                     );

      /* Try to restore SP from current_regs if assert from interrupt. */

      tcbstack_sp = up_interrupt_context() ?
                    up_getusrsp((FAR void *)running_regs()) : 0;
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
      dump_stackinfo("Kernel",
                     kernelstack_sp,
                     kernelstack_base,
                     kernelstack_size,
                     0);
    }
#endif

  if (tcbstack_sp != 0 || force)
    {
      dump_stackinfo("User",
                     tcbstack_sp,
                     tcbstack_base,
                     tcbstack_size,
#ifdef CONFIG_STACK_COLORATION
                     up_check_tcbstack(rtcb)
#else
                     0
#endif
                     );
    }
}

#endif

#ifdef CONFIG_DEBUG_ALERT
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
#ifndef CONFIG_SCHED_CPULOAD_NONE
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

  nxtask_argvstr(tcb, args, sizeof(args));

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
         " %3d %-8s %-7s %-3c"
         " %-18s"
         " " SIGSET_FMT
         " %p"
         "   %7zu"
#ifdef CONFIG_STACK_COLORATION
         "   %7zu   %3zu.%1zu%%%c"
#endif
#ifndef CONFIG_SCHED_CPULOAD_NONE
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
#ifndef CONFIG_SCHED_CPULOAD_NONE
         , intpart, fracpart
#endif
         , get_task_name(tcb)
         , args
        );
}
#endif

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
 * Name: dump_filelist
 ****************************************************************************/

#ifdef CONFIG_SCHED_DUMP_ON_EXIT
static void dump_filelist(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct filelist *filelist = &tcb->group->tg_filelist;
  files_dumplist(filelist);
}
#endif

/****************************************************************************
 * Name: dump_tasks
 ****************************************************************************/

static void dump_tasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 0
  int cpu;
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
#ifndef CONFIG_SCHED_CPULOAD_NONE
         "      CPU"
#endif
         "   COMMAND\n");

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
#  ifdef CONFIG_STACK_COLORATION
      size_t stack_used = up_check_intstack(cpu);
      size_t stack_filled = 0;

      if (stack_used > 0)
        {
          /* Use fixed-point math with one decimal place */

          stack_filled = 10 * 100 *
                         stack_used / CONFIG_ARCH_INTERRUPTSTACK;
        }
#  endif

      _alert("  ----   ---"
#  ifdef CONFIG_SMP
             "  %4d"
#  endif
             " --- --------"
             " ------- ---"
             " ------- ----------"
             " ----------------"
             " %p"
             "   %7u"
#  ifdef CONFIG_STACK_COLORATION
             "   %7zu   %3zu.%1zu%%%c"
#  endif
#  ifndef CONFIG_SCHED_CPULOAD_NONE
             "     ----"
#  endif
             "   irq\n"
#ifdef CONFIG_SMP
             , cpu
#endif
             , (FAR void *)up_get_intstackbase(cpu)
             , CONFIG_ARCH_INTERRUPTSTACK
#  ifdef CONFIG_STACK_COLORATION
             , stack_used
             , stack_filled / 10, stack_filled % 10,
             (stack_filled >= 10 * 80 ? '!' : ' ')
#  endif
            );
    }
#endif

#ifdef CONFIG_DEBUG_ALERT
  nxsched_foreach(dump_task, NULL);
#endif

#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(dump_backtrace, NULL);
#endif

#ifdef CONFIG_SCHED_DUMP_ON_EXIT
  nxsched_foreach(dump_filelist, NULL);
#endif
}

/****************************************************************************
 * Name: dump_lockholder
 ****************************************************************************/

#if CONFIG_LIBC_MUTEX_BACKTRACE > 0
static void dump_lockholder(pid_t tid)
{
  char buf[BACKTRACE_BUFFER_SIZE(CONFIG_LIBC_MUTEX_BACKTRACE)];
  FAR mutex_t *mutex;

  mutex = (FAR mutex_t *)nxsched_get_tcb(tid)->waitobj;

  backtrace_format(buf, sizeof(buf), mutex->backtrace,
                   CONFIG_LIBC_MUTEX_BACKTRACE);

  _alert("Mutex holder(%d) backtrace:%s\n", mutex->holder, buf);
}
#else
#  define dump_lockholder(tid)
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
#  ifdef CONFIG_SCHED_BACKTRACE
          sched_dumpstack(deadlock[i]);
          dump_lockholder(deadlock[i]);
#  else
          _alert("deadlock pid: %d\n", deadlock[i]);
#  endif
        }
    }
}
#endif

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: pause_cpu_handler
 ****************************************************************************/

static noreturn_function int pause_cpu_handler(FAR void *arg)
{
  memcpy(g_last_regs[this_cpu()], running_regs(), sizeof(g_last_regs[0]));
  g_cpu_paused[this_cpu()] = true;
  up_flush_dcache_all();
  while (1);
}

/****************************************************************************
 * Name: pause_all_cpu
 ****************************************************************************/

static void pause_all_cpu(void)
{
  cpu_set_t cpus = (1 << CONFIG_SMP_NCPUS) - 1;
  int delay = CONFIG_ASSERT_PAUSE_CPU_TIMEOUT;

  CPU_CLR(this_cpu(), &cpus);
  nxsched_smp_call_async(cpus, &g_call_data);
  g_cpu_paused[this_cpu()] = true;

  /* Check if all CPUs paused with timeout */

  cpus = 0;
  while (delay-- > 0 && cpus < CONFIG_SMP_NCPUS)
    {
      if (g_cpu_paused[cpus])
        {
          cpus++;
        }
      else
        {
          up_mdelay(1);
        }
    }
}
#endif

#ifdef CONFIG_DEBUG_ALERT
/****************************************************************************
 * Name: dump_running_task
 ****************************************************************************/

static void dump_running_task(FAR struct tcb_s *rtcb, FAR void *regs)
{
  /* Register dump */

  up_dump_register(regs);

#ifdef CONFIG_ARCH_STACKDUMP
  dump_stacks(rtcb, up_getusrsp(regs));
#endif

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif
}

/****************************************************************************
 * Name: dump_assert_info
 *
 * Description:
 *   Dump basic information of assertion
 *
 ****************************************************************************/

static void dump_assert_info(FAR struct tcb_s *rtcb,
                             FAR const char *filename, int linenum,
                             FAR const char *msg, FAR void *regs)
{
  FAR struct tcb_s *ptcb = NULL;
  struct utsname name;

  if (rtcb->group &&
      (rtcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      ptcb = nxsched_get_tcb(rtcb->group->tg_pid);
    }

  uname(&name);
  _alert("Current Version: %s %s %s %s %s\n",
         name.sysname, name.nodename,
         name.release, name.version, name.machine);

  _alert("Assertion failed %s: at file: %s:%d task"
#ifdef CONFIG_SMP
         "(CPU%d)"
#endif
         ": "
         "%s "
         "process: %s "
         "%p\n",
         msg ? msg : "",
         filename ? filename : "", linenum,
#ifdef CONFIG_SMP
         this_cpu(),
#endif
         get_task_name(rtcb),
         ptcb ? get_task_name(ptcb) : "Kernel",
         rtcb->entry.main);

  /* Dump current CPU registers, running task stack and backtrace. */

  dump_running_task(rtcb, regs);

  /* Flush any buffered SYSLOG data */

  syslog_flush();
}
#endif  /* CONFIG_DEBUG_ALERT */

/****************************************************************************
 * Name: dump_fatal_info
 ****************************************************************************/

static void dump_fatal_info(FAR struct tcb_s *rtcb,
                            FAR const char *filename, int linenum,
                            FAR const char *msg, FAR void *regs)
{
#if defined(CONFIG_SMP) && defined(CONFIG_DEBUG_ALERT)
  int cpu;

  /* Dump other CPUs registers, running task stack and backtrace. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (cpu == this_cpu())
        {
          continue;
        }

      _alert("Dump CPU%d: %s\n", cpu,
             g_cpu_paused[cpu] ? "PAUSED" : "RUNNING");

      if (g_cpu_paused[cpu])
        {
          dump_running_task(g_running_tasks[cpu], g_last_regs[cpu]);
        }
    }
#endif

  /* Dump backtrace of other tasks. */

  dump_tasks();

#ifdef CONFIG_ARCH_DEADLOCKDUMP
  /* Deadlock Dump */

  dump_deadlock();
#endif

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif

  /* Flush previous SYSLOG data before possible long time coredump */

  syslog_flush();

#ifdef CONFIG_BOARD_CRASHDUMP_CUSTOM
  board_crashdump(up_getsp(), rtcb, filename, linenum, msg, regs);
#elif !defined(CONFIG_BOARD_CRASHDUMP_NONE)

  /* Dump core information */

#  ifdef CONFIG_BOARD_COREDUMP_FULL
  coredump_dump(INVALID_PROCESS_ID);
#  else
  coredump_dump(rtcb->pid);
#  endif
#endif

  /* Flush any buffered SYSLOG data */

  syslog_flush();
}

/****************************************************************************
 * Name: reset_board
 *
 * Description:
 *   Reset board or stuck here to flash LED. It should never return.
 ****************************************************************************/

static void reset_board(void)
{
#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
  up_flush_dcache_all();
  board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#else
  for (; ; )
    {
#ifdef CONFIG_ARCH_LEDS
      /* FLASH LEDs a 2Hz */

      board_autoled_on(LED_PANIC);
      up_mdelay(250);
      board_autoled_off(LED_PANIC);
#endif
      up_mdelay(250);
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _assert
 ****************************************************************************/

void _assert(FAR const char *filename, int linenum,
             FAR const char *msg, FAR void *regs)
{
  const bool os_ready = OSINIT_OS_READY();
  FAR struct tcb_s *rtcb = running_task();
  struct panic_notifier_s notifier_data;
  irqstate_t flags;

  if (g_nx_initstate == OSINIT_PANIC)
    {
      /* Already in fatal state, reset board directly. */

      reset_board(); /* Should not return. */
    }

  flags = 0; /* suppress GCC warning */
  if (os_ready)
    {
      flags = spin_lock_irqsave(&g_assert_lock);
      sched_lock();
    }

#if CONFIG_BOARD_RESET_ON_ASSERT < 2
  if (up_interrupt_context() ||
      (rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
#endif
    {
      /* Fatal error, enter panic state. */

      g_nx_initstate = OSINIT_PANIC;

      /* Disable KASAN to avoid false positive */

      kasan_stop();

#ifdef CONFIG_SMP
      if (os_ready)
        {
          pause_all_cpu();
        }
#endif
    }

  /* try to save current context if regs is null */

  if (regs == NULL)
    {
      up_saveusercontext(g_last_regs[this_cpu()]);
      regs = g_last_regs[this_cpu()];
    }
  else
    {
      memcpy(g_last_regs[this_cpu()], regs, sizeof(g_last_regs[0]));
    }

  notifier_data.rtcb = rtcb;
  notifier_data.regs = regs;
  notifier_data.filename = filename;
  notifier_data.linenum = linenum;
  notifier_data.msg = msg;
  panic_notifier_call_chain(g_nx_initstate == OSINIT_PANIC
                            ? PANIC_KERNEL : PANIC_TASK,
                            &notifier_data);
#ifdef CONFIG_ARCH_LEDS
  board_autoled_on(LED_ASSERTION);
#endif

  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#ifdef CONFIG_DEBUG_ALERT
  /* Dump basic info of assertion. */

  dump_assert_info(rtcb, filename, linenum, msg, regs);
#endif

  if (g_nx_initstate == OSINIT_PANIC)
    {
      /* Dump fatal info of assertion. */

      dump_fatal_info(rtcb, filename, linenum, msg, regs);

      panic_notifier_call_chain(PANIC_KERNEL_FINAL, &notifier_data);

      reboot_notifier_call_chain(SYS_HALT, NULL);

      reset_board(); /* Should not return. */
    }

  if (os_ready)
    {
      spin_unlock_irqrestore(&g_assert_lock, flags);
      sched_unlock();
    }
}
