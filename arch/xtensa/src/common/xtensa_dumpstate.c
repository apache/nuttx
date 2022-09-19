/****************************************************************************
 * arch/xtensa/src/common/xtensa_dumpstate.c
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
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/tls.h>
#include <nuttx/arch.h>
#include <nuttx/syslog/syslog.h>

#include <arch/xtensa/xtensa_corebits.h>
#include <arch/board/board.h>
#include <arch/chip/core-isa.h>

#include "sched/sched.h"
#include "xtensa.h"

#ifdef CONFIG_DEBUG_ALERT

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t s_last_regs[XCPTCONTEXT_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_dump_task
 ****************************************************************************/

static void xtensa_dump_task(struct tcb_s *tcb, void *arg)
{
  char args[64] = "";
#ifdef CONFIG_STACK_COLORATION
  uint32_t stack_filled = 0;
  uint32_t stack_used;
#endif
#ifdef CONFIG_SCHED_CPULOAD
  struct cpuload_s cpuload;
  uint32_t fracpart;
  uint32_t intpart;
  uint32_t tmp;

  clock_cpuload(tcb->pid, &cpuload);

  if (cpuload.total > 0)
    {
      tmp      = (1000 * cpuload.active) / cpuload.total;
      intpart  = tmp / 10;
      fracpart = tmp - 10 * intpart;
    }
  else
    {
      intpart  = 0;
      fracpart = 0;
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

      snprintf(args, sizeof(args), " %p", ptcb->arg);
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
         "   %7lu"
#ifdef CONFIG_STACK_COLORATION
         "   %7lu   %3" PRId32 ".%1" PRId32 "%%%c"
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "   %3" PRId32 ".%01" PRId32 "%%"
#endif
         "   %s%s\n"
         , tcb->pid, tcb->sched_priority
#ifdef CONFIG_SMP
         , tcb->cpu
#endif
         , (unsigned long)tcb->adj_stack_size
#ifdef CONFIG_STACK_COLORATION
         , (unsigned long)up_check_tcbstack(tcb)
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
 * Name: xtensa_dump_backtrace
 ****************************************************************************/

#ifdef CONFIG_SCHED_BACKTRACE
static void xtensa_dump_backtrace(struct tcb_s *tcb, void *arg)
{
  /* Show back trace */

  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: xtensa_showtasks
 ****************************************************************************/

static inline void xtensa_showtasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 15
#  ifdef CONFIG_STACK_COLORATION
  uint32_t stack_used = up_check_intstack();
  uint32_t stack_filled = 0;

  if ((CONFIG_ARCH_INTERRUPTSTACK & ~15) > 0 && stack_used > 0)
    {
      /* Use fixed-point math with one decimal place */

      stack_filled = 10 * 100 *
                     stack_used / (CONFIG_ARCH_INTERRUPTSTACK & ~15);
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

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  _alert("  ----   ----"
#  ifdef CONFIG_SMP
         "  ----"
#  endif
         "   %7u"
#  ifdef CONFIG_STACK_COLORATION
         "   %7" PRId32 "   %3" PRId32 ".%1" PRId32 "%%%c"
#  endif
#  ifdef CONFIG_SCHED_CPULOAD
         "     ----"
#  endif
         "   irq\n"
         , (CONFIG_ARCH_INTERRUPTSTACK & ~15)
#  ifdef CONFIG_STACK_COLORATION
         , stack_used
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
#  endif
        );
#endif

  nxsched_foreach(xtensa_dump_task, NULL);
#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(xtensa_dump_backtrace, NULL);
#endif
}

/****************************************************************************
 * Name: xtensa_stackdump
 ****************************************************************************/

static void xtensa_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  /* Flush any buffered SYSLOG data to avoid overwrite */

  syslog_flush();

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: xtensa_registerdump
 ****************************************************************************/

static inline void xtensa_registerdump(uintptr_t *regs)
{
  _alert("   PC: %08lx    PS: %08lx\n",
         (unsigned long)regs[REG_PC], (unsigned long)regs[REG_PS]);
  _alert("   A0: %08lx    A1: %08lx    A2: %08lx    A3: %08lx\n",
         (unsigned long)regs[REG_A0], (unsigned long)regs[REG_A1],
         (unsigned long)regs[REG_A2], (unsigned long)regs[REG_A3]);
  _alert("   A4: %08lx    A5: %08lx    A6: %08lx    A7: %08lx\n",
         (unsigned long)regs[REG_A4], (unsigned long)regs[REG_A5],
         (unsigned long)regs[REG_A6], (unsigned long)regs[REG_A7]);
  _alert("   A8: %08lx    A9: %08lx   A10: %08lx   A11: %08lx\n",
         (unsigned long)regs[REG_A8], (unsigned long)regs[REG_A9],
         (unsigned long)regs[REG_A10], (unsigned long)regs[REG_A11]);
  _alert("  A12: %08lx   A13: %08lx   A14: %08lx   A15: %08lx\n",
         (unsigned long)regs[REG_A12], (unsigned long)regs[REG_A13],
         (unsigned long)regs[REG_A14], (unsigned long)regs[REG_A15]);
  _alert("  SAR: %08lx CAUSE: %08lx VADDR: %08lx\n",
         (unsigned long)regs[REG_SAR], (unsigned long)regs[REG_EXCCAUSE],
         (unsigned long)regs[REG_EXCVADDR]);
#if XCHAL_HAVE_LOOPS != 0
  _alert(" LBEG: %08lx  LEND: %08lx  LCNT: %08lx\n",
         (unsigned long)regs[REG_LBEG], (unsigned long)regs[REG_LEND],
         (unsigned long)regs[REG_LCOUNT]);
#endif
}

/****************************************************************************
 * Name: xtensa_dump_stack
 ****************************************************************************/

static void xtensa_dump_stack(const char *tag, uint32_t sp,
                              uint32_t base, uint32_t size, bool force)
{
  uint32_t top = base + size;
#ifdef CONFIG_STACK_COLORATION
  uint32_t used = xtensa_stack_check((uintptr_t)base, size);
#endif

  _alert("%s Stack:\n", tag);
  _alert("sp:     %08" PRIx32 "\n", sp);
  _alert("  base: %08" PRIx32 "\n", base);
  _alert("  size: %08" PRIx32 "\n", size);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08" PRIx32 "\n", used);
#endif

  if (sp >= base && sp < top)
    {
      xtensa_stackdump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
#ifdef CONFIG_STACK_COLORATION
          base += size - used;
          size = used;
#endif

#if CONFIG_ARCH_STACKDUMP_MAX_LENGTH > 0
          if (size > CONFIG_ARCH_STACKDUMP_MAX_LENGTH)
            {
              size = CONFIG_ARCH_STACKDUMP_MAX_LENGTH;
            }
#endif

          xtensa_stackdump(base, base + size);
        }
    }
}

/****************************************************************************
 * Name: xtensa_dumpstate
 ****************************************************************************/

void xtensa_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = up_getsp();

#ifdef CONFIG_SMP
  /* Show the CPU number */

  _alert("CPU%d:\n", up_cpu_index());
#endif

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      rtcb->xcp.regs = (uint32_t *)CURRENT_REGS;
    }
  else
    {
      up_saveusercontext(s_last_regs);
      rtcb->xcp.regs = (uint32_t *)s_last_regs;
    }

  /* Dump the registers (if available) */

  xtensa_registerdump(rtcb->xcp.regs);

  /* Dump the backtrace */

#if defined(CONFIG_XTENSA_DUMPBT_ON_ASSERT) && \
    defined(CONFIG_SCHED_BACKTRACE)
  sched_dumpstack(rtcb->pid);
#endif

  /* Dump the irq stack */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  xtensa_dump_stack("IRQ", sp,
#  ifdef CONFIG_SMP
                    (uint32_t)xtensa_intstack_alloc(),
#  else
                    (uint32_t)g_intstackalloc,
#  endif
                    INTSTACK_SIZE,
                    !!CURRENT_REGS);

  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_A1];
    }
#endif

  /* Dump the user stack */

  xtensa_dump_stack("User", sp,
                    (uint32_t)rtcb->stack_base_ptr,
                    (uint32_t)rtcb->adj_stack_size,
                    true);

  /* Dump the state of all tasks (if available) */

  xtensa_showtasks();
}

#endif /* CONFIG_DEBUG_ALERT */
