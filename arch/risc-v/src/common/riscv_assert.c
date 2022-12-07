/****************************************************************************
 * arch/risc-v/src/common/riscv_assert.c
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
#include <debug.h>

#include <nuttx/tls.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "riscv_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t s_last_regs[XCPTCONTEXT_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_stackdump
 ****************************************************************************/

static void riscv_stackdump(uintptr_t sp, uintptr_t stack_top)
{
  uintptr_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%" PRIxREG ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
             " %08" PRIx32 "\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: riscv_registerdump
 ****************************************************************************/

static inline void riscv_registerdump(volatile uintptr_t *regs)
{
  /* Are user registers available from interrupt processing? */

  _alert("EPC: %" PRIxREG "\n", regs[REG_EPC]);
  _alert("A0: %" PRIxREG " A1: %" PRIxREG " A2: %" PRIxREG
         " A3: %" PRIxREG "\n",
         regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3]);
  _alert("A4: %" PRIxREG " A5: %" PRIxREG " A6: %" PRIxREG
         " A7: %" PRIxREG "\n",
         regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  _alert("T0: %" PRIxREG " T1: %" PRIxREG " T2: %" PRIxREG
         " T3: %" PRIxREG "\n",
         regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3]);
  _alert("T4: %" PRIxREG " T5: %" PRIxREG " T6: %" PRIxREG "\n",
         regs[REG_T4], regs[REG_T5], regs[REG_T6]);
  _alert("S0: %" PRIxREG " S1: %" PRIxREG " S2: %" PRIxREG
         " S3: %" PRIxREG "\n",
         regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3]);
  _alert("S4: %" PRIxREG " S5: %" PRIxREG " S6: %" PRIxREG
         " S7: %" PRIxREG "\n",
         regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
  _alert("S8: %" PRIxREG " S9: %" PRIxREG " S10: %" PRIxREG
         " S11: %" PRIxREG "\n",
         regs[REG_S8], regs[REG_S9], regs[REG_S10], regs[REG_S11]);
#ifdef RISCV_SAVE_GP
  _alert("GP: %" PRIxREG " SP: %" PRIxREG " FP: %" PRIxREG
         " TP: %" PRIxREG " RA: %" PRIxREG "\n",
         regs[REG_GP], regs[REG_SP], regs[REG_FP], regs[REG_TP],
         regs[REG_RA]);
#else
  _alert("SP: %" PRIxREG " FP: %" PRIxREG " TP: %" PRIxREG
         " RA: %" PRIxREG "\n",
         regs[REG_SP], regs[REG_FP], regs[REG_TP], regs[REG_RA]);
#endif
}

/****************************************************************************
 * Name: riscv_dump_task
 ****************************************************************************/

static void riscv_dump_task(struct tcb_s *tcb, void *arg)
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
 * Name: riscv_dump_backtrace
 ****************************************************************************/

#ifdef CONFIG_SCHED_BACKTRACE
static void riscv_dump_backtrace(struct tcb_s *tcb, void *arg)
{
  /* Show back trace */

  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: riscv_showtasks
 ****************************************************************************/

static inline void riscv_showtasks(void)
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

  nxsched_foreach(riscv_dump_task, NULL);
#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(riscv_dump_backtrace, NULL);
#endif
}

/****************************************************************************
 * Name: riscv_dump_stack
 ****************************************************************************/

static void riscv_dump_stack(const char *tag, uintptr_t sp,
                             uintptr_t base, uint32_t size, bool force)
{
  uintptr_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("sp:     %08" PRIxPTR "\n", sp);
  _alert("  base: %08" PRIxPTR "\n", base);
  _alert("  size: %08" PRIx32 "\n", size);

  if (sp >= base && sp < top)
    {
      riscv_stackdump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
#ifdef CONFIG_STACK_COLORATION
          uint32_t remain;

          remain = size - riscv_stack_check((uintptr_t)base, size);
          base  += remain;
          size  -= remain;
#endif

#if CONFIG_ARCH_STACKDUMP_MAX_LENGTH > 0
          if (size > CONFIG_ARCH_STACKDUMP_MAX_LENGTH)
            {
              size = CONFIG_ARCH_STACKDUMP_MAX_LENGTH;
            }
#endif

          riscv_stackdump(base, base + size);
        }
    }
}

/****************************************************************************
 * Name: riscv_dumpstate
 ****************************************************************************/

static void riscv_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uintptr_t sp = up_getsp();

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      rtcb->xcp.regs = (uintptr_t *)CURRENT_REGS;
    }
  else
    {
      up_saveusercontext(s_last_regs);
      rtcb->xcp.regs = (uintptr_t *)s_last_regs;
    }

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif

  /* Dump the registers (if available) */

  riscv_registerdump(rtcb->xcp.regs);

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  riscv_dump_stack("IRQ", sp,
                   (uintptr_t)g_intstackalloc,
                   (CONFIG_ARCH_INTERRUPTSTACK & ~15),
                   !!CURRENT_REGS);
  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_SP];
    }
#endif

  riscv_dump_stack("User", sp,
                   (uintptr_t)rtcb->stack_base_ptr,
                   (uint32_t)rtcb->adj_stack_size,
#ifdef CONFIG_ARCH_KERNEL_STACK
                 false
#else
                 true
#endif
                );

#ifdef CONFIG_ARCH_KERNEL_STACK
  riscv_dump_stack("Kernel", sp,
                   (uintptr_t)rtcb->xcp.kstack,
                   CONFIG_ARCH_KERNEL_STACKSIZE,
                   false);
#endif
}
#else
#  define riscv_dumpstate()
#endif /* CONFIG_ARCH_STACKDUMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const char *filename, int lineno)
{
  board_autoled_on(LED_ASSERTION);
  riscv_dumpstate();

  /* Dump the state of all tasks (if available) */

  riscv_showtasks();
}
