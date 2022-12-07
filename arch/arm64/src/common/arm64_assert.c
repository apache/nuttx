/****************************************************************************
 * arch/arm64/src/common/arm64_assert.c
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

#include <nuttx/arch.h>
#include <debug.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Name: arm_registerdump
 ****************************************************************************/

static void arm64_registerdump(struct regs_context * regs)
{
  _alert("stack = %p\n", regs);
  _alert("x0:   0x%-16"PRIx64"  x1:   0x%"PRIx64"\n",
    regs->regs[REG_X0], regs->regs[REG_X1]);
  _alert("x2:   0x%-16"PRIx64"  x3:   0x%"PRIx64"\n",
    regs->regs[REG_X2], regs->regs[REG_X3]);
  _alert("x4:   0x%-16"PRIx64"  x5:   0x%"PRIx64"\n",
    regs->regs[REG_X4], regs->regs[REG_X5]);
  _alert("x6:   0x%-16"PRIx64"  x7:   0x%"PRIx64"\n",
    regs->regs[REG_X6], regs->regs[REG_X7]);
  _alert("x8:   0x%-16"PRIx64"  x9:   0x%"PRIx64"\n",
    regs->regs[REG_X8], regs->regs[REG_X9]);
  _alert("x10:  0x%-16"PRIx64"  x11:  0x%"PRIx64"\n",
    regs->regs[REG_X10], regs->regs[REG_X11]);
  _alert("x12:  0x%-16"PRIx64"  x13:  0x%"PRIx64"\n",
    regs->regs[REG_X12], regs->regs[REG_X13]);
  _alert("x14:  0x%-16"PRIx64"  x15:  0x%"PRIx64"\n",
    regs->regs[REG_X14], regs->regs[REG_X15]);
  _alert("x16:  0x%-16"PRIx64"  x17:  0x%"PRIx64"\n",
    regs->regs[REG_X16], regs->regs[REG_X17]);
  _alert("x18:  0x%-16"PRIx64"  x19:  0x%"PRIx64"\n",
    regs->regs[REG_X18], regs->regs[REG_X19]);
  _alert("x20:  0x%-16"PRIx64"  x21:  0x%"PRIx64"\n",
    regs->regs[REG_X20], regs->regs[REG_X21]);
  _alert("x22:  0x%-16"PRIx64"  x23:  0x%"PRIx64"\n",
    regs->regs[REG_X22], regs->regs[REG_X23]);
  _alert("x24:  0x%-16"PRIx64"  x25:  0x%"PRIx64"\n",
    regs->regs[REG_X24], regs->regs[REG_X25]);
  _alert("x26:  0x%-16"PRIx64"  x27:  0x%"PRIx64"\n",
    regs->regs[REG_X26], regs->regs[REG_X27]);
  _alert("x28:  0x%-16"PRIx64"  x29:  0x%"PRIx64"\n",
    regs->regs[REG_X28], regs->regs[REG_X29]);
  _alert("x30:  0x%-16"PRIx64"\n", regs->regs[REG_X30]);

  _alert("\n");
  _alert("STATUS Registers:\n");
  _alert("SPSR:      0x%-16"PRIx64"\n", regs->spsr);
  _alert("ELR:       0x%-16"PRIx64"\n", regs->elr);
  _alert("SP_EL0:    0x%-16"PRIx64"\n", regs->sp_el0);
  _alert("SP_ELX:    0x%-16"PRIx64"\n", regs->sp_elx);
  _alert("TPIDR_EL0: 0x%-16"PRIx64"\n", regs->tpidr_el0);
  _alert("TPIDR_EL1: 0x%-16"PRIx64"\n", regs->tpidr_el1);
  _alert("EXE_DEPTH: 0x%-16"PRIx64"\n", regs->exe_depth);
}

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_stackdump
 ****************************************************************************/

static void arm64_stackdump(uint64_t sp, uint64_t stack_top)
{
  uint64_t stack;

  for (stack = sp & ~0x1f; stack < (stack_top & ~0x1f); stack += 64)
    {
      uint64_t *ptr = (uint64_t *)stack;
      _alert("%08" PRIx64 ": %08" PRIx64 " %08" PRIx64 " %08" PRIx64
             " %08" PRIx64 " %08" PRIx64 " %08" PRIx64 " %08" PRIx64
             " %08" PRIx64 "\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: arm_dump_task
 ****************************************************************************/

static void arm64_dump_task(struct tcb_s *tcb, void *arg)
{
  char args[64] = "";
#ifdef CONFIG_STACK_COLORATION
  uint64_t stack_filled = 0;
  uint64_t stack_used;
#endif
#ifdef CONFIG_SCHED_CPULOAD
  struct cpuload_s cpuload;
  uint64_t fracpart;
  uint64_t intpart;
  uint64_t tmp;

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

      snprintf(args, sizeof(args), "%p ", ptcb->arg);
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
#ifdef CONFIG_STACK_COLORATION
         "   %7lu"
#endif
         "   %7lu"
#ifdef CONFIG_STACK_COLORATION
         "   %3" PRId64 ".%1" PRId64 "%%%c"
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "   %3" PRId64 ".%01" PRId64 "%%"
#endif
#if CONFIG_TASK_NAME_SIZE > 0
         "   %s %s\n",
#else
         "   %s\n",
#endif
         tcb->pid, tcb->sched_priority,
#ifdef CONFIG_SMP
         tcb->cpu,
#endif
#ifdef CONFIG_STACK_COLORATION
         (unsigned long)up_check_tcbstack(tcb),
#endif
         (unsigned long)tcb->adj_stack_size
#ifdef CONFIG_STACK_COLORATION
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
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
 * Name: arm_dump_backtrace
 ****************************************************************************/

#ifdef CONFIG_SCHED_BACKTRACE
static void arm64_dump_backtrace(struct tcb_s *tcb, void *arg)
{
  /* Show back trace */

  sched_dumpstack(tcb->pid);
}
#endif

/****************************************************************************
 * Name: arm_showtasks
 ****************************************************************************/

static void arm64_showtasks(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 15
#  ifdef CONFIG_STACK_COLORATION
  uint64_t stack_used = up_check_intstack();
  uint64_t stack_filled = 0;

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
#ifdef CONFIG_STACK_COLORATION
         "      USED"
#endif
         "     STACK"
#ifdef CONFIG_STACK_COLORATION
         "   FILLED "
#endif
#ifdef CONFIG_SCHED_CPULOAD
         "      CPU"
#endif
         "   COMMAND\n");

#if CONFIG_ARCH_INTERRUPTSTACK > 7
  _alert("  ----   ----"
#  ifdef CONFIG_SMP
         "  ----"
#  endif
#  ifdef CONFIG_STACK_COLORATION
         "   %7lu"
#  endif
         "   %7lu"
#  ifdef CONFIG_STACK_COLORATION
         "   %3" PRId64 ".%1" PRId64 "%%%c"
#  endif
#  ifdef CONFIG_SCHED_CPULOAD
         "     ----"
#  endif
#  if CONFIG_TASK_NAME_SIZE > 0
         "   irq"
#  endif
         "\n"
#  ifdef CONFIG_STACK_COLORATION
         , (unsigned long)stack_used
#  endif
         , (unsigned long)(CONFIG_ARCH_INTERRUPTSTACK & ~15)
#  ifdef CONFIG_STACK_COLORATION
         , stack_filled / 10, stack_filled % 10,
         (stack_filled >= 10 * 80 ? '!' : ' ')
#  endif
        );
#endif

  nxsched_foreach(arm64_dump_task, NULL);
#ifdef CONFIG_SCHED_BACKTRACE
  nxsched_foreach(arm64_dump_backtrace, NULL);
#endif
}

/****************************************************************************
 * Name: arm_dump_stack
 ****************************************************************************/

static void arm64_dump_stack(const char *tag, uint64_t sp,
                           uint64_t base, uint64_t size, bool force)
{
  uint64_t top = base + size;

  _alert("%s Stack:\n", tag);
  _alert("sp:     %08" PRIx64 "\n", sp);
  _alert("  base: %08" PRIx64 "\n", base);
  _alert("  size: %08" PRIx64 "\n", size);

  if (sp >= base && sp < top)
    {
      arm64_stackdump(sp, top);
    }
  else
    {
      _alert("ERROR: %s Stack pointer is not within the stack\n", tag);

      if (force)
        {
          arm64_stackdump(base, top);
        }
    }
}

/****************************************************************************
 * Name: arm64_dumpstate_assert (for assert)
 ****************************************************************************/

static void arm64_dumpstate_assert(void)
{
  struct tcb_s *rtcb = (struct tcb_s *)arch_get_current_tcb();
  uint64_t sp = up_getsp();

  /* Show back trace */

#ifdef CONFIG_SCHED_BACKTRACE
  sched_dumpstack(rtcb->pid);
#endif

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      /* in interrupt */

      rtcb->xcp.regs = (uint64_t *)CURRENT_REGS;
    }
  else
    {
      up_saveusercontext(rtcb->xcp.regs);
    }

  /* Dump the registers */

  arm64_registerdump((struct regs_context *)rtcb->xcp.regs);

  /* Dump the irq stack */

#if CONFIG_ARCH_INTERRUPTSTACK > 7
  arm64_dump_stack("IRQ", sp,
#  ifdef CONFIG_SMP
                 (uint64_t)arm64_intstack_alloc(),
#  else
                 (uint64_t)&g_interrupt_stack,
#  endif
                 (CONFIG_ARCH_INTERRUPTSTACK & ~15),
                 !!CURRENT_REGS);

  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_X13];
    }
#endif

  /* Dump the user stack */

  arm64_dump_stack("User", sp,
                 (uint64_t)rtcb->stack_base_ptr,
                 (uint64_t)rtcb->adj_stack_size,
#ifdef CONFIG_ARCH_KERNEL_STACK
                 false
#else
                 true
#endif
                );

#ifdef CONFIG_ARCH_KERNEL_STACK
  arm64_dump_stack("Kernel", sp,
                 (uint64_t)rtcb->xcp.kstack,
                 CONFIG_ARCH_KERNEL_STACKSIZE,
                 false);
#endif

  /* Dump the state of all tasks (if available) */

  arm64_showtasks();
}

#endif /* CONFIG_ARCH_STACKDUMP */

/****************************************************************************
 * Name: arm64_dump_fatal
 ****************************************************************************/

void arm64_dump_fatal(struct regs_context * regs)
{
#ifdef CONFIG_SCHED_BACKTRACE
  struct tcb_s *rtcb = (struct tcb_s *)regs->tpidr_el1;

  /* Show back trace */

  sched_dumpstack(rtcb->pid);
#endif

  /* Dump the registers */

  arm64_registerdump(regs);
}

void up_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
        {
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const char *filename, int lineno)
{
#ifdef CONFIG_ARCH_STACKDUMP
  arm64_dumpstate_assert();
#endif
}
