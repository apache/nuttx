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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/xtensa/xtensa_corebits.h>
#include <arch/board/board.h>
#include <arch/chip/core-isa.h>
#include "sched/sched.h"
#include "xtensa.h"
#include "chip_memory.h"
#include "chip_macros.h"

#ifdef CONFIG_DEBUG_ALERT

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_taskdump
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void up_taskdump(FAR struct tcb_s *tcb, FAR void *arg)
{
  /* Dump interesting properties of this task */

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("%s: PID=%d Stack Used=%lu of %lu\n",
        tcb->name, tcb->pid, (unsigned long)up_check_tcbstack(tcb),
        (unsigned long)tcb->adj_stack_size);
#else
  _alert("PID: %d Stack Used=%lu of %lu\n",
        tcb->pid, (unsigned long)up_check_tcbstack(tcb),
        (unsigned long)tcb->adj_stack_size);
#endif
}
#endif

/****************************************************************************
 * Name: up_showtasks
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static inline void up_showtasks(void)
{
  /* Dump interesting properties of each task in the crash environment */

  nxsched_foreach(up_taskdump, NULL);
}
#else
#  define up_showtasks()
#endif

/****************************************************************************
 * Name: xtensa_stackdump
 ****************************************************************************/

static void xtensa_stackdump(uint32_t sp, uint32_t stack_top)
{
  uint32_t stack;

  for (stack = sp & ~0x1f; stack < stack_top; stack += 32)
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

static inline void xtensa_registerdump(void)
{
  uint32_t *regs = (uint32_t *)CURRENT_REGS; /* Don't need volatile here */

  /* Are user registers available from interrupt processing? */

  if (regs == NULL)
    {
      /* No.. capture user registers by hand */

      xtensa_context_save(s_last_regs);
      regs = s_last_regs;
    }

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
#ifndef __XTENSA_CALL0_ABI__
  _alert(" TMP0: %08lx  TMP1: %08lx\n",
         (unsigned long)regs[REG_TMP0], (unsigned long)regs[REG_TMP1]);
#endif
}

#ifdef CONFIG_XTENSA_DUMPBT_ON_ASSERT

/****************************************************************************
 * Name: xtensa_getcause
 ****************************************************************************/

static inline uint32_t xtensa_getcause(void)
{
  uint32_t cause;

  __asm__ __volatile__
  (
    "rsr %0, EXCCAUSE"  : "=r"(cause)
  );

  return cause;
}

/****************************************************************************
 * Name: stackpc
 ****************************************************************************/

static inline uint32_t stackpc(uint32_t pc)
{
  if (pc & 0x80000000)
    {
      /* Top two bits of a0 (return address) specify window increment.
       * Overwrite to map to address space.
       */

      pc = (pc & 0x3fffffff) | 0x40000000;
    }

  /* Minus 3 to get PC of previous instruction (i.e. instruction executed
   * before return address).
   */

  return pc - 3;
}

/****************************************************************************
 * Name: corruptedframe
 ****************************************************************************/

static inline bool corruptedframe(uint32_t pc, uint32_t sp)
{
  return !(xtensa_ptr_exec((void *)stackpc(pc)) || xtensa_sp_sane(sp));
}

/****************************************************************************
 * Name: nextframe
 ****************************************************************************/

static bool nextframe(uint32_t *pc, uint32_t *sp, uint32_t *npc)
{
  /* Use frame(i - 1)'s base save area located below frame(i)'s sp to get
   * frame(i - 1)'s sp and frame(i - 2)'s pc. Base save area consists of
   * 4 words under SP.
   */

  void *bsa = (void *)*sp;

  *pc  = *npc;
  *npc = *((uint32_t *)(bsa - 16));
  *sp  = *((uint32_t *)(bsa - 12));

  return !corruptedframe(*pc, *sp);
}

/****************************************************************************
 * Name: xtensa_btdump
 ****************************************************************************/

static inline void xtensa_btdump(void)
{
  uint32_t pc;
  uint32_t sp;
  uint32_t npc;
  int i;
  bool corrupted = false;

  uint32_t *regs = (uint32_t *)CURRENT_REGS;

  pc  = regs[REG_PC];
  npc = regs[REG_A0]; /* return register */
  sp  = regs[REG_A1]; /* stack pointer */

  _alert("Backtrace0: %x:%x\n", stackpc(pc), sp);

  corrupted = corruptedframe(pc, sp) &&
              !(xtensa_getcause() == EXCCAUSE_INSTR_PROHIBITED);

  for (i = 1; i <= CONFIG_XTENSA_BTDEPTH && npc != 0 && !corrupted; i++)
    {
      if (!nextframe(&pc, &sp, &npc))
        {
          corrupted = true;
        }

      _alert("Backtrace%d: %x:%x\n", i, stackpc(pc), sp);
    }

  _alert("BACKTRACE %s\n",
         (corrupted ? "CORRUPTED!" : (npc == 0 ? "Done":"CONTINUES...")));
}
#endif /* CONFIG_XTENSA_DUMPBT_ON_ASSERT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_dumpstate
 ****************************************************************************/

void xtensa_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 15
  uint32_t istackbase;
  uint32_t istacksize;
#endif

#ifdef CONFIG_SMP
  /* Show the CPU number */

  _alert("CPU%d:\n", up_cpu_index());
#endif

  /* Dump the registers (if available) */

  xtensa_registerdump();

  /* Dump the backtrace */

#ifdef CONFIG_XTENSA_DUMPBT_ON_ASSERT
  xtensa_btdump();
#endif

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->stack_base_ptr;
  ustacksize = (uint32_t)rtcb->adj_stack_size;

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
#ifdef CONFIG_SMP
  istackbase = (uint32_t)xtensa_intstack_alloc();
#else
  istackbase = (uint32_t)&g_intstackalloc;
#endif
  istacksize = INTSTACK_SIZE;

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_intstack());
#endif

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp >= istackbase && sp < istackbase + istacksize)
    {
      /* Yes.. dump the interrupt stack */

      xtensa_stackdump(sp, istackbase + istacksize);
    }
  else if (CURRENT_REGS)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      xtensa_stackdump(istackbase, istackbase + istacksize);
    }

  /* Extract the user stack pointer if we are in an interrupt handler.
   * If we are not in an interrupt handler.  Then sp is the user stack
   * pointer (and the above range check should have failed).
   */

  if (CURRENT_REGS)
    {
      sp = CURRENT_REGS[REG_A1];
      _alert("sp:     %08x\n", sp);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("  used: %08x\n", up_check_tcbstack(rtcb));
#endif
#else
  _alert("sp:         %08x\n", sp);
  _alert("stack base: %08x\n", ustackbase);
  _alert("stack size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
  _alert("stack used: %08x\n", up_check_tcbstack(rtcb));
#endif
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp >= ustackbase && sp < ustackbase + ustacksize)
    {
      xtensa_stackdump(sp, ustackbase + ustacksize);
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      xtensa_stackdump(ustackbase, ustackbase + ustacksize);
    }

  /* Dump the state of all tasks (if available) */

  up_showtasks();
}

#endif /* CONFIG_DEBUG_ALERT */
