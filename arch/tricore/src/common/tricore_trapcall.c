/****************************************************************************
 * arch/tricore/src/common/tricore_trapcall.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <sched/sched.h>
#include <nuttx/coredump.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"
#include "tricore_vectors.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tricore_trap_s g_trapinfo;

/* Bus trap hook for safety test modules (e.g., BusMonTst). */

static tricore_bustrap_hook_t g_bustrap_hook;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function void tricore_trapinfo(volatile void *trap)
{
  volatile struct tricore_trap_s *ctrap = (struct tricore_trap_s *)trap;

  g_trapinfo.tCpu   = ctrap->tCpu;
  g_trapinfo.tClass = ctrap->tClass;
  g_trapinfo.tId    = ctrap->tId;
  g_trapinfo.tAddr  = ctrap->tAddr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_bustrap_register_hook
 *
 * Description:
 *   Register a hook function for bus trap handling. This allows safety test
 *   modules (e.g., BusMonTst) to intercept and handle expected bus errors
 *   during self-tests without triggering a system panic.
 *
 * Input Parameters:
 *   hook - The hook function to register, or NULL to unregister
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tricore_bustrap_register_hook(tricore_bustrap_hook_t hook)
{
  g_bustrap_hook = hook;
}

int weak_function tricore_nmitrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! NMI Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
         TRICORE_TRAP_CLASS_NMI, tid, context);

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_mmutrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! MMU Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
         TRICORE_TRAP_CLASS_MMU,
         tid, context);

  _alert("MMU Trap Reason:\n");
  if (tid == TRICORE_TRAP_MMU_VAF)
    {
      _alert("\tVirtual Address Fill\n");
    }

  if (tid == TRICORE_TRAP_MMU_VAP)
    {
      _alert("\tVirtual Address Protection\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_internalprotrape(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Internal Protection Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
        TRICORE_TRAP_CLASS_INTPROT, tid, context);

  _alert("Internal Protection Reason:\n");
  if (tid == TRICORE_TRAP_INTPROT_PRIV)
    {
      _alert("\tPrivileged Instruction\n");
    }

  if (tid == TRICORE_TRAP_INTPROT_MPR)
    {
      _alert("\tMemory Protection Read\n");
    }

  if (tid == TRICORE_TRAP_INTPROT_MPW)
    {
      _alert("\tMemory Proteciton Write\n");
    }

  if (tid == TRICORE_TRAP_INTPROT_MPX)
    {
      _alert("\tMemory Protection Execution\n");
    }

  if (tid == TRICORE_TRAP_INTPROT_MPP)
    {
      _alert("\tMemory Protection Peripheral Access\n");
    }

  if (tid == TRICORE_TRAP_INTPROT_MPN)
    {
      _alert("\tMemory Protection Null Address\n");
    }

  if (tid == TRICORE_TRAP_INTPROT_GRWP)
    {
      _alert("\tGlobal Register Write Protection\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_insterrorstrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Instruction Errors Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
        TRICORE_TRAP_CLASS_INSTERR, tid, context);

  _alert("Instruction Errors Trap Reason:\n");
  if (tid == TRICORE_TRAP_INSTERR_IOPC)
    {
      _alert("\tIllegal Opcode\n");
    }

  if (tid == TRICORE_TRAP_INSTERR_UOPC)
    {
      _alert("\tUnimplemented Opcode\n");
    }

  if (tid == TRICORE_TRAP_INSTERR_OPD)
    {
      _alert("\tInvalid Operand Specification\n");
    }

  if (tid == TRICORE_TRAP_INSTERR_ALN)
    {
      _alert("\tData Address Alignment\n");
    }

  if (tid == TRICORE_TRAP_INSTERR_MEM)
    {
      _alert("\tInvalid Local Memory Address\n");
    }

#ifdef CONFIG_ARCH_TC1V8
  if (tid == TRICORE_TRAP_INSTERR_CSE)
    {
      _alert("\tCoprocessor Trap Synchronous Error\n");
#  ifdef CONFIG_ARCH_HAVE_FPU
      _alert("\tFPU_SYNC_TRAP:%" PRIX32 "\n\n",
             tricore_mfcr(FPU_SYNC_TRAP_REG));
      tricore_mtcr(FPU_SYNC_TRAP_REG, tricore_mfcr(FPU_SYNC_TRAP_REG) |
                                      (1U << FPU_TRAP_TCL_SHIFT));
#  endif
    }
#endif

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_contexmnttrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Context Management Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
        TRICORE_TRAP_CLASS_CTXMGMT, tid, context);

  _alert("Context Management Reason:\n");
  if (tid == TRICORE_TRAP_CTXMGMT_FCD)
    {
      _alert("\tFree Context List Depletion\n");
    }

  if (tid == TRICORE_TRAP_CTXMGMT_CDO)
    {
      _alert("\tCall Depth Overflow\n");
    }

  if (tid == TRICORE_TRAP_CTXMGMT_CDU)
    {
      _alert("\tCall Depth Underflow\n");
    }

  if (tid == TRICORE_TRAP_CTXMGMT_FCU)
    {
      _alert("\tFree Context List Underflow\n");
    }

  if (tid == TRICORE_TRAP_CTXMGMT_CSU)
    {
      _alert("\tCall Stack Underflow\n");
    }

  if (tid == TRICORE_TRAP_CTXMGMT_CTYP)
    {
      _alert("\tContext Type\n");
    }

  if (tid == TRICORE_TRAP_CTXMGMT_NEST)
    {
      _alert("\tNesting Error:RFE with non-zero call depth\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_bustrap(uint32_t tid, void *context, void *arg)
{
  /* First, let the registered hook try to handle the trap.
   * This is used by safety test modules (e.g., BusMonTst) to handle
   * expected bus errors during self-tests.
   */

  if (g_bustrap_hook != NULL)
    {
      if (g_bustrap_hook(tid, context) == OK)
        {
          /* Hook handled the trap, return normally */

          return OK;
        }
    }

  /* Hook did not handle or no hook registered, proceed with panic */

  _alert("PANIC!!! System Bus Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n", TRICORE_TRAP_CLASS_BUS,
         tid, context);

  _alert("System Bus Reason:\n");
  if (tid == TRICORE_TRAP_BUS_PSE)
    {
      _alert("\tProgram Fetch Synchronous Error\n");
    }

  if (tid == TRICORE_TRAP_BUS_DSE)
    {
      _alert("\tData Access Synchronous Error\n");
    }

  if (tid == TRICORE_TRAP_BUS_DAE)
    {
      _alert("\tData Access Asysnchronous Error\n");
    }

  if (tid == TRICORE_TRAP_BUS_CAE)
    {
      _alert("\tCoprocessor Trap Asynchronous Error\n");
#ifdef CONFIG_ARCH_HAVE_FPU
      _alert("\tFPU_ASYNC_TRAP:%" PRIX32 "\n\n",
             tricore_mfcr(FPU_ASYNC_TRAP_REG));
      tricore_mtcr(FPU_ASYNC_TRAP_REG, tricore_mfcr(FPU_ASYNC_TRAP_REG) |
                                       (1U << FPU_TRAP_TCL_SHIFT));
      UP_ISB();
#endif
    }

  if (tid == TRICORE_TRAP_BUS_PIE)
    {
      _alert("\tProgram Memory Integrity Error\n");
    }

  if (tid == TRICORE_TRAP_BUS_DIE)
    {
      _alert("\tData Memory Integrity Error\n");
    }

  if (tid == TRICORE_TRAP_BUS_TAE)
    {
      _alert("\tTemporal Asynchronous Error\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_assertiontrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Assertion Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
         TRICORE_TRAP_CLASS_ASSERT,
         tid, context);

  _alert("System Bus Reason:\n");
  if (tid == TRICORE_TRAP_ASSERT_OVF)
    {
      _alert("\tArithmetic Overflow\n");
    }

  if (tid == TRICORE_TRAP_ASSERT_SOVF)
    {
      _alert("\tSticky Arithmetic Overflow\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

/****************************************************************************
 * Name: tricore_trapcall
 *
 * Description:
 *   This is Trap exception handler
 *
 ****************************************************************************/

void tricore_trapcall(volatile void *trap)
{
  uintptr_t *regs;
  bool was_interrupt_context;

  struct tricore_trap_s *ctrap = (struct tricore_trap_s *)trap;
  enum tricore_trap_class_e tclass =
    (enum tricore_trap_class_e)ctrap->tClass;
  unsigned int tid = ctrap->tId;

  tricore_trapinfo(trap);

  regs = tricore_csa2addr(tricore_mfcr(CPU_PCXI));

  was_interrupt_context = up_interrupt_context();
  if (!was_interrupt_context)
    {
      up_set_interrupt_context(true);

      /* Update the current task's regs */

      g_running_tasks[this_cpu()]->xcp.regs = regs;
    }

  if (tclass == TRICORE_TRAP_CLASS_NMI)
    {
      tricore_nmitrap(tid, regs, NULL);
      up_set_interrupt_context(false);
      __jumpBackToLink();
    }

  if (tclass == TRICORE_TRAP_CLASS_MMU)
    {
      tricore_mmutrap(tid, regs, NULL);
      __jumpBackToLink();
    }

  if (tclass == TRICORE_TRAP_CLASS_INTPROT)
    {
      tricore_internalprotrape(tid, regs, NULL);
      __jumpBackToLink();
    }

  if (tclass == TRICORE_TRAP_CLASS_INSTERR)
    {
      tricore_insterrorstrap(tid, regs, NULL);
      __jumpBackToLink();
    }

  if (tclass == TRICORE_TRAP_CLASS_CTXMGMT)
    {
      tricore_contexmnttrap(tid, regs, NULL);
      __jumpBackToLink();
    }

  if (tclass == TRICORE_TRAP_CLASS_BUS)
    {
      if (tricore_bustrap(tid, regs, NULL) == OK)
        {
          /* Bus trap was handled by hook */

          if (!was_interrupt_context)
            {
              up_set_interrupt_context(false);

              /* g_running_tasks[]->xcp.regs is about to become invalid
               * and will be marked as NULL to avoid misusage.
               */

              g_running_tasks[this_cpu()]->xcp.regs = NULL;
            }
        }

      __jumpBackToLink();
    }

  if (tclass == TRICORE_TRAP_CLASS_ASSERT)
    {
      tricore_assertiontrap(tid, regs, NULL);
      __jumpBackToLink();
    }

  up_irq_save();
  PANIC_WITH_REGS("Trap", regs);
}

/****************************************************************************
 * Function:  tricore_trapinit
 *
 * Description:
 *   Trap init for tricore arch.
 *
 ****************************************************************************/

void tricore_trapinit(void)
{
#ifdef CONFIG_COREDUMP
  coredump_add_memory_region(&g_trapinfo, sizeof(struct tricore_trap_s),
                             PF_REGISTER);
#endif
}
