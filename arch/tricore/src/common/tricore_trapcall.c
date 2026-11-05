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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Trap classes and trap identification numbers (TIN), as named in the
 * TriCore architecture manual.
 */

#define TRICORE_TRAP_CLASS_MMU    0  /* Memory management */
#define TRICORE_TRAP_CLASS_PROT   1  /* Internal protection */
#define TRICORE_TRAP_CLASS_INSN   2  /* Instruction errors */
#define TRICORE_TRAP_CLASS_CTX    3  /* Context management */
#define TRICORE_TRAP_CLASS_BUS    4  /* System bus and peripheral errors */
#define TRICORE_TRAP_CLASS_ASSERT 5  /* Assertion */
#define TRICORE_TRAP_CLASS_NMI    7  /* Non-maskable interrupt */

#define TRICORE_TIN_VAF           0  /* Class 0: virtual address fill */
#define TRICORE_TIN_VAP           1  /* Class 0: virtual addr protection */

#define TRICORE_TIN_PRIV          1  /* Class 1: privilege violation */
#define TRICORE_TIN_MPR           2  /* Class 1: protection read */
#define TRICORE_TIN_MPW           3  /* Class 1: protection write */
#define TRICORE_TIN_MPX           4  /* Class 1: protection execute */
#define TRICORE_TIN_MPP           5  /* Class 1: peripheral access */
#define TRICORE_TIN_MPN           6  /* Class 1: null address */
#define TRICORE_TIN_GRWP          7  /* Class 1: global register write */

#define TRICORE_TIN_IOPC          1  /* Class 2: illegal opcode */
#define TRICORE_TIN_UOPC          2  /* Class 2: unimplemented opcode */
#define TRICORE_TIN_OPD           3  /* Class 2: invalid operand */
#define TRICORE_TIN_ALN           4  /* Class 2: data address alignment */
#define TRICORE_TIN_MEM           5  /* Class 2: invalid local memory */
#define TRICORE_TIN_CSE           6  /* Class 2: coprocessor synchronous */

#define TRICORE_TIN_FCD           1  /* Class 3: free context depletion */
#define TRICORE_TIN_CDO           2  /* Class 3: call depth overflow */
#define TRICORE_TIN_CDU           3  /* Class 3: call depth underflow */
#define TRICORE_TIN_FCU           4  /* Class 3: free context underflow */
#define TRICORE_TIN_CSU           5  /* Class 3: call stack underflow */
#define TRICORE_TIN_CTYP          6  /* Class 3: context type error */
#define TRICORE_TIN_NEST          7  /* Class 3: nesting error */

#define TRICORE_TIN_PSE           1  /* Class 4: program fetch synchronous */
#define TRICORE_TIN_DSE           2  /* Class 4: data access synchronous */
#define TRICORE_TIN_DAE           3  /* Class 4: data access asynchronous */
#define TRICORE_TIN_CAE           4  /* Class 4: coprocessor asynchronous */
#define TRICORE_TIN_PIE           5  /* Class 4: program memory integrity */
#define TRICORE_TIN_DIE           6  /* Class 4: data memory integrity */
#define TRICORE_TIN_TAE           7  /* Class 4: temporal asynchronous */

#define TRICORE_TIN_OVF           1  /* Class 5: arithmetic overflow */
#define TRICORE_TIN_SOVF          2  /* Class 5: sticky overflow */

/* The trap vectors enter tricore_trapcall() with jli, return with ji */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
#  define tricore_trap_return() __asm__ volatile ("ji a11" : : : "a11")
#else
#  define tricore_trap_return() __asm__ volatile ("ji %a11")
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tricore_trap_s g_trapinfo;

/* Bus trap hook for safety test modules (e.g., BusMonTst). */

static tricore_bustrap_hook_t g_bustrap_hook;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tricore_trapinfo(volatile void *trap)
{
  g_trapinfo = *(struct tricore_trap_s *)trap;
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
  _alert("\tClass %d TID: %d regs: %p\n",
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
  if (tid == TRICORE_TIN_VAF)
    {
      _alert("\tVirtual Address Fill\n");
    }

  if (tid == TRICORE_TIN_VAP)
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
        TRICORE_TRAP_CLASS_PROT, tid, context);

  _alert("Internal Protection Reason:\n");
  if (tid == TRICORE_TIN_PRIV)
    {
      _alert("\tPrivileged Instruction\n");
    }

  if (tid == TRICORE_TIN_MPR)
    {
      _alert("\tMemory Protection Read\n");
    }

  if (tid == TRICORE_TIN_MPW)
    {
      _alert("\tMemory Proteciton Write\n");
    }

  if (tid == TRICORE_TIN_MPX)
    {
      _alert("\tMemory Protection Execution\n");
    }

  if (tid ==
      TRICORE_TIN_MPP)
    {
      _alert("\tMemory Protection Peripheral Access\n");
    }

  if (tid == TRICORE_TIN_MPN)
    {
      _alert("\tMemory Protection Null Address\n");
    }

  if (tid == TRICORE_TIN_GRWP)
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
        TRICORE_TRAP_CLASS_INSN, tid, context);

  _alert("Instruction Errors Trap Reason:\n");
  if (tid == TRICORE_TIN_IOPC)
    {
      _alert("\tIllegal Opcode\n");
    }

  if (tid == TRICORE_TIN_UOPC)
    {
      _alert("\tUnimplemented Opcode\n");
    }

  if (tid == TRICORE_TIN_OPD)
    {
      _alert("\tInvalid Operand Specification\n");
    }

  if (tid == TRICORE_TIN_ALN)
    {
      _alert("\tData Address Alignment\n");
    }

  if (tid == TRICORE_TIN_MEM)
    {
      _alert("\tInvalid Local Memory Address\n");
    }

#ifdef CONFIG_ARCH_TC1V8
  if (tid ==
      TRICORE_TIN_CSE)
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
        TRICORE_TRAP_CLASS_CTX, tid, context);

  _alert("Context Management Reason:\n");
  if (tid == TRICORE_TIN_FCD)
    {
      _alert("\tFree Context List Depletion\n");
    }

  if (tid == TRICORE_TIN_CDO)
    {
      _alert("\tCall Depth Overflow\n");
    }

  if (tid == TRICORE_TIN_CDU)
    {
      _alert("\tCall Depth Underflow\n");
    }

  if (tid == TRICORE_TIN_FCU)
    {
      _alert("\tFree Context List Underflow\n");
    }

  if (tid == TRICORE_TIN_CSU)
    {
      _alert("\tCall Stack Underflow\n");
    }

  if (tid == TRICORE_TIN_CTYP)
    {
      _alert("\tContext Type\n");
    }

  if (tid == TRICORE_TIN_NEST)
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
  if (tid == TRICORE_TIN_PSE)
    {
      _alert("\tProgram Fetch Synchronous Error\n");
    }

  if (tid == TRICORE_TIN_DSE)
    {
      _alert("\tData Access Synchronous Error\n");
    }

  if (tid == TRICORE_TIN_DAE)
    {
      _alert("\tData Access Asysnchronous Error\n");
    }

  if (tid == TRICORE_TIN_CAE)
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

  if (tid == TRICORE_TIN_PIE)
    {
      _alert("\tProgram Memory Integrity Error\n");
    }

  if (tid == TRICORE_TIN_DIE)
    {
      _alert("\tData Memory Integrity Error\n");
    }

  if (tid == TRICORE_TIN_TAE)
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
  if (tid == TRICORE_TIN_OVF)
    {
      _alert("\tArithmetic Overflow\n");
    }

  if (tid == TRICORE_TIN_SOVF)
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
  uint8_t tclass = ctrap->tclass;
  unsigned int tid = ctrap->tid;

  tricore_trapinfo(trap);

  regs = tricore_csa2addr(tricore_mfcr(TRICORE_CPU_PCXI));

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
      tricore_trap_return();
    }

  if (tclass == TRICORE_TRAP_CLASS_MMU)
    {
      tricore_mmutrap(tid, regs, NULL);
      tricore_trap_return();
    }

  if (tclass == TRICORE_TRAP_CLASS_PROT)
    {
      tricore_internalprotrape(tid, regs, NULL);
      tricore_trap_return();
    }

  if (tclass == TRICORE_TRAP_CLASS_INSN)
    {
      tricore_insterrorstrap(tid, regs, NULL);
      tricore_trap_return();
    }

  if (tclass == TRICORE_TRAP_CLASS_CTX)
    {
      tricore_contexmnttrap(tid, regs, NULL);
      tricore_trap_return();
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

      tricore_trap_return();
    }

  if (tclass == TRICORE_TRAP_CLASS_ASSERT)
    {
      tricore_assertiontrap(tid, regs, NULL);
      tricore_trap_return();
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
  coredump_add_memory_region(&g_trapinfo, sizeof(g_trapinfo),
                             PF_REGISTER);
#endif
}
