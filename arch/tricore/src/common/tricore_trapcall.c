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
#include <stdio.h>
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

/* Panic message of a trap, "Trap c=<class> t=<TIN>" */

#define TRICORE_TRAP_MSG_LEN      24

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Last trap and the trap status registers, for the coredump */

struct tricore_trapinfo_s
{
  struct tricore_trap_s trap;
  uint32_t pstr;   /* Program synchronous trap status */
  uint32_t dstr;   /* Data synchronous trap status */
  uint32_t datr;   /* Data asynchronous trap */
  uint32_t deadd;  /* Data error address */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tricore_trapinfo_s g_trapinfo;

/* FCU pool, also read by tricore_fcu_entry in tricore_vectors.S/.asm */

uintptr_t *g_tricore_fcu_pool;
static size_t g_tricore_fcu_num;

#if CONFIG_TRICORE_FCU_CSA_NUM > 0
static uintptr_t g_fcu_csa[CONFIG_TRICORE_FCU_CSA_NUM * TC_CONTEXT_REGS]
  aligned_data(TC_CONTEXT_SIZE);
#endif

/* Bus trap hook for safety test modules (e.g., BusMonTst). */

static tricore_bustrap_hook_t g_bustrap_hook;

/* Trap names and reasons, indexed by trap class and TIN */

#ifdef CONFIG_TRICORE_TRAP_ALERT
static const char * const g_trap_names[8] =
{
  [TRICORE_TRAP_CLASS_MMU]    = "MMU",
  [TRICORE_TRAP_CLASS_PROT]   = "Internal Protection",
  [TRICORE_TRAP_CLASS_INSN]   = "Instruction Errors",
  [TRICORE_TRAP_CLASS_CTX]    = "Context Management",
  [TRICORE_TRAP_CLASS_BUS]    = "System Bus",
  [TRICORE_TRAP_CLASS_ASSERT] = "Assertion",
  [TRICORE_TRAP_CLASS_NMI]    = "NMI",
};

static const char * const g_trap_reasons[8][8] =
{
  [TRICORE_TRAP_CLASS_MMU] =
  {
    [TRICORE_TIN_VAF]  = "Virtual Address Fill",
    [TRICORE_TIN_VAP]  = "Virtual Address Protection",
  },
  [TRICORE_TRAP_CLASS_PROT] =
  {
    [TRICORE_TIN_PRIV] = "Privileged Instruction",
    [TRICORE_TIN_MPR]  = "Memory Protection Read",
    [TRICORE_TIN_MPW]  = "Memory Protection Write",
    [TRICORE_TIN_MPX]  = "Memory Protection Execution",
    [TRICORE_TIN_MPP]  = "Memory Protection Peripheral Access",
    [TRICORE_TIN_MPN]  = "Memory Protection Null Address",
    [TRICORE_TIN_GRWP] = "Global Register Write Protection",
  },
  [TRICORE_TRAP_CLASS_INSN] =
  {
    [TRICORE_TIN_IOPC] = "Illegal Opcode",
    [TRICORE_TIN_UOPC] = "Unimplemented Opcode",
    [TRICORE_TIN_OPD]  = "Invalid Operand Specification",
    [TRICORE_TIN_ALN]  = "Data Address Alignment",
    [TRICORE_TIN_MEM]  = "Invalid Local Memory Address",
    [TRICORE_TIN_CSE]  = "Coprocessor Trap Synchronous Error",
  },
  [TRICORE_TRAP_CLASS_CTX] =
  {
    [TRICORE_TIN_FCD]  = "Free Context List Depletion",
    [TRICORE_TIN_CDO]  = "Call Depth Overflow",
    [TRICORE_TIN_CDU]  = "Call Depth Underflow",
    [TRICORE_TIN_FCU]  = "Free Context List Underflow",
    [TRICORE_TIN_CSU]  = "Call Stack Underflow",
    [TRICORE_TIN_CTYP] = "Context Type",
    [TRICORE_TIN_NEST] = "Nesting Error:RFE with non-zero call depth",
  },
  [TRICORE_TRAP_CLASS_BUS] =
  {
    [TRICORE_TIN_PSE]  = "Program Fetch Synchronous Error",
    [TRICORE_TIN_DSE]  = "Data Access Synchronous Error",
    [TRICORE_TIN_DAE]  = "Data Access Asynchronous Error",
    [TRICORE_TIN_CAE]  = "Coprocessor Trap Asynchronous Error",
    [TRICORE_TIN_PIE]  = "Program Memory Integrity Error",
    [TRICORE_TIN_DIE]  = "Data Memory Integrity Error",
    [TRICORE_TIN_TAE]  = "Temporal Asynchronous Error",
  },
  [TRICORE_TRAP_CLASS_ASSERT] =
  {
    [TRICORE_TIN_OVF]  = "Arithmetic Overflow",
    [TRICORE_TIN_SOVF] = "Sticky Arithmetic Overflow",
  },
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tricore_trapinfo(volatile void *trap)
{
  g_trapinfo.trap  = *(struct tricore_trap_s *)trap;
  g_trapinfo.pstr  = tricore_mfcr(TRICORE_CPU_PSTR);
  g_trapinfo.dstr  = tricore_mfcr(TRICORE_CPU_DSTR);
  g_trapinfo.datr  = tricore_mfcr(TRICORE_CPU_DATR);
  g_trapinfo.deadd = tricore_mfcr(TRICORE_CPU_DEADD);
}

/* The message tells a panic notifier which trap it was.  The buffer is on
 * the stack of the trapping CPU, which the panic does not return to.
 */

static void tricore_trap_panic(unsigned int tclass, uint32_t tid,
                               void *regs)
{
  char msg[TRICORE_TRAP_MSG_LEN];

#ifdef CONFIG_TRICORE_TRAP_ALERT
  const char *name = g_trap_names[tclass & 7];

  _alert("PANIC!!! %s Trap:\n", name != NULL ? name : "Unknown");
  _alert("\tClass %u TID: %" PRIu32 " regs: %p\n", tclass, tid, regs);
  if (tid < 8 && g_trap_reasons[tclass & 7][tid] != NULL)
    {
      _alert("\t%s\n", g_trap_reasons[tclass & 7][tid]);
    }
#endif

  snprintf(msg, sizeof(msg), "Trap c=%u t=%" PRIu32, tclass, tid);
  up_irq_save();
  PANIC_WITH_REGS(msg, regs);
}

/* Report and clear the FPU trap status of a coprocessor trap */

static void tricore_trap_fpu(unsigned int tclass, uint32_t tid)
{
#ifdef CONFIG_ARCH_HAVE_FPU
#  ifdef CONFIG_ARCH_TC1V8
  if (tclass == TRICORE_TRAP_CLASS_INSN && tid == TRICORE_TIN_CSE)
    {
      _alert("\tFPU_SYNC_TRAP:%" PRIX32 "\n\n",
             tricore_mfcr(FPU_SYNC_TRAP_REG));
      tricore_mtcr(FPU_SYNC_TRAP_REG, tricore_mfcr(FPU_SYNC_TRAP_REG) |
                                      (1U << FPU_TRAP_TCL_SHIFT));
    }
#  endif

  if (tclass == TRICORE_TRAP_CLASS_BUS && tid == TRICORE_TIN_CAE)
    {
      _alert("\tFPU_ASYNC_TRAP:%" PRIX32 "\n\n",
             tricore_mfcr(FPU_ASYNC_TRAP_REG));
      tricore_mtcr(FPU_ASYNC_TRAP_REG, tricore_mfcr(FPU_ASYNC_TRAP_REG) |
                                       (1U << FPU_TRAP_TCL_SHIFT));
      UP_ISB();
    }
#endif
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
  tricore_trap_panic(TRICORE_TRAP_CLASS_NMI, tid, context);
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
      tricore_jump_backto_link();
    }

  /* A bus trap hook of a safety test module (e.g., BusMonTst) may handle
   * an expected bus error.
   */

  if (tclass == TRICORE_TRAP_CLASS_BUS && g_bustrap_hook != NULL &&
      g_bustrap_hook(tid, regs) == OK)
    {
      if (!was_interrupt_context)
        {
          up_set_interrupt_context(false);

          /* g_running_tasks[]->xcp.regs is about to become invalid
           * and will be marked as NULL to avoid misusage.
           */

          g_running_tasks[this_cpu()]->xcp.regs = NULL;
        }

      tricore_jump_backto_link();
    }

  tricore_trap_fpu(tclass, tid);
  tricore_trap_panic(tclass, tid, regs);
}

/****************************************************************************
 * Name: tricore_fcu_register_pool
 *
 * Description:
 *   Register the context save areas that the free context list underflow
 *   (FCU) trap uses to report the trap when no free CSA is left.  The pool
 *   is linked only when the trap happens, so it may also be RAM that is in
 *   use until then.
 *
 * Input Parameters:
 *   base - 64-byte aligned start of the pool
 *   num  - Number of context save areas, at least 20: two for the trapped
 *          context, the rest for the panic path
 *
 ****************************************************************************/

void tricore_fcu_register_pool(void *base, size_t num)
{
  DEBUGASSERT(base != NULL);
  DEBUGASSERT(((uintptr_t)base & (TC_CONTEXT_SIZE - 1)) == 0);

  g_tricore_fcu_num  = num;
  g_tricore_fcu_pool = base;
}

/****************************************************************************
 * Name: tricore_fcu_recover
 *
 * Description:
 *   Entered by a jump from tricore_fcu_entry, after the trapped upper and
 *   lower context were saved into the first two CSAs of the FCU pool.
 *   Link the rest of the pool into the free context list, point PCXI at
 *   the saved lower context and report the trap through the normal trap
 *   entry, as a context management trap with TIN 4.
 *
 ****************************************************************************/

noinstrument_function void tricore_fcu_recover(void)
{
  uintptr_t *csa = g_tricore_fcu_pool;
  size_t i;

  for (i = 2; i < g_tricore_fcu_num - 1; i++)
    {
      csa[i * TC_CONTEXT_REGS + REG_LPCXI] =
        tricore_addr2csa(&csa[(i + 1) * TC_CONTEXT_REGS]);
    }

  csa[(g_tricore_fcu_num - 1) * TC_CONTEXT_REGS + REG_LPCXI] = 0;
  csa[TC_CONTEXT_REGS + REG_LPCXI] = tricore_addr2csa(csa) | PCXI_UL;

  tricore_mtcr(TRICORE_CPU_PCXI, tricore_addr2csa(&csa[TC_CONTEXT_REGS]));
  tricore_mtcr(TRICORE_CPU_FCX,
               tricore_addr2csa(&csa[2 * TC_CONTEXT_REGS]));
  UP_ISB();

  /* A11 is the trap return address, from the saved upper context */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
  __asm__ volatile ("ld.a a11, [%0]12\n\t"
                    "mov d15, #4\n\t"
                    "mov d5, #3\n\t"
                    "j tricore_trap_entry\n\t" : : "a"(csa));
#else
  __asm__ volatile ("ld.a %%a11, [%0]12\n\t"
                    "mov %%d15, 4\n\t"
                    "mov %%d5, 3\n\t"
                    "j tricore_trap_entry\n\t" : : "a"(csa));
#endif

  for (; ; );
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
#if CONFIG_TRICORE_FCU_CSA_NUM > 0
  tricore_fcu_register_pool(g_fcu_csa, CONFIG_TRICORE_FCU_CSA_NUM);
#endif

#ifdef CONFIG_COREDUMP
  coredump_add_memory_region(&g_trapinfo, sizeof(g_trapinfo),
                             PF_REGISTER);
#endif
}
