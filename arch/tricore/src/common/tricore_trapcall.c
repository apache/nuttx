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
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>

#include <arch/irq.h>
#include <arch/barriers.h>
#include <sched/sched.h>
#include <nuttx/debug.h>
#include <nuttx/coredump.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"
#include "tricore_vectors.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TriCore trap panic message contract. */

#define TRICORE_TRAP_MSG_LEN    24

#define TRICORE_FCU_CSA_MINNUM  20

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tricore_trap_s g_trapinfo;

/* Bus trap hook for safety test modules (e.g., BusMonTst). */

static tricore_bustrap_hook_t g_bustrap_hook;

/* Trap reason description tables. */

#ifdef CONFIG_TRICORE_TRAP_ALERT
static const char *const g_mmutrap_reasons[] =
{
  [TRICORE_TRAP_MMU_VAF] = "Virtual Address Fill",
  [TRICORE_TRAP_MMU_VAP] = "Virtual Address Protection",
};

static const char *const g_intprot_reasons[] =
{
  [TRICORE_TRAP_INTPROT_PRIV] = "Privileged instruction",
  [TRICORE_TRAP_INTPROT_MPR]  = "Memory Protection Read",
  [TRICORE_TRAP_INTPROT_MPW]  = "Memory Protection Write",
  [TRICORE_TRAP_INTPROT_MPX]  = "Memory Protection Execution",
  [TRICORE_TRAP_INTPROT_MPP]  = "Memory Protection Peripheral Access",
  [TRICORE_TRAP_INTPROT_MPN]  = "Memory Protection Null Address",
  [TRICORE_TRAP_INTPROT_GRWP] = "Global Register Write Protection",
};

static const char *const g_insterr_reasons[] =
{
  [TRICORE_TRAP_INSTERR_IOPC] = "Illegal Opcode",
  [TRICORE_TRAP_INSTERR_UOPC] = "Unimplemented Opcode",
  [TRICORE_TRAP_INSTERR_OPD]  = "Invalid Operand Specification",
  [TRICORE_TRAP_INSTERR_ALN]  = "Data Address Alignment",
  [TRICORE_TRAP_INSTERR_MEM]  = "Invalid Local Memory Address",
  [TRICORE_TRAP_INSTERR_CSE]  = "Coprocessor Trap Synchronous Error",
};

static const char *const g_ctxmgmt_reasons[] =
{
  [TRICORE_TRAP_CTXMGMT_FCD]  = "Free Context List Depletion",
  [TRICORE_TRAP_CTXMGMT_CDO]  = "Call Depth Overflow",
  [TRICORE_TRAP_CTXMGMT_CDU]  = "Call Depth Underflow",
  [TRICORE_TRAP_CTXMGMT_FCU]  = "Free Context List Underflow",
  [TRICORE_TRAP_CTXMGMT_CSU]  = "Call Stack Underflow",
  [TRICORE_TRAP_CTXMGMT_CTYP] = "Context Type",
  [TRICORE_TRAP_CTXMGMT_NEST] = "RFE with non-zero call depth",
};

static const char *const g_bustrap_reasons[] =
{
  [TRICORE_TRAP_BUS_PSE] = "Program Fetch Synchronous Error",
  [TRICORE_TRAP_BUS_DSE] = "Data Access Synchronous Error",
  [TRICORE_TRAP_BUS_DAE] = "Data Access Asynchronous Error",
  [TRICORE_TRAP_BUS_CAE] = "Coprocessor Trap Asynchronous Error",
  [TRICORE_TRAP_BUS_PIE] = "Program Memory Integrity Error",
  [TRICORE_TRAP_BUS_DIE] = "Data Memory Integrity Error",
  [TRICORE_TRAP_BUS_TAE] = "Temporal Asynchronous Error",
};

static const char *const g_assert_reasons[] =
{
  [TRICORE_TRAP_ASSERT_OVF]  = "Arithmetic Overflow",
  [TRICORE_TRAP_ASSERT_SOVF] = "Sticky Arithmetic Overflow",
};

static const char *const g_nmitrap_reasons[] =
{
  [TRICORE_TRAP_NMI_NMI] = "Non Maskable Interrupt",
};

static const char *const g_trapclass_names[] =
{
  [TRICORE_TRAP_CLASS_MMU]     = "Memory Management Error",
  [TRICORE_TRAP_CLASS_INTPROT] = "Internal Protection Error",
  [TRICORE_TRAP_CLASS_INSTERR] = "Instruction Error",
  [TRICORE_TRAP_CLASS_CTXMGMT] = "Context Management Error",
  [TRICORE_TRAP_CLASS_BUS]     = "System Bus Error",
  [TRICORE_TRAP_CLASS_ASSERT]  = "System Assertion",
  [TRICORE_TRAP_CLASS_NMI]     = "Non Maskable Interrupt",
};

static const char *const *const g_trapclass_reasons[] =
{
  [TRICORE_TRAP_CLASS_MMU]     = g_mmutrap_reasons,
  [TRICORE_TRAP_CLASS_INTPROT] = g_intprot_reasons,
  [TRICORE_TRAP_CLASS_INSTERR] = g_insterr_reasons,
  [TRICORE_TRAP_CLASS_CTXMGMT] = g_ctxmgmt_reasons,
  [TRICORE_TRAP_CLASS_BUS]     = g_bustrap_reasons,
  [TRICORE_TRAP_CLASS_ASSERT]  = g_assert_reasons,
  [TRICORE_TRAP_CLASS_NMI]     = g_nmitrap_reasons,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

uintptr_t *g_fcu_csa_base;
size_t g_fcu_csa_num;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_TRICORE_TRAP_ALERT
static void tricore_trap_alert(enum tricore_trap_class_e cls, uint32_t tid,
                               void *context)
{
  _alert("PANIC!!! %s Trap:\n", g_trapclass_names[cls]);
  _alert("\tclass %d TID: %" PRIu32 " regs: %p\n", cls, tid, context);
  _alert("\t%s\n", g_trapclass_reasons[cls][tid]);
}
#else
#  define tricore_trap_alert(cls, tid, context)
#endif

/* TriCore trap panic message contract. */

static void tricore_trap_panic(uint32_t cls, uint32_t tid, void *regs)
{
  char buf[TRICORE_TRAP_MSG_LEN];

  snprintf(buf, sizeof(buf), "Trap c=%" PRIu32 " t=%" PRIu32, cls, tid);
  PANIC_WITH_REGS(buf, regs);
}

static void tricore_defaulttrap(enum tricore_trap_class_e cls, uint32_t tid,
                                void *context)
{
  tricore_trap_alert(cls, tid, context);
  tricore_trap_panic(cls, tid, context);
}

static void tricore_insterrorstrap(uint32_t tid, void *context)
{
#ifdef CONFIG_ARCH_TC1V8
  if (tid == TRICORE_TRAP_INSTERR_CSE)
    {
#  ifdef CONFIG_ARCH_HAVE_FPU
      uint32_t fpu_sync = tricore_mfcr(FPU_SYNC_TRAP_REG);

      _alert("\tFPU_SYNC_TRAP:%" PRIX32 "\n\n", fpu_sync);
      tricore_mtcr(FPU_SYNC_TRAP_REG,
                   fpu_sync | (1U << FPU_TRAP_TCL_SHIFT));
      UP_ISB();
#  endif
    }
#endif

  tricore_defaulttrap(TRICORE_TRAP_CLASS_INSTERR, tid, context);
}

static int tricore_bustrap(uint32_t tid, void *context)
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

#ifdef CONFIG_ARCH_HAVE_FPU
  if (tid == TRICORE_TRAP_BUS_CAE)
    {
      uint32_t fpu_async = tricore_mfcr(FPU_ASYNC_TRAP_REG);

      _alert("\tFPU_ASYNC_TRAP:%" PRIX32 "\n\n", fpu_async);
      tricore_mtcr(FPU_ASYNC_TRAP_REG,
                   fpu_async | (1U << FPU_TRAP_TCL_SHIFT));
      UP_ISB();
    }
#endif

  tricore_defaulttrap(TRICORE_TRAP_CLASS_BUS, tid, context);

  return OK;
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

void weak_function tricore_nmitrap(uint32_t tid, void *context)
{
  tricore_defaulttrap(TRICORE_TRAP_CLASS_NMI, tid, context);
}

/****************************************************************************
 * Name: tricore_trapcall
 *
 * Description:
 *   This is Trap exception handler
 *
 ****************************************************************************/

noinstrument_function
void tricore_trapcall(void *taddr, uint32_t tid, uint32_t tclass)
{
  struct tricore_trap_s trap;
  uintptr_t *regs;
  bool from_task;

  trap.taddr  = (uintptr_t)taddr;
  trap.tid    = tid;
  trap.tclass = tclass;
  trap.tcpu   = up_cpu_index();
  trap.tpstr  = tricore_mfcr(TRICORE_CPU_PSTR);
  trap.tdstr  = tricore_mfcr(TRICORE_CPU_DSTR);
  trap.tdatr  = tricore_mfcr(TRICORE_CPU_DATR);
  trap.tdeadd = tricore_mfcr(TRICORE_CPU_DEADD);

  regs = tricore_csa2addr(tricore_mfcr(TRICORE_CPU_PCXI));

  from_task = !(tricore_csa2addr(regs[REG_LPCXI])[REG_PSW] & PSW_IS);
  if (from_task)
    {
      /* Update the current task's regs */

      g_running_tasks[this_cpu()]->xcp.regs = regs;
    }

  /* Save trap info for coredump */

  g_trapinfo = trap;

  switch (tclass)
    {
      case TRICORE_TRAP_CLASS_INSTERR:
        tricore_insterrorstrap(tid, regs);
        break;

      case TRICORE_TRAP_CLASS_BUS:
        tricore_bustrap(tid, regs);
        break;

      case TRICORE_TRAP_CLASS_NMI:
        tricore_nmitrap(tid, regs);
        break;

      default:
        tricore_defaulttrap(tclass, tid, regs);
        break;
    }

  if (from_task)
    {
      /* g_running_tasks[]->xcp.regs is about to become invalid
       * and will be marked as NULL to avoid misusage.
       */

      g_running_tasks[this_cpu()]->xcp.regs = NULL;
    }

  tricore_jump_backto_link();
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

/****************************************************************************
 * Name: tricore_fcutrap_recover
 *
 * Description:
 *   Entered from tricore_fcu_entry, after the trapped upper and lower
 *   context have been saved into the first two CSAs of the FCU pool.
 *
 *   The FCU pool registered via tricore_fcu_register_pool() may be backed
 *   in one of two ways:
 *
 *     1. Reuse a RAM block that normal business keeps occupying until the
 *        system crashes (preferred when RAM is tight), the block becomes
 *        available at trap time.
 *     2. A dedicated static buffer reserved up front (when RAM is enough).
 *
 *   Either way the LPCXI chain is built lazily here at trap time, not at
 *   registration: for case (1) writing it earlier would corrupt live
 *   business data, so the linking is deferred until normal business has
 *   ceased.
 *
 ****************************************************************************/

used_code noinstrument_function
void tricore_fcutrap_recover(void)
{
  uintptr_t *csa_iter;
  uintptr_t csa;
  size_t i;

  DEBUGASSERT(g_fcu_csa_num >= TRICORE_FCU_CSA_MINNUM);

  /* Link the pool csas into free chain. */

  csa = tricore_addr2csa(g_fcu_csa_base);
  csa_iter = &g_fcu_csa_base[2 * TC_CONTEXT_REGS];
  for (i = 2; i < g_fcu_csa_num - 1; i++)
    {
      csa_iter[REG_LPCXI] = tricore_addr2csa(csa_iter + TC_CONTEXT_REGS);
      csa_iter += TC_CONTEXT_REGS;
    }

  csa_iter[REG_LPCXI] = 0;

  g_fcu_csa_base[TC_CONTEXT_REGS + REG_LPCXI] = csa | PCXI_UL;

  tricore_mtcr(TRICORE_CPU_PCXI, csa + 1);
  tricore_mtcr(TRICORE_CPU_FCX, csa + 2);
  UP_ISB();

  tricore_jump_backto_link();
}

/****************************************************************************
 * Name: tricore_fcu_register_pool
 ****************************************************************************/

void tricore_fcu_register_pool(void *base, size_t num)
{
  DEBUGASSERT(base != NULL);
  DEBUGASSERT(((uintptr_t)base & (TC_CONTEXT_SIZE - 1)) == 0);
  DEBUGASSERT(num >= TRICORE_FCU_CSA_MINNUM);

  g_fcu_csa_base = (uintptr_t *)base;
  g_fcu_csa_num  = num;
}
