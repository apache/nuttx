/****************************************************************************
 * arch/tricore/src/common/tricore_vectors.c
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

#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>

#include <arch/barriers.h>
#include <arch/irq.h>

#include "tricore_internal.h"
#include "tricore_vectors.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(__TASKING__)
#define TRICORE_TRAP_CALLTSR(serviceroutine, tclass)        \
{                                                           \
  __asm__(".align 32\n\t");                                 \
  __asm__("svlcx\n\t");                                     \
  __asm__("mov d4, d15\n\t");                               \
  __asm__("mov d5, %0\n\t" :: "i" (tclass));                \
  __asm__("movh.a a2, #@his("#serviceroutine ")\n\t");      \
  __asm__("lea a2, [a2]@los("#serviceroutine ")\n\t");      \
  __asm__("ji a2\n\t");                                     \
}

#define TRICORE_TRAP_CALLCSATSR(serviceroutine, tclass)     \
{                                                           \
  __asm__(".align 32\n\t");                                 \
  __asm__("jeq d15, #4, tricore_fcutrap_entry\n\t");        \
  __asm__("svlcx\n\t");                                     \
  __asm__("mov d4, d15\n\t");                               \
  __asm__("mov d5, %0\n\t" :: "i" (tclass));                \
  __asm__("movh.a a2, #@his("#serviceroutine ")\n\t");      \
  __asm__("lea a2, [a2]@los("#serviceroutine ")\n\t");      \
  __asm__("ji a2\n\t");                                     \
}

#define TRICORE_TRAP_TSRHOOK(trap, handler)                 \
{                                                           \
  __asm__("mov.aa a4, %0" :: "a"(trap) : "a4");             \
  __asm__("jli %0" :: "a"(handler));                        \
  __asm__("rslcx\n\trfe");                                  \
}

#else
#define TRICORE_TRAP_CALLTSR(serviceroutine, tclass)        \
{                                                           \
  __asm__(".align 5\n\t");                                  \
  __asm__("svlcx\n\t");                                     \
  __asm__("mov %d4, %d15\n\t");                             \
  __asm__("mov %%d5, %0\n\t" :: "i" (tclass));              \
  __asm__("movh.a %a2, hi:"#serviceroutine "\n\t");         \
  __asm__("lea %a2, [%a2]lo:"#serviceroutine "\n\t");       \
  __asm__("ji %a2\n\t");                                    \
}

#define TRICORE_TRAP_CALLCSATSR(serviceroutine, tclass)     \
{                                                           \
  __asm__(".align 5\n\t");                                  \
  __asm__("jeq %d15, 4, tricore_fcutrap_entry\n\t");        \
  __asm__("svlcx\n\t");                                     \
  __asm__("mov %d4, %d15\n\t");                             \
  __asm__("mov %%d5, %0\n\t" :: "i" (tclass));              \
  __asm__("movh.a %a2, hi:"#serviceroutine "\n\t");         \
  __asm__("lea %a2, [%a2]lo:"#serviceroutine "\n\t");       \
  __asm__("ji %a2\n\t");                                    \
}

#define TRICORE_TRAP_TSRHOOK(trap, handler)                 \
{                                                           \
  __asm__("mov.aa %%a4, %0" :: "a"(trap) : "a4");           \
  __asm__("jli %0" :: "a"(handler));                        \
  __asm__("rslcx\n\trfe");                                  \
}
#endif

#define TRICORE_FCU_CSA_MINNUM  20

/****************************************************************************
 * Private Data
 ****************************************************************************/

used_data static uintptr_t *g_fcu_csa_base;
static size_t g_fcu_csa_num;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

noinstrument_function
static void tricore_trap_class(struct tricore_trap_s *trap, uintptr_t tpc,
                               uint8_t tclass, uint8_t tin)
{
  trap->taddr  = tpc;
  trap->tclass = tclass;
  trap->tid    = tin;
  trap->tcpu   = up_cpu_index();

  trap->tpstr  = tricore_mfcr(CPU_PSTR);
  trap->tdstr  = tricore_mfcr(CPU_DSTR);
  trap->tdatr  = tricore_mfcr(CPU_DATR);
  trap->tdeadd = tricore_mfcr(CPU_DEADD);
}

used_code noinstrument_function
static void tricore_trap_trapcall(uint8_t tin, uint8_t tclass)
{
  struct tricore_trap_s trap;

  tricore_trap_class(&trap, (uintptr_t)__getA11(), tclass, tin);
  TRICORE_TRAP_TSRHOOK(&trap, tricore_trapcall);
}

used_code noinstrument_function
static void tricore_trap_svcall(uint8_t tin, uint8_t tclass)
{
  struct tricore_trap_s trap;

  tricore_trap_class(&trap, (uintptr_t)__getA11(), tclass, tin);
  TRICORE_TRAP_TSRHOOK(&trap, tricore_svcall);
}

/****************************************************************************
 * Name: tricore_fcutrap_recover
 *
 * Description:
 *   The FCU pool registered via tricore_fcu_register_pool() may be backed
 *   in one of two ways:
 *
 *     1. Reuse a RAM block that normal business keeps occupying until the
 *        system crashes (preferred when RAM is tight) - the block becomes
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
static void tricore_fcutrap_recover(uint8_t tin, uint8_t tclass)
{
  uintptr_t *csa_iter;
  uintptr_t csa;
  size_t i;

  DEBUGASSERT(g_fcu_csa_base != NULL);
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

  tricore_mtcr(CPU_PCXI, csa + 1);
  tricore_mtcr(CPU_FCX, csa + 2);
  UP_ISB();

  tricore_trap_trapcall(tin, tclass);
}

/****************************************************************************
 * Name: tricore_fcutrap_entry
 *
 * Description:
 *   BTV trap table requires each entry to fit in 32 bytes, embedding fcu
 *   assembly may exceed this limit, so keep it as a separate function.
 *
 ****************************************************************************/

used_code noinstrument_function
static void tricore_fcutrap_entry(void)
{
  __asm__ volatile
  (
#if defined(__TASKING__)
    "movh.a  a4, #@his(g_fcu_csa_base)\n\t"
    "lea     a4, [a4]@los(g_fcu_csa_base)\n\t"
    "ld.a    a4, [a4]0\n\t"
    "stucx   [a4]0\n\t"
    "stlcx   [a4]64\n\t"
    "mov     d4, d15\n\t"
    "mov     d5, %0\n\t"
    "j       tricore_fcutrap_recover\n\t"
#else
    "movh.a  %%a4, hi:g_fcu_csa_base\n\t"
    "lea     %%a4, [%%a4]lo:g_fcu_csa_base\n\t"
    "ld.a    %%a4, [%%a4]0\n\t"
    "stucx   [%%a4]0\n\t"
    "stlcx   [%%a4]64\n\t"
    "mov     %%d4, %%d15\n\t"
    "mov     %%d5, %0\n\t"
    "j       tricore_fcutrap_recover\n\t"
#endif
    :: "i"(TRICORE_TRAP_CLASS_CTXMGMT)
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: tricore_traptab
 *
 * Description:
 *   Trap vector table for tricore arch.
 *
 ****************************************************************************/

#if defined(__TASKING__)
noinstrument_function locate_code(".text.traptab")
#else
noinstrument_function locate_code(".traptab")
#endif
void tricore_traptab(void)
{
  TRICORE_TRAP_CALLTSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_MMU)
  TRICORE_TRAP_CALLTSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_INTPROT)
  TRICORE_TRAP_CALLTSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_INSTERR)
  TRICORE_TRAP_CALLCSATSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_CTXMGMT)
  TRICORE_TRAP_CALLTSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_BUS)
  TRICORE_TRAP_CALLTSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_ASSERT)
  TRICORE_TRAP_CALLTSR(tricore_trap_svcall, TRICORE_TRAP_CLASS_SYSCALL)
  TRICORE_TRAP_CALLTSR(tricore_trap_trapcall, TRICORE_TRAP_CLASS_NMI)
}
