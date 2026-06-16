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

#include <nuttx/arch.h>
#include <nuttx/compiler.h>

#include <arch/irq.h>

#include "tricore_vectors.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(__TASKING__)
#define TRICORE_TRAP_CALLTSR(serviceRoutine)      \
{                                                 \
  __asm__(".align 32\n\t");                       \
  __asm__("svlcx\n\t");                           \
  __asm__("mov d4, d15\n\t");                     \
  __asm__("ji %0\n\t" : : "a" (serviceRoutine));  \
  __asm__("rfe");                                 \
}

#define TRICORE_TRAP_TSRHOOK(trap, handler)       \
{                                                 \
  __asm__("mov.aa a4, %0" :: "a"(trap) : "a4");   \
  __asm__("jli %0" :: "a"(handler));              \
}

#else
#define TRICORE_TRAP_CALLTSR(serviceRoutine)      \
{                                                 \
  __asm__(".align 5\n\t");                        \
  __asm__("svlcx\n\t");                           \
  __asm__("mov %d4, %d15\n\t");                   \
  __asm__("ji %0\n\t" : : "a" (serviceRoutine));  \
  __asm__("rfe");                                 \
}

#define TRICORE_TRAP_TSRHOOK(trap, handler)       \
{                                                 \
  __asm__("mov.aa %%a4, %0" :: "a"(trap) : "a4"); \
  __asm__("jli %0" :: "a"(handler));              \
}
#endif

#define TRICORE_TRAP_RET()  __asm__("rslcx\n\trfe")

/****************************************************************************
 * Private Functions
 ****************************************************************************/

noinstrument_function inline_function
static void tricore_trap_class(uint32_t tin, uint8_t tclass, void *handler)
{
  struct tricore_trap_s trap;

  trap.tAddr  = (uintptr_t)__getA11();
  trap.tClass = tclass;
  trap.tId    = tin;
  trap.tCpu   = up_cpu_index();

  TRICORE_TRAP_TSRHOOK(&trap, handler);
}

noinstrument_function
static void tricore_trap_memory_management(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_MMU,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_internal_protection(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_INTPROT,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_instruction(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_INSTERR,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_context_management(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_CTXMGMT,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_bus(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_BUS,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_assertion(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_ASSERT,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_system_call(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_SYSCALL,
                     (void *)tricore_svcall);
  TRICORE_TRAP_RET();
}

noinstrument_function
static void tricore_trap_non_maskable_interrupt(uint32_t tin)
{
  tricore_trap_class(tin, TRICORE_TRAP_CLASS_NMI,
                     (void *)tricore_trapcall);
  TRICORE_TRAP_RET();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * TriCore TRAP Vector Table (BTV)
 ****************************************************************************/

#if defined(__TASKING__)
noinstrument_function locate_code(".text.traptab")
#else
noinstrument_function locate_code(".traptab")
#endif
void tricore_traptab(void)
{
  TRICORE_TRAP_CALLTSR(tricore_trap_memory_management)
  TRICORE_TRAP_CALLTSR(tricore_trap_internal_protection)
  TRICORE_TRAP_CALLTSR(tricore_trap_instruction)
  TRICORE_TRAP_CALLTSR(tricore_trap_context_management)
  TRICORE_TRAP_CALLTSR(tricore_trap_bus)
  TRICORE_TRAP_CALLTSR(tricore_trap_assertion)
  TRICORE_TRAP_CALLTSR(tricore_trap_system_call)
  TRICORE_TRAP_CALLTSR(tricore_trap_non_maskable_interrupt)
}
