/****************************************************************************
 * arch/rh850/include/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_RH850_INCLUDE_IRQ_H
#define __ARCH_RH850_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>
#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register save area: the frame stored by interrupt and exception entry.
 * REG_SP is the stack pointer of the saved context (before the frame push).
 */

#define REG_R1              0
#define REG_R2              1
#define REG_R4              2
#define REG_R5              3
#define REG_R6              4
#define REG_R7              5
#define REG_R8              6
#define REG_R9              7
#define REG_R10             8
#define REG_R11             9
#define REG_R12             10
#define REG_R13             11
#define REG_R14             12
#define REG_R15             13
#define REG_R16             14
#define REG_R17             15
#define REG_R18             16
#define REG_R19             17
#define REG_R20             18
#define REG_R21             19
#define REG_R22             20
#define REG_R23             21
#define REG_R24             22
#define REG_R25             23
#define REG_R26             24
#define REG_R27             25
#define REG_R28             26
#define REG_R29             27
#define REG_R30             28
#define REG_R31             29
#define REG_EIPC            30
#define REG_EIPSW           31
#define REG_FPSR            32
#define REG_FPEPC           33
#define REG_SP              34

#define XCPTCONTEXT_REGS    35
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#define REG_LP              REG_R31
#define REG_EP              REG_R30

/* The stack pointer is kept 4-byte aligned (GCC __BIGGEST_ALIGNMENT__ 4) */

#define STACKFRAME_ALIGN    4

#define PSW_ID              (1 << 5)    /* EI level interrupts disabled */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct xcptcontext
{
#ifdef CONFIG_ENABLE_ALL_SIGNALS
  /* Saved copy of the context used during signal processing */

  uintptr_t *saved_regs;
#endif

  /* Register save area of XCPTCONTEXT_SIZE, valid while the task is not
   * running or is interrupted.
   */

  uintptr_t *regs;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile bool g_interrupt_context;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function uintptr_t up_getsp(void)
{
  uintptr_t sp;

  __asm__ __volatile__("mov sp, %0" : "=r"(sp));
  return sp;
}

/* Save the current interrupt state (PSW) and disable EI level interrupts */

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  __asm__ __volatile__("stsr 5, %0, 0\n\tdi" : "=r"(flags) : : "memory");
  return flags;
}

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
  if ((flags & PSW_ID) == 0)
    {
      __asm__ __volatile__("ei" : : : "memory");
    }
}

noinstrument_function
static inline_function void up_set_interrupt_context(bool flag)
{
  g_interrupt_context = flag;
}

noinstrument_function static inline_function bool up_interrupt_context(void)
{
  return g_interrupt_context;
}

static inline_function uintptr_t up_getusrsp(void *regs)
{
  return ((uintptr_t *)regs)[REG_SP];
}

static inline_function uintptr_t up_getusrpc(void *regs)
{
  return ((uintptr_t *)regs)[REG_EIPC];
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void up_irq_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RH850_INCLUDE_IRQ_H */
