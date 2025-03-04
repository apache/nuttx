/****************************************************************************
 * arch/arm/include/arm/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_ARM_IRQ_H
#define __ARCH_ARM_INCLUDE_ARM_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ Stack Frame Format:
 *
 * Context is always saved/restored in the same way:
 *
 *   (1) stmia rx, {r0-r14}
 *   (2) then the PC and CPSR
 *
 * This results in the following set of indices that
 * can be used to access individual registers in the
 * xcp.regs array:
 */

#define REG_R0              (0)
#define REG_R1              (1)
#define REG_R2              (2)
#define REG_R3              (3)
#define REG_R4              (4)
#define REG_R5              (5)
#define REG_R6              (6)
#define REG_R7              (7)
#define REG_R8              (8)
#define REG_R9              (9)
#define REG_R10             (10)
#define REG_R11             (11)
#define REG_R12             (12)
#define REG_R13             (13)
#define REG_R14             (14)
#define REG_R15             (15)
#define REG_CPSR            (16)

#define XCPTCONTEXT_REGS    (17)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#define REG_A1              REG_R0
#define REG_A2              REG_R1
#define REG_A3              REG_R2
#define REG_A4              REG_R3
#define REG_V1              REG_R4
#define REG_V2              REG_R5
#define REG_V3              REG_R6
#define REG_V4              REG_R7
#define REG_V5              REG_R8
#define REG_V6              REG_R9
#define REG_V7              REG_R10
#define REG_SB              REG_R9
#define REG_SL              REG_R10
#ifdef CONFIG_ARM_THUMB
  #define REG_FP            REG_R7
#else
  #define REG_FP            REG_R11
#endif /* CONFIG_ARM_THUMB */
#define REG_IP              REG_R12
#define REG_SP              REG_R13
#define REG_LR              REG_R14
#define REG_PC              REG_R15

/* The PIC register is usually R10. It can be R9 is stack checking is enabled
 * or if the user changes it with -mpic-register on the GCC command line.
 */

#define REG_PIC             REG_R10

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored.  We
 * need to save:
 *
 *  1  CPSR
 *  7  Static registers, v1-v7 (aka r4-r10)
 *  1  Frame pointer, fp (aka r11)
 *  1  Stack pointer, sp (aka r13)
 *  1  Return address, lr (aka r14)
 * ---
 * 11  (XCPTCONTEXT_USER_REG)
 *
 * On interrupts, we also need to save:
 *  4  Volatile registers, a1-a4 (aka r0-r3)
 *  1  Scratch Register, ip (aka r12)
 * ---
 *  5  (XCPTCONTEXT_IRQ_REGS)
 *
 * For a total of 17 (XCPTCONTEXT_REGS)
 */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* These are saved copies of the context used during
   * signal processing.
   */

  uint32_t *saved_regs;

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uint32_t *regs;

  /* Extra fault address register saved for common paging logic.  In the
   * case of the prefetch abort, this value is the same as regs[REG_R15];
   * For the case of the data abort, this value is the value of the fault
   * address register (FAR) at the time of data abort exception.
   */

#ifdef CONFIG_LEGACY_PAGING
  uintptr_t far;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_interrupt_context store irq status */

extern volatile bool g_interrupt_context[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Save the current interrupt enable state & disable IRQs. */

static inline_function irqstate_t up_irq_save(void)
{
  unsigned int flags;
  unsigned int temp;
  __asm__ __volatile__
    (
     "\tmrs    %0, cpsr\n"
     "\torr    %1, %0, #128\n"
     "\tmsr    cpsr_c, %1"
     : "=r" (flags), "=r" (temp)
     :
     : "cc", "memory");
  return flags;
}

/* Restore saved IRQ & FIQ state */

static inline_function void up_irq_restore(irqstate_t flags)
{
  __asm__ __volatile__
    (
     "msr    cpsr_c, %0"
     :
     : "r" (flags)
     : "cc", "memory");
}

/* Enable IRQs and return the previous IRQ state */

static inline_function irqstate_t up_irq_enable(void)
{
  unsigned int flags;
  unsigned int temp;
  __asm__ __volatile__
    (
     "\tmrs    %0, cpsr\n"
     "\tbic    %1, %0, #128\n"
     "\tmsr    cpsr_c, %1"
     : "=r" (flags), "=r" (temp)
     :
     : "cc", "memory");
  return flags;
}

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_MULTICPU
int up_cpu_index(void) noinstrument_function;
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

noinstrument_function
static inline_function bool up_interrupt_context(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
  bool ret = g_interrupt_context[up_cpu_index()];
  up_irq_restore(flags);
  return ret;
#else
  return g_interrupt_context[0];
#endif
}

noinstrument_function
static inline_function void up_set_interrupt_context(bool flag)
{
#ifdef CONFIG_ARCH_HAVE_MULTICPU
  g_interrupt_context[up_cpu_index()] = flag;
#else
  g_interrupt_context[0] = flag;
#endif
}

static inline_function uint32_t up_getsp(void)
{
  register uint32_t sp;

  __asm__ __volatile__
  (
    "mov %0, sp\n"
    : "=r" (sp)
  );

  return sp;
}

static inline_function uintptr_t up_getusrsp(void *regs)
{
  uint32_t *ptr = (uint32_t *)regs;
  return ptr[REG_SP];
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif  /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_ARM_IRQ_H */
