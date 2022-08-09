/****************************************************************************
 * arch/avr/include/avr32/irq.h
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

#ifndef __ARCH_AVR_INCLUDE_AVR32_IRQ_H
#define __ARCH_AVR_INCLUDE_AVR32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr32/avr32.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General notes about the AVR32 ABI:
 *
 * Scratch/Volatile Registers: r8-r12
 * Preserved/Static Registers: r0-r7
 * Parameter Passing:          r12-R8 (in that order)
 */

/* Register state save array indices.
 *
 * The following registers are saved by the AVR32 hardware (for the case of
 * interrupts only).  Note the registers are order in the opposite order the
 * they appear in memory (i.e., in the order of increasing address) because
 * this makes it easier to following the ordering of pushing on a push-down
 * stack.
 */

#define REG_R8           16
#define REG_R9           15
#define REG_R10          14
#define REG_R11          13
#define REG_R12          12
#define REG_R14          11
#define REG_R15          10
#define REG_SR            9

#define REG_LR           REG_R14
#define REG_PC           REG_R15

/* Additional registers saved in order have the full CPU context */

#define REG_R13           8
#define REG_SP           REG_R13

#define REG_R0            7
#define REG_R1            6
#define REG_R2            5
#define REG_R3            4
#define REG_R4            3
#define REG_R5            2
#define REG_R6            1
#define REG_R7            0

/* Size of the register state save array (in 32-bit words) */

#define INTCONTEXT_REGS   8 /* r8-r12, lr, pc, sr */
#define XCPTCONTEXT_REGS 17 /* Plus r0-r7, sp */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored. */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of PC and SR used during signal processing.
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  uint32_t saved_pc;
  uint32_t saved_sr;

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Read the AVR32 status register */

static inline uint32_t avr32_sr(void)
{
  uint32_t sr;
  __asm__ __volatile__ (
    "mfsr\t%0,%1\n\t"
    : "=r" (sr)
    : "i" (AVR32_SR)
  );
  return sr;
}

/* Read the interrupt vector base address */

static inline uint32_t avr32_evba(void)
{
  uint32_t evba;
  __asm__ __volatile__ (
    "mfsr\t%0,%1\n\t"
    : "=r" (evba)
    : "i" (AVR32_EVBA)
  );
  return evba;
}

/* Return the current value of the stack pointer */

static inline uint32_t up_getsp(void)
{
  uint32_t retval;
  __asm__ __volatile__
    (
      "mov\t%0,sp\n\t"
      : "=r" (retval)
      :
    );

  return retval;
}

/* Return the current interrupt enable state and disable all interrupts */

static inline irqstate_t up_irq_save(void)
{
  irqstate_t sr = (irqstate_t)avr32_sr();
  __asm__ __volatile__ (
    "ssrf\t%0\n\t"
    "nop\n\t"
    "nop"
    :
    : "i" (AVR32_SR_GM_SHIFT)
  );
  return sr;
}

/* Restore saved interrupt state */

static inline void up_irq_restore(irqstate_t flags)
{
  if ((flags & AVR32_SR_GM_MASK) == 0)
    {
      __asm__ __volatile__ (
        "csrf\t%0\n\t"
        "nop\n\t"
        "nop"
        :
        : "i" (AVR32_SR_GM_SHIFT)
      );
    }
}

/* Return the current interrupt enable state and enable all interrupts */

static inline irqstate_t up_irq_enable(void)
{
  irqstate_t sr = (irqstate_t)avr32_sr();
  __asm__ __volatile__ (
    "csrf\t%0\n\t"
    "nop\n\t"
    "nop"
    :
    : "i" (AVR32_SR_GM_SHIFT)
  );
  return sr;
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_AVR_INCLUDE_AVR32_IRQ_H */
