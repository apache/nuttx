/****************************************************************************
 * arch/arm/include/irq_thumb2.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_IRQ_THUMB2_H
#define __ARCH_ARM_INCLUDE_IRQ_THUMB2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <sys/types.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format: */

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define REG_XPSR            (16) /* xPSR */
#define REG_R15             (15) /* R15 = PC */
#define REG_R14             (14) /* R14 = LR */
#define REG_R12             (13) /* R12 */
#define REG_R3              (12) /* R3 */
#define REG_R2              (11) /* R2 */
#define REG_R1              (10) /* R1 */
#define REG_R0              (9)  /* R0 */

#define HW_XCPT_REGS        (8)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

/* The following additional registers are stored by the interrupt handling
 * logic.
 */

#define REG_R13             (8) /* R13 = SP at time of interrupt */
#define REG_R11             (7) /* R11 */
#define REG_R10             (6) /* R10 */
#define REG_R9              (5) /* R9 */
#define REG_R8              (4) /* R8 */
#define REG_R7              (3) /* R7 */
#define REG_R6              (2) /* R6 */
#define REG_R5              (1) /* R5 */
#define REG_R4              (0) /* R4 */

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
#define REG_FP              REG_R11
#define REG_IP              REG_R12
#define REG_SP              REG_R13
#define REG_LR              REG_R14
#define REG_PC              REG_R15

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of LR and CPSR used during
   * signal processing.
   */

  uint32 saved_pc;
  uint32 saved_xpsr;
#endif

  /* Register save area */

  uint32 regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Save the current interrupt enable state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  unsigned short primask;

  /* Return the the current value of primask register and set
   * bit 0 of the primask register to disable interrupts
   */

  __asm__ __volatile__
    (
     "\tmrs    %0, primask\n"
     "\tcpsid  i\n"
     : "=r" (primask)
     :
     : "memory");
  return primask;
}

/* Restore saved IRQ & FIQ state */

static inline void irqrestore(irqstate_t primask)
{
  /* If bit 0 of the primask is 0, then we need to restore
   * interupts.
   */

  __asm__ __volatile__
    (
     "\ttst    %0, #1\n"
     "\tbne    1f\n"
     "\tcpsie  i\n"
     "1:\n"
     :
     : "r" (primask)
     : "memory");
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_IRQ_THUMB2_H */

