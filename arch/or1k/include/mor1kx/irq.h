/****************************************************************************
 * arch/or1k/include/mor1kx/irq.h
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_OR1K_INCLUDE_MOR1KX_IRQ_H
#define __ARCH_OR1K_INCLUDE_MOR1KX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <arch/spr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OR1K_NR_EXT_IRQ     (32)
#define OR1K_NR_INT_IRQ     (1)
#define NR_IRQS             (OR1K_NR_EXT_IRQ + OR1K_NR_INT_IRQ)

/* External interrupts are numbered 0-31. These can be used directly
 * as bit shift offsets in the programmable interrupt controller register.
 */

#define OR1K_IRQ_EXT0       (0)
#define OR1K_IRQ_EXT1       (1)
#define OR1K_IRQ_EXT2       (2)
#define OR1K_IRQ_EXT3       (3)
#define OR1K_IRQ_EXT4       (4)
#define OR1K_IRQ_EXT5       (5)
#define OR1K_IRQ_EXT6       (6)
#define OR1K_IRQ_EXT7       (7)
#define OR1K_IRQ_EXT8       (8)
#define OR1K_IRQ_EXT9       (9)
#define OR1K_IRQ_EXT10      (10)
#define OR1K_IRQ_EXT11      (11)
#define OR1K_IRQ_EXT12      (12)
#define OR1K_IRQ_EXT13      (13)
#define OR1K_IRQ_EXT14      (14)
#define OR1K_IRQ_EXT15      (15)
#define OR1K_IRQ_EXT16      (16)
#define OR1K_IRQ_EXT17      (17)
#define OR1K_IRQ_EXT18      (18)
#define OR1K_IRQ_EXT19      (19)
#define OR1K_IRQ_EXT20      (20)
#define OR1K_IRQ_EXT21      (21)
#define OR1K_IRQ_EXT22      (22)
#define OR1K_IRQ_EXT23      (23)
#define OR1K_IRQ_EXT24      (24)
#define OR1K_IRQ_EXT25      (25)
#define OR1K_IRQ_EXT26      (26)
#define OR1K_IRQ_EXT27      (27)
#define OR1K_IRQ_EXT28      (28)
#define OR1K_IRQ_EXT29      (29)
#define OR1K_IRQ_EXT30      (30)
#define OR1K_IRQ_EXT31      (31)

/* Internal interrupts are numbered 32-xx */

#define OR1K_IRQ_TICK       (32)

/* IRQ Stack Frame Format:
 *
 * We're going to store [r1..r31], pc and sr
 * into a register set for context switches
 * and exception handlers.
 *
 * Note that the PC and SR can be removed once
 * context switches are done through syscall exceptions.
 *
 * We should rely on EPCR[0-15], ESR[0-15] and shadow regs.
 *
 * Indices into the xcp.regs array:
 */

#define REG_R1              (0)
#define REG_R2              (1)
#define REG_R3              (2)
#define REG_R4              (3)
#define REG_R5              (4)
#define REG_R6              (5)
#define REG_R7              (6)
#define REG_R8              (7)
#define REG_R9              (8)
#define REG_R10             (9)
#define REG_R11             (10)
#define REG_R12             (11)
#define REG_R13             (12)
#define REG_R14             (13)
#define REG_R15             (14)
#define REG_R16             (15)
#define REG_R17             (16)
#define REG_R18             (17)
#define REG_R19             (18)
#define REG_R20             (19)
#define REG_R21             (20)
#define REG_R22             (21)
#define REG_R23             (22)
#define REG_R24             (23)
#define REG_R25             (24)
#define REG_R26             (25)
#define REG_R27             (26)
#define REG_R28             (27)
#define REG_R29             (28)
#define REG_R30             (29)
#define REG_R31             (30)
#define REG_PC              (31)
#define REG_SR              (32)

#define XCPTCONTEXT_REGS    (33)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The exception context structure defines how state is stored
 * for interrupt handlers and context switches
 *
 * All general purpose registers, the re-entry point (PC), and SR
 * are stored in the regs array.
 */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];

  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of LR and CPSR used during
   * signal processing.
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  uint32_t saved_pc;
  uint32_t saved_flags;
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
 * function should not be called directly from operating system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Save the current interrupt enable state & disable IRQs. */

static inline irqstate_t up_irq_save(void)
{
  irqstate_t flags;
  irqstate_t x;
  mfspr(SPR_SYS_SR, flags);

  /* Disable IRQs */

  x = flags & ~(SPR_SR_IEE | SPR_SR_TEE);
  mtspr(SPR_SYS_SR, x);

  return flags;
}

/* Restore saved state */

static inline void up_irq_restore(irqstate_t flags)
{
  uint32_t x;
  mfspr(SPR_SYS_SR, x);
  x |= flags & (SPR_SR_IEE | SPR_SR_TEE);
  mtspr(SPR_SYS_SR, x);
}

/* Enable IRQs */

static inline void up_irq_enable(void) inline_function;
static inline void up_irq_enable(void)
{
  irqstate_t flags;
  mfspr(SPR_SYS_SR, flags);
  flags |= (SPR_SR_IEE | SPR_SR_TEE);
  mtspr(SPR_SYS_SR, flags);
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

#endif /* __ARCH_OR1K_INCLUDE_MOR1KX_IRQ_H */
