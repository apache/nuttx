/****************************************************************************
 * arch/misoc/include/lm32/irq.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Ramtin Amin <keytwo@gmail.com>
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

#ifndef __ARCH_MISOC_INCLUDE_LM32_IRQ_H
#define __ARCH_MISOC_INCLUDE_LM32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32 True interrupts plus the sofware interrupt */

#define LM32_NINTERRUPTS 32
#define LM32_IRQ_SWINT   32
#define NR_IRQS          33

/* Registers */

#define REG_X0_NDX    0      /* Holds the value zero */
#define REG_X1_NDX    1      /* General-purpose/argument 0/return value 0 */
#define REG_X2_NDX    2      /* General-purpose/argument 1/return value 1 */
#define REG_X3_NDX    3      /* General-purpose/argument 2 */
#define REG_X4_NDX    4      /* General-purpose/argument 3 */
#define REG_X5_NDX    5      /* General-purpose/argument 4 */
#define REG_X6_NDX    6      /* General-purpose/argument 5 */
#define REG_X7_NDX    7      /* General-purpose/argument 6 */
#define REG_X8_NDX    8      /* General-purpose/argument 7 */
#define REG_X9_NDX    9      /* General-purpose */
#define REG_X10_NDX   10     /* General-purpose */
#define REG_X11_NDX   11     /* General-purpose */
#define REG_X12_NDX   12     /* General-purpose */
#define REG_X13_NDX   13     /* General-purpose */
#define REG_X14_NDX   14     /* General-purpose */
#define REG_X15_NDX   15     /* General-purpose */
#define REG_X16_NDX   16     /* General-purpose */
#define REG_X17_NDX   17     /* General-purpose */
#define REG_X18_NDX   18     /* General-purpose */
#define REG_X19_NDX   19     /* General-purpose */
#define REG_X20_NDX   20     /* General-purpose */
#define REG_X21_NDX   21     /* General-purpose */
#define REG_X22_NDX   22     /* General-purpose */
#define REG_X23_NDX   23     /* General-purpose */
#define REG_X24_NDX   24     /* General-purpose */
#define REG_X25_NDX   25     /* General-purpose */
#define REG_X26_NDX   26     /* General-purpose/global pointer */
#define REG_X27_NDX   27     /* General-purpose/frame pointer */
#define REG_X28_NDX   28     /* Stack pointer */
#define REG_X29_NDX   29     /* General-purpose/return address */
#define REG_X30_NDX   30     /* Exception address */
#define REG_X31_NDX   31     /* Breakpoint address */
#define REG_X32_NDX   32     /* Reg IE */

/* Interrupt Context register */

#define XCPTCONTEXT_REGS  33
#define XCPTCONTEXT_SIZE  (4 * XCPTCONTEXT_REGS)

#ifdef __ASSEMBLY__
#  define REG_X0      (4*REG_X0_NDX)
#  define REG_X1      (4*REG_X1_NDX)
#  define REG_X2      (4*REG_X2_NDX)
#  define REG_X3      (4*REG_X3_NDX)
#  define REG_X4      (4*REG_X4_NDX)
#  define REG_X5      (4*REG_X5_NDX)
#  define REG_X6      (4*REG_X6_NDX)
#  define REG_X7      (4*REG_X7_NDX)
#  define REG_X8      (4*REG_X8_NDX)
#  define REG_X9      (4*REG_X9_NDX)
#  define REG_X10     (4*REG_X10_NDX)
#  define REG_X11     (4*REG_X11_NDX)
#  define REG_X12     (4*REG_X12_NDX)
#  define REG_X13     (4*REG_X13_NDX)
#  define REG_X14     (4*REG_X14_NDX)
#  define REG_X15     (4*REG_X15_NDX)
#  define REG_X16     (4*REG_X16_NDX)
#  define REG_X17     (4*REG_X17_NDX)
#  define REG_X18     (4*REG_X18_NDX)
#  define REG_X19     (4*REG_X19_NDX)
#  define REG_X20     (4*REG_X20_NDX)
#  define REG_X21     (4*REG_X21_NDX)
#  define REG_X22     (4*REG_X22_NDX)
#  define REG_X23     (4*REG_X23_NDX)
#  define REG_X24     (4*REG_X24_NDX)
#  define REG_X25     (4*REG_X25_NDX)
#  define REG_X26     (4*REG_X26_NDX)
#  define REG_X27     (4*REG_X27_NDX)
#  define REG_X28     (4*REG_X28_NDX)
#  define REG_X29     (4*REG_X29_NDX)
#  define REG_X30     (4*REG_X30_NDX)
#  define REG_X31     (4*REG_X31_NDX)
#  define REG_INT_CTX (4*REG_X32_NDX)
#else
#  define REG_X0       REG_X0_NDX
#  define REG_X1       REG_X1_NDX
#  define REG_X2       REG_X2_NDX
#  define REG_X3       REG_X3_NDX
#  define REG_X4       REG_X4_NDX
#  define REG_X5       REG_X5_NDX
#  define REG_X6       REG_X6_NDX
#  define REG_X7       REG_X7_NDX
#  define REG_X8       REG_X8_NDX
#  define REG_X9       REG_X9_NDX
#  define REG_X10      REG_X10_NDX
#  define REG_X11      REG_X11_NDX
#  define REG_X12      REG_X12_NDX
#  define REG_X13      REG_X13_NDX
#  define REG_X14      REG_X14_NDX
#  define REG_X15      REG_X15_NDX
#  define REG_X16      REG_X16_NDX
#  define REG_X17      REG_X17_NDX
#  define REG_X18      REG_X18_NDX
#  define REG_X19      REG_X19_NDX
#  define REG_X20      REG_X20_NDX
#  define REG_X21      REG_X21_NDX
#  define REG_X22      REG_X22_NDX
#  define REG_X23      REG_X23_NDX
#  define REG_X24      REG_X24_NDX
#  define REG_X25      REG_X25_NDX
#  define REG_X26      REG_X26_NDX
#  define REG_X27      REG_X27_NDX
#  define REG_X28      REG_X28_NDX
#  define REG_X29      REG_X29_NDX
#  define REG_X30      REG_X30_NDX
#  define REG_X31      REG_X31_NDX
#  define REG_INT_CTX  REG_X32_NDX
#endif

/* Register aliases */

#define REG_GP         REG_X26
#define REG_FP         REG_X27
#define REG_SP         REG_X28
#define REG_RA         REG_X29
#define REG_EA         REG_X30
#define REG_BA         REG_X31

#define REG_EPC        REG_X30

#define REG_A0         REG_X1
#define REG_A1         REG_X2
#define REG_A2         REG_X3
#define REG_A3         REG_X4
#define REG_A4         REG_X5
#define REG_A5         REG_X6
#define REG_A6         REG_X7
#define REG_A7         REG_X8

#define REG_IE         REG_INT_CTX

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct xcptcontext
{
#ifndef CONFIG_DISABLE_SIGNALS
  /* The following function pointer is non-NULL if there are pending signals
   * to be processed.
   */

  void *sigdeliver;       /* Actual type is sig_deliver_t */

  /* These additional register save locations are used to implement the
   * signal delivery trampoline.
   */

  uint32_t saved_epc;     /* Trampoline PC */
  uint32_t saved_int_ctx; /* Interrupt context with interrupts disabled. */

# ifdef CONFIG_BUILD_KERNEL
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uint32_t sigreturn;

# endif
#endif

#ifdef CONFIG_BUILD_KERNEL
  /* The following array holds information needed to return from each nested
   * system call.
   */

  uint8_t nsyscalls;
  struct xcpt_syscall_s syscall[CONFIG_SYS_NNEST];

#endif

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];
};

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MISOC_INCLUDE_LM32_IRQ_H */
