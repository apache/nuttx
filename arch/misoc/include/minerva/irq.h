/****************************************************************************
 * arch/risc-v/include/rv32im/irq.h
 *
 *   Copyright (C) 2011, 2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *  Modified for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef __ARCH_MINERVA_INCLUDE_IRQ_H
#define __ARCH_MINERVA_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If this is a kernel build, how many nested system calls should we support? */

#define MINERVA_NINTERRUPTS 32
#define MINERVA_IRQ_SWINT   32
#define NR_IRQS             33

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

#define REG_X0_NDX          0
#define REG_X1_NDX          1
#define REG_X2_NDX          2
#define REG_X3_NDX          3
#define REG_X4_NDX          4
#define REG_X5_NDX          5
#define REG_X6_NDX          6
#define REG_X7_NDX          7
#define REG_X8_NDX          8
#define REG_X9_NDX          9
#define REG_X10_NDX         10
#define REG_X11_NDX         11
#define REG_X12_NDX         12
#define REG_X13_NDX         13
#define REG_X14_NDX         14
#define REG_X15_NDX         15
#define REG_X16_NDX         16
#define REG_X17_NDX         17
#define REG_X18_NDX         18
#define REG_X19_NDX         19
#define REG_X20_NDX         20
#define REG_X21_NDX         21
#define REG_X22_NDX         22
#define REG_X23_NDX         23
#define REG_X24_NDX         24
#define REG_X25_NDX         25
#define REG_X26_NDX         26
#define REG_X27_NDX         27
#define REG_X28_NDX         28
#define REG_X29_NDX         29
#define REG_X30_NDX         30
#define REG_X31_NDX         31

#define REG_CSR_MSTATUS_NDX     32
#define REG_CSR_MEPC_NDX        33
#define REG_CSR_MBADADDR_NDX   34
#define REG_CSR_MCAUSE_NDX      35

#define XCPTCONTEXT_REGS    36
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/* In assembly language, values have to be referenced as byte address
 * offsets.  But in C, it is more convenient to reference registers as
 * register save table offsets.
 */

#ifdef __ASSEMBLY__
#  define REG_X0            (4*REG_X0_NDX)
#  define REG_X1            (4*REG_X1_NDX)
#  define REG_X2            (4*REG_X2_NDX)
#  define REG_X3            (4*REG_X3_NDX)
#  define REG_X4            (4*REG_X4_NDX)
#  define REG_X5            (4*REG_X5_NDX)
#  define REG_X6            (4*REG_X6_NDX)
#  define REG_X7            (4*REG_X7_NDX)
#  define REG_X8            (4*REG_X8_NDX)
#  define REG_X9            (4*REG_X9_NDX)
#  define REG_X10           (4*REG_X10_NDX)
#  define REG_X11           (4*REG_X11_NDX)
#  define REG_X12           (4*REG_X12_NDX)
#  define REG_X13           (4*REG_X13_NDX)
#  define REG_X14           (4*REG_X14_NDX)
#  define REG_X15           (4*REG_X15_NDX)
#  define REG_X16           (4*REG_X16_NDX)
#  define REG_X17           (4*REG_X17_NDX)
#  define REG_X18           (4*REG_X18_NDX)
#  define REG_X19           (4*REG_X19_NDX)
#  define REG_X20           (4*REG_X20_NDX)
#  define REG_X21           (4*REG_X21_NDX)
#  define REG_X22           (4*REG_X22_NDX)
#  define REG_X23           (4*REG_X23_NDX)
#  define REG_X24           (4*REG_X24_NDX)
#  define REG_X25           (4*REG_X25_NDX)
#  define REG_X26           (4*REG_X26_NDX)
#  define REG_X27           (4*REG_X27_NDX)
#  define REG_X28           (4*REG_X28_NDX)
#  define REG_X29           (4*REG_X29_NDX)
#  define REG_X30           (4*REG_X30_NDX)
#  define REG_X31           (4*REG_X31_NDX)
#  define REG_CSR_MSTATUS   (4*REG_CSR_MSTATUS_NDX)
#  define REG_CSR_MEPC      (4*REG_CSR_MEPC_NDX)
#  define REG_CSR_MBADADDR  (4*REG_CSR_MBADADDR_NDX)
#  define REG_CSR_MCAUSE    (4*REG_CSR_MCAUSE_NDX)
#else
#  define REG_X0            REG_X0_NDX
#  define REG_X1            REG_X1_NDX
#  define REG_X2            REG_X2_NDX
#  define REG_X3            REG_X3_NDX
#  define REG_X4            REG_X4_NDX
#  define REG_X5            REG_X5_NDX
#  define REG_X6            REG_X6_NDX
#  define REG_X7            REG_X7_NDX
#  define REG_X8            REG_X8_NDX
#  define REG_X9            REG_X9_NDX
#  define REG_X10           REG_X10_NDX
#  define REG_X11           REG_X11_NDX
#  define REG_X12           REG_X12_NDX
#  define REG_X13           REG_X13_NDX
#  define REG_X14           REG_X14_NDX
#  define REG_X15           REG_X15_NDX
#  define REG_X16           REG_X16_NDX
#  define REG_X17           REG_X17_NDX
#  define REG_X18           REG_X18_NDX
#  define REG_X19           REG_X19_NDX
#  define REG_X20           REG_X20_NDX
#  define REG_X21           REG_X21_NDX
#  define REG_X22           REG_X22_NDX
#  define REG_X23           REG_X23_NDX
#  define REG_X24           REG_X24_NDX
#  define REG_X25           REG_X25_NDX
#  define REG_X26           REG_X26_NDX
#  define REG_X27           REG_X27_NDX
#  define REG_X28           REG_X28_NDX
#  define REG_X29           REG_X29_NDX
#  define REG_X30           REG_X30_NDX
#  define REG_X31           REG_X31_NDX
#  define REG_CSR_MSTATUS   REG_CSR_MSTATUS_NDX
#  define REG_CSR_MEPC      REG_CSR_MEPC_NDX
#  define REG_CSR_MBADADDR  REG_CSR_MBADADDR_NDX
#  define REG_CSR_MCAUSE    REG_CSR_MCAUSE_NDX
#endif

/* Now define more user friendly alternative name that can be used either
 * in assembly or C contexts.
 */

/* $1 = ra: Return address */

#define REG_RA              REG_X1

/* $2 = sp:  The value of the stack pointer on return from the exception */

#define REG_SP              REG_X2

/* $3 = gp: Only needs to be saved under conditions where there are
 * multiple, per-thread values for the GP.
 */

#define REG_GP              REG_X3

/* $4 = tp:  Thread Pointer */

#define REG_TP              REG_X4

/* $5-$7 = t0-t2: Caller saved temporary registers */

#define REG_T0              REG_X5
#define REG_T1              REG_X6
#define REG_T2              REG_X7

/* $8 = either s0 or fp:  Depends if a frame pointer is used or not */

#define REG_S0              REG_X8
#define REG_FP              REG_X8

/* $9 = s1: Caller saved register */

#define REG_S1              REG_X9

/* $10-$17 = a0-a7: Argument registers */

#define REG_A0              REG_X10
#define REG_A1              REG_X11
#define REG_A2              REG_X12
#define REG_A3              REG_X13
#define REG_A4              REG_X14
#define REG_A5              REG_X15
#define REG_A6              REG_X16
#define REG_A7              REG_X17

/* $18-$27 = s2-s11: Callee saved registers */

#define REG_S2              REG_X18
#define REG_S3              REG_X19
#define REG_S4              REG_X20
#define REG_S5              REG_X21
#define REG_S6              REG_X22
#define REG_S7              REG_X23
#define REG_S8              REG_X24
#define REG_S9              REG_X25
#define REG_S10             REG_X26
#define REG_S11             REG_X27

/* $28-$31 = t3-t6: Caller saved temporary registers */

#define REG_T3              REG_X28
#define REG_T4              REG_X29
#define REG_T5              REG_X30
#define REG_T6              REG_X31

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This structure represents the return state from a system call */

#ifdef CONFIG_BUILD_KERNEL
struct xcpt_syscall_s
{
  uint32_t sysreturn;         /* The return PC */
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext
{
  /* The following function pointer is non-NULL if there are pending signals
   * to be processed.
   */

  void *sigdeliver;           /* Actual type is sig_deliver_t */

  /* These additional register save locations are used to implement the
   * signal delivery trampoline.
   */

  uint32_t saved_epc;         /* Trampoline PC */
  uint32_t saved_int_ctx;     /* Interrupt context with interrupts disabled. */

#ifdef CONFIG_BUILD_KERNEL
  /* This is the saved address to use when returning from a user-space signal
   * handler.
   */

  uint32_t sigreturn;
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

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_RV32IM_IRQ_H */
