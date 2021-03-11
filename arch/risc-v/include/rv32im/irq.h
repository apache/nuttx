/****************************************************************************
 * arch/risc-v/include/rv32im/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_RV32IM_IRQ_H
#define __ARCH_RISCV_INCLUDE_RV32IM_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* How many nested system calls should we support? */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* Processor PC */

#define REG_EPC_NDX         0

/* General pupose registers */

/* $0: Zero register does not need to be saved
 * $1: ra (return address)
 */

#define REG_X1_NDX          1

/* $2: Stack POinter
 * $3: Global Pointer
 * $4: Thread Pointer
 */

#define REG_X2_NDX          2
#define REG_X3_NDX          3
#define REG_X4_NDX          4

/* $5-$7 = t0-t3: Temporary registers */

#define REG_X5_NDX          5
#define REG_X6_NDX          6
#define REG_X7_NDX          7

/* $8: s0 / fp Frame pointer */

#define REG_X8_NDX          8

/* $89 s1 Saved register */

#define REG_X9_NDX          9

/* $10-$17 = a0-a7: Argument registers */

#define REG_X10_NDX         10
#define REG_X11_NDX         11
#define REG_X12_NDX         12
#define REG_X13_NDX         13
#define REG_X14_NDX         14
#define REG_X15_NDX         15
#define REG_X16_NDX         16
#define REG_X17_NDX         17

/* $18-$27 = s2-s11: Saved registers */

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

/* $28-31 = t3-t6: Temporary (Volatile) registers */

#define REG_X28_NDX         28
#define REG_X29_NDX         29
#define REG_X30_NDX         30
#define REG_X31_NDX         31

/* Interrupt Context register */

#define REG_INT_CTX_NDX     32

#define INT_XCPT_REGS       33
#define INT_XCPT_SIZE       (4 * INT_XCPT_REGS)

#ifdef CONFIG_ARCH_FPU

#if defined(CONFIG_ARCH_DPFPU)
#  define FPU_REG_SIZE      2
#elif defined(CONFIG_ARCH_QPFPU)
#  define FPU_REG_SIZE      4
#else
#  define FPU_REG_SIZE      1
#endif

#  define REG_F0_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 0)
#  define REG_F1_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 1)
#  define REG_F2_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 2)
#  define REG_F3_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 3)
#  define REG_F4_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 4)
#  define REG_F5_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 5)
#  define REG_F6_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 6)
#  define REG_F7_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 7)
#  define REG_F8_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 8)
#  define REG_F9_NDX        (INT_XCPT_REGS + FPU_REG_SIZE * 9)
#  define REG_F10_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 10)
#  define REG_F11_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 11)
#  define REG_F12_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 12)
#  define REG_F13_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 13)
#  define REG_F14_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 14)
#  define REG_F15_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 15)
#  define REG_F16_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 16)
#  define REG_F17_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 17)
#  define REG_F18_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 18)
#  define REG_F19_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 19)
#  define REG_F20_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 20)
#  define REG_F21_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 21)
#  define REG_F22_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 22)
#  define REG_F23_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 23)
#  define REG_F24_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 24)
#  define REG_F25_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 25)
#  define REG_F26_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 26)
#  define REG_F27_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 27)
#  define REG_F28_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 28)
#  define REG_F29_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 29)
#  define REG_F30_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 30)
#  define REG_F31_NDX       (INT_XCPT_REGS + FPU_REG_SIZE * 31)
#  define REG_FCSR_NDX      (INT_XCPT_REGS + FPU_REG_SIZE * 32)

#  define FPU_XCPT_REGS     (FPU_REG_SIZE * 33)
#else
#  define FPU_XCPT_REGS     0
#endif

#define XCPTCONTEXT_REGS    (INT_XCPT_REGS + FPU_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/* In assembly language, values have to be referenced as byte address
 * offsets.  But in C, it is more convenient to reference registers as
 * register save table offsets.
 */

#ifdef __ASSEMBLY__
#  define REG_EPC           (4*REG_EPC_NDX)
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
#  define REG_INT_CTX       (4*REG_INT_CTX_NDX)
#ifdef CONFIG_ARCH_FPU
#  define REG_F0            (4*REG_F0_NDX)
#  define REG_F1            (4*REG_F1_NDX)
#  define REG_F2            (4*REG_F2_NDX)
#  define REG_F3            (4*REG_F3_NDX)
#  define REG_F4            (4*REG_F4_NDX)
#  define REG_F5            (4*REG_F5_NDX)
#  define REG_F6            (4*REG_F6_NDX)
#  define REG_F7            (4*REG_F7_NDX)
#  define REG_F8            (4*REG_F8_NDX)
#  define REG_F9            (4*REG_F9_NDX)
#  define REG_F10           (4*REG_F10_NDX)
#  define REG_F11           (4*REG_F11_NDX)
#  define REG_F12           (4*REG_F12_NDX)
#  define REG_F13           (4*REG_F13_NDX)
#  define REG_F14           (4*REG_F14_NDX)
#  define REG_F15           (4*REG_F15_NDX)
#  define REG_F16           (4*REG_F16_NDX)
#  define REG_F17           (4*REG_F17_NDX)
#  define REG_F18           (4*REG_F18_NDX)
#  define REG_F19           (4*REG_F19_NDX)
#  define REG_F20           (4*REG_F20_NDX)
#  define REG_F21           (4*REG_F21_NDX)
#  define REG_F22           (4*REG_F22_NDX)
#  define REG_F23           (4*REG_F23_NDX)
#  define REG_F24           (4*REG_F24_NDX)
#  define REG_F25           (4*REG_F25_NDX)
#  define REG_F26           (4*REG_F26_NDX)
#  define REG_F27           (4*REG_F27_NDX)
#  define REG_F28           (4*REG_F28_NDX)
#  define REG_F29           (4*REG_F29_NDX)
#  define REG_F30           (4*REG_F30_NDX)
#  define REG_F31           (4*REG_F31_NDX)
#  define REG_FCSR          (4*REG_FCSR_NDX)
#endif

#else
#  define REG_EPC           REG_EPC_NDX
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
#  define REG_INT_CTX       REG_INT_CTX_NDX

#ifdef CONFIG_ARCH_FPU
#  define REG_F0            REG_F0_NDX
#  define REG_F1            REG_F1_NDX
#  define REG_F2            REG_F2_NDX
#  define REG_F3            REG_F3_NDX
#  define REG_F4            REG_F4_NDX
#  define REG_F5            REG_F5_NDX
#  define REG_F6            REG_F6_NDX
#  define REG_F7            REG_F7_NDX
#  define REG_F8            REG_F8_NDX
#  define REG_F9            REG_F9_NDX
#  define REG_F10           REG_F10_NDX
#  define REG_F11           REG_F11_NDX
#  define REG_F12           REG_F12_NDX
#  define REG_F13           REG_F13_NDX
#  define REG_F14           REG_F14_NDX
#  define REG_F15           REG_F15_NDX
#  define REG_F16           REG_F16_NDX
#  define REG_F17           REG_F17_NDX
#  define REG_F18           REG_F18_NDX
#  define REG_F19           REG_F19_NDX
#  define REG_F20           REG_F20_NDX
#  define REG_F21           REG_F21_NDX
#  define REG_F22           REG_F22_NDX
#  define REG_F23           REG_F23_NDX
#  define REG_F24           REG_F24_NDX
#  define REG_F25           REG_F25_NDX
#  define REG_F26           REG_F26_NDX
#  define REG_F27           REG_F27_NDX
#  define REG_F28           REG_F28_NDX
#  define REG_F29           REG_F29_NDX
#  define REG_F30           REG_F30_NDX
#  define REG_F31           REG_F31_NDX
#  define REG_FCSR          REG_FCSR_NDX
#endif

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
  uint32_t sysreturn;   /* The return PC */
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

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These additional register save locations are used to implement the
   * signal delivery trampoline.
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  uint32_t saved_epc;     /* Trampoline PC */
  uint32_t saved_int_ctx; /* Interrupt context with interrupts disabled. */

#ifdef CONFIG_BUILD_KERNEL
  /* This is the saved address to use when returning from a user-space
   * signal handler.
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

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_RISCV_INCLUDE_RV32IM_IRQ_H */
