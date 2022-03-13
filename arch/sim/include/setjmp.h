/****************************************************************************
 * arch/sim/include/setjmp.h
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

#ifndef __ARCH_SIM_INCLUDE_SETJUMP_H
#define __ARCH_SIM_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of registers saved in context switch */

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
  /* Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip */

#  define XCPTCONTEXT_REGS    8
#  define XCPTCONTEXT_SIZE    (8 * XCPTCONTEXT_REGS)

#  ifdef __ASSEMBLY__

#    define JB_RBX            (0*8)
#    define JB_RSP            (1*8)
#    define JB_RBP            (2*8)
#    define JB_R12            (3*8)
#    define JB_R13            (4*8)
#    define JB_R14            (5*8)
#    define JB_R15            (6*8)
#    define JB_RSI            (7*8)

#  else

#    define JB_RBX            (0)
#    define JB_RSP            (1)
#    define JB_RBP            (2)
#    define JB_R12            (3)
#    define JB_R13            (4)
#    define JB_R14            (5)
#    define JB_R15            (6)
#    define JB_RSI            (7)

#  endif /* __ASSEMBLY__ */

/* Compatibility definitions */

#  define JB_FP               JB_RBP
#  define JB_SP               JB_RSP
#  define JB_PC               JB_RSI

#elif defined(CONFIG_HOST_X86) || defined(CONFIG_SIM_M32)
  /* Storage order: %ebx, %esi, %edi, %ebp, sp, and return PC */

#  define XCPTCONTEXT_REGS    6
#  define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#  ifdef __ASSEMBLY__

#    define JB_EBX            (0*4)
#    define JB_ESI            (1*4)
#    define JB_EDI            (2*4)
#    define JB_EBP            (3*4)
#    define JB_SP             (4*4)
#    define JB_PC             (5*4)

#  else

#    define JB_EBX            (0)
#    define JB_ESI            (1)
#    define JB_EDI            (2)
#    define JB_EBP            (3)
#    define JB_SP             (4)
#    define JB_PC             (5)

#  endif /* __ASSEMBLY__ */

/* Compatibility definitions */

#  define JB_FP               JB_EBP

#elif defined(CONFIG_HOST_ARM)

#  define XCPTCONTEXT_REGS    16
#  define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#  define JB_FP               7
#  define JB_SP               8
#  define JB_PC               9

#elif defined(CONFIG_HOST_ARM64)

#  define XCPTCONTEXT_REGS    32
#  define XCPTCONTEXT_SIZE    (8 * XCPTCONTEXT_REGS)

#  ifdef __ASSEMBLY__

#    define JB_X19_X20        #0x00
#    define JB_X21_X22        #0x10
#    define JB_X23_X24        #0x20
#    define JB_X25_X26        #0x30
#    define JB_X27_X28        #0x40
#    define JB_X29_XLR        #0x50
#    define JB_XFP_XSP        #0x60

#    define JB_D08_D09        #0x70
#    define JB_D10_D11        #0x80
#    define JB_D12_D13        #0x90
#    define JB_D14_D15        #0xA0

#  else

#    define JB_PC             (11)
#    define JB_FP             (12)
#    define JB_SP             (13)

#  endif /* __ASSEMBLY__ */

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef unsigned long xcpt_reg_t;
typedef xcpt_reg_t jmp_buf[XCPTCONTEXT_REGS];

#endif

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

int setjmp(jmp_buf env);
void longjmp(jmp_buf env, int val) noreturn_function;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !__ASSEMBLY__ */
#endif /* __ARCH_SIM_INCLUDE_SETJUMP_H */
