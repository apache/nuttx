/****************************************************************************
 * arch/x86_64/include/setjmp.h
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

#ifndef __ARCH_X86_64_INCLUDE_SETJUMP_H
#define __ARCH_X86_64_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

  /* Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip */

#  define XCPTCONTEXT_REGS    9
#  define XCPTCONTEXT_SIZE    (8 * XCPTCONTEXT_REGS)

#  ifdef __ASSEMBLY__

#    define JB_RBX            (0*8)
#    define JB_RSP            (1*8)
#    define JB_RBP            (2*8)
#    define JB_R12            (3*8)
#    define JB_R13            (4*8)
#    define JB_R14            (5*8)
#    define JB_R15            (6*8)
#    define JB_RIP            (7*8)
#    define JB_FLAG           (8*8)

#  else

#    define JB_RBX            (0)
#    define JB_RSP            (1)
#    define JB_RBP            (2)
#    define JB_R12            (3)
#    define JB_R13            (4)
#    define JB_R14            (5)
#    define JB_R15            (6)
#    define JB_RIP            (7)
#    define JB_FLAG           (8)

#  endif /* __ASSEMBLY__ */

/* Compatibility definitions */

#  define JB_FP               JB_RBP
#  define JB_SP               JB_RSP
#  define JB_PC               JB_RIP

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
#endif /* __ARCH_X86_64_INCLUDE_SETJUMP_H */
