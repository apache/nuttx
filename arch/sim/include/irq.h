/****************************************************************************
 * arch/sim/include/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_SIM_INCLUDE_IRQ_H
#define __ARCH_SIM_INCLUDE_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NR_IRQS 64

/* Number of registers saved in context switch */

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
  /* Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip */

#  define XCPTCONTEXT_REGS    8
#  define XCPTCONTEXT_SIZE    (8 * XCPTCONTEXT_REGS)
#elif defined(CONFIG_HOST_X86) || defined(CONFIG_SIM_M32)
  /* Storage order: %ebx, %esi, %edi, %ebp, sp, and return PC */

#  define XCPTCONTEXT_REGS    6
#  define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)
#elif defined(CONFIG_HOST_ARM)
#  define XCPTCONTEXT_REGS    16
#  define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
typedef unsigned long xcpt_reg_t;
#else
typedef unsigned int xcpt_reg_t;
#endif

/* This struct defines the way the registers are stored */

struct xcptcontext
{
  void *sigdeliver; /* Actual type is sig_deliver_t */

  xcpt_reg_t regs[XCPTCONTEXT_REGS];
};
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

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: These functions should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

irqstate_t up_irq_save(void);
void up_irq_restore(irqstate_t flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !__ASSEMBLY__ */
#endif /* __ARCH_SIM_INCLUDE_IRQ_H */
