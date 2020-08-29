/****************************************************************************
 * arch/sim/include/irq.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#elif defined(CONFIG_HOST_X86) || defined(CONFIG_SIM_M32)
  /* Storage order: %ebx, %esi, %edi, %ebp, sp, and return PC */

#  define XCPTCONTEXT_REGS    6
#elif defined(CONFIG_HOST_ARM)
#  define XCPTCONTEXT_REGS    16
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
typedef unsigned long xcpt_reg_t;
#else
typedef int xcpt_reg_t;
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
