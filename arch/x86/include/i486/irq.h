/****************************************************************************
 * arch/x86/include/i486/irq.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_X86_INCLUDE_I486_IRQ_H
#define __ARCH_X86_INCLUDE_I486_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <arch/arch.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Storage order: %ebx, $esi, %edi, %ebp, sp, and return PC */

#ifdef __ASSEMBLY__
# define REG_EBX (0*4)
# define REG_ESI (1*4)
# define REG_EDI (2*4)
# define REG_EBP (3*4)
# define REG_SP  (4*4)
# define REG_PC  (5*4)
#else
# define REG_EBX (0)
# define REG_ESI (1)
# define REG_EDI (2)
# define REG_EBP (3)
# define REG_SP  (4)
# define REG_PC  (5)
#endif /* __ASSEMBLY__ */

#define XCPTCONTEXT_REGS    (6)
#define XCPTCONTEXT_SIZE    (6 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored */

#ifndef __ASSEMBLY__
struct xcptcontext
{
   void *sigdeliver; /* Actual type is sig_deliver_t */

   /* Storage order: %ebx, $esi, %edi, %ebp, sp, and return PC */

   uint32_t regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Get the current FLAGS register contents */

static inline irqstate_t irqflags()
{
  irqstate_t flags;

  asm volatile(
    "\tpushf\n"
    "\tpop %0\n"
    : "=rm" (flags)
    :
    : "memory");
  return flags;
}

/* Get a sample of the FLAGS register, determine if interrupts are disabled */

static inline bool irqdisabled(irqstate_t flags)
{
  return ((flags & X86_FLAGS_IF) == 0);
}

/* Disable interrupts unconditionally */

static inline void irqdisable(void)
{
  asm volatile("cli": : :"memory");
}

/* Enable interrupts unconditionally */

static inline void irqenable(void)
{
  asm volatile("sti": : :"memory");
}

/* Disable interrupts, but return previous interrupt state */

static inline irqstate_t irqsave(void)
{
  irqstate_t flags = irqflags();
  irqdisable();
  return flags;
}

/* Conditionally disable interrupts */

static inline void irqrestore(irqstate_t flags)
{
  if (irqdisabled(flags))
    {
      irqdisable();
    }
}

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_INCLUDE_I486_IRQ_H */

