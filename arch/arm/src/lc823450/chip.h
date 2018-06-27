/****************************************************************************
 * arch/arm/src/lc823450/chip.h
 *
 *   Copyright 2014, 2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#ifndef _ARCH_ARM_SRC_LC823450_CHIP_H
#define _ARCH_ARM_SRC_LC823450_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <arch/lc823450/chip.h>
#  include <arch/lc823450/irq.h>
#  include "up_arch.h"
#  include "lc823450_irq.h"
#  include "up_arch.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide the required number of peripheral interrupt vector definitions.
 * The definition LC823450_IRQ_NEXTINT simply comes from the chip-specific
 * IRQ header file included by arch/lc823450/irq.h.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  LC823450_IRQ_NEXTINT

/* Access to COREID register */

#define LC823450_CORE_BASE  0xe00fe000
#define CORE_COREID         (LC823450_CORE_BASE + 0)
#define CORE_COREID_ID      (1 << 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __ASSEMBLY__

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.globl	g_instack_alloc
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 7 */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the correct interrupt stack
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setintstack, tmp1, tmp2
#if CONFIG_SMP_NCPUS > 1
	ldr		\tmp1, =CORE_COREID
	ldr		\tmp1, [\tmp1, 0]     /* \tmp = getreg32(coreid_reg) */
	and		\tmp1, \tmp1, 1       /* \tmp = COREID */
	cmp		\tmp1, #0
	bne		1f
	ldr		\tmp1, =g_cpu0_instack_base
	ldr		sp, [\tmp1, 0]        /* sp = getreg32(g_cpu0_instack_base) */
	b		2f
1:
	ldr		\tmp1, =g_cpu1_instack_base
	ldr		sp, [\tmp1, 0]        /* sp = getreg32(g_cpu1_instack_base) */
2:
#else
	ldr		\tmp1, =g_cpu0_instack_base
	ldr		sp, [\tmp1, 0]        /* sp = getreg32(g_cpu0_instack_base) */
#endif
	.endm
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 7 */

#endif /* __ASSEMBLY__  */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_intstack_base
 *
 * Description:
 *   Set the current stack pointer to the "base" the correct interrupt stack
 *   allocation for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
static inline uintptr_t up_intstack_base(void)
{
  uintptr_t base = (uintptr_t)g_instack_alloc;
#if CONFIG_SMP_NCPUS > 1
  uint32_t coreid = getreg32(CORE_COREID);

  if ((coreid & CORE_COREID_ID) != 0)
    {
      base += INTSTACK_SIZE;
    }
#endif

  return base;
}
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* !__ASSEMBLY__ */

#endif  /* _ARCH_ARM_SRC_LC823450_CHIP_H */
