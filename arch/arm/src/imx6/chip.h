/****************************************************************************
 * arch/arm/src/imx6/chip.h
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_H
#define __ARCH_ARM_SRC_IMX6_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#endif

#include "chip/imx_memorymap.h"
#include "imx_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPCore registers are memory mapped and accessed through a processor
 * specific private address space via the SCU.  The Cortex-A9 MCU chip.h
 * header file must provide the definition CHIP_MPCORE_VBASE to access this
 * the registers in this memory region.
 */

#define CHIP_MPCORE_VBASE IMX_ARMMP_VSECTION

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __ASSEMBLY__

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.globl	g_irqstack_top
	.globl	g_fiqstack_top
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 7 */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

/***************************************************************************
 * Name: cpuindex
 *
 * Description:
 *   Return an index idenifying the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	cpuindex, index
	mrc		p15, 0, \index, c0, c0, 5	/* Read the MPIDR */
	and		\index, \index, #3			/* Bits 0-1=CPU ID */
	.endm
#endif

/***************************************************************************
 * Name: setirqstack
 *
 * Description:
 *   Set the current stack pointer to the  -"top" of the IRQ interrupt
 *   stack for the current CPU.
 *
 ***************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setirqstack, tmp1, tmp2
	mrc		p15, 0, \tmp1, c0, c0, 5	/* tmp1=MPIDR */
	and		\tmp1, \tmp1, #3			/* Bits 0-1=CPU ID */
	ldr		\tmp2, =g_irqstack_top		/* tmp2=Array of IRQ stack pointers */
	lsls	\tmp1, \tmp1, #2			/* tmp1=Array byte offset */
	add		\tmp2, \tmp2, \tmp1			/* tmp2=Offset address into array */
	ldr		sp, [\tmp2, #0]				/* sp=Address in stack allocation */
	.endm
#endif

/****************************************************************************
 * Name: setfiqstack
 *
 * Description:
 *   Set the current stack pointer to the  -"top" of the FIQ interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setfiqstack, tmp1, tmp2
	mrc		p15, 0, \tmp1, c0, c0, 5	/* tmp1=MPIDR */
	and		\tmp1, \tmp1, #3			/* Bits 0-1=CPU ID */
	ldr		\tmp2, =g_fiqstack_top		/* tmp2=Array of FIQ stack pointers */
	lsls	\tmp1, \tmp1, #2			/* tmp1=Array byte offset */
	add		\tmp2, \tmp2, \tmp1			/* tmp2=Offset address into array */
	ldr		sp, [\tmp2, #0]				/* sp=Address in stack allocation */
	.endm
#endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: up_intstack_base
 *
 * Description:
 *   Return a pointer to the "base" the correct interrupt stack allocation
 *   for the  current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
static inline uintptr_t up_intstack_base(void)
{
  uintptr_t base = (uintptr_t)g_irqstack_alloc;
#if CONFIG_SMP_NCPUS > 1
  uint32_t cpu = up_cpu_index();

  base += cpu * INTSTACK_SIZE;
#endif

  return base;
}
#endif

/****************************************************************************
 * Name: up_intstack_top
 *
 * Description:
 *   Return a pointer to the "top" the correct interrupt stack for the
 *   current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
static inline uintptr_t up_intstack_top(void)
{
  return up_intstack_base() + INTSTACK_SIZE;
}
#endif

#endif /* !__ASSEMBLY__ */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_H */
