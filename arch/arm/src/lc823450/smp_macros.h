/****************************************************************************
 * arch/arm/src/lc823450/smp_macros.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LC823450_SMP_MACROS_H
#define __ARCH_ARM_SRC_LC823450_SMP_MACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(__ASSEMBLY__) && defined(CONFIG_SMP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COREID_REG 0xe00fe000

/****************************************************************************
 * Imported Public Data
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	.globl	g_cpu0_instack_base
	.globl	g_cpu1_instack_base
#endif

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setintstack, tmp
#if CONFIG_SMP_NCPUS > 1
	ldr		\tmp, =COREID_REG
	ldr		\tmp, [\tmp, 0]     /* \tmp = getreg32(coreid_reg) */
	and		\tmp, \tmp, 1       /* \tmp = COREID */
	cmp		\tmp, #0
	bne		1f
	ldr		sp, =g_cpu0_instack_base
	b		2f
1:
	ldr		sp, =g_cpu1_instack_base
2:
#else
	ldr		sp, =g_cpu0_instack_base
#endif
	.endm
#endif

#endif /* __ASSEMBLY__ && CONFIG_SMP */
#endif /* __ARCH_ARM_SRC_LC823450_SMP_MACROS_H */
