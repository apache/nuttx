/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpuint.S
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Cadence Design Systems Inc.
 *
 *   Copyright (c) 2006-2015 Cadence Design Systems Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

	.file	"xtensa_cpuint.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/core-isa.h>

#include "xtensa_abi.h"

#if XCHAL_HAVE_INTERRUPTS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_enable_cpuint
 *
 * C Prototype:
 *   uint32_t xtensa_enable_cpuint(uint32_t *shadow, unsigned int intmask)
 *
 * Description:
 *   Enables a set of interrupts. Does not simply set INTENABLE directly,
 *   but operates on a shadow copy of the CPU INTENABLE register then
 *   writes that value to the hardware INTENABLE register.  Can be called
 *   from interrupt handlers.
 *
 *   NOTE: It is possible only to enable interrupts on the current CPU
 *   because there is an INTENABLE register implemented in each CPU.
 *
 ****************************************************************************/

	.text
	.global	xtensa_enable_cpuint
	.type	xtensa_enable_cpuint, @function
	.align	4

xtensa_enable_cpuint:
	ENTRY(16)

	movi	a4, 0
	xsr		a4, INTENABLE		/* Disables all interrupts */
	rsync

	l32i	a4, a2, 0			  /* a4 = value of INTENABLE shadow */
	or		a5, a4, a3			/* a5 = shadow | mask */
	s32i	a5, a2, 0			  /* shadow |= mask */

	wsr		a5, INTENABLE		/* Set CPU INTENABLE to shadow */
	mov		a3, a4			   	/* Return previous shadow content */
	RET(16)

	.size	xtensa_enable_cpuint, . - xtensa_enable_cpuint

/****************************************************************************
 * Name: xtensa_disable_cpuint
 *
 * C Prototype:
 *   uint32_t xtensa_disable_cpuint(uint32_t *shadow, unsigned int intmask)
 *
 * Description:
 *   Disables a set of interrupts. Does not simply set INTENABLE directly,
 *   but operates on a shadow copy of the CPU INTENABLE register then
 *   writes that value to the hardware INTENABLE register.  Can be called
 *   from interrupt handlers.
 *
 *   NOTE: It is possible only to disable interrupts on the current CPU
 *   because there is an INTENABLE register implemented in each CPU.
 *
 ****************************************************************************/

	.text
	.global	xtensa_disable_cpuint
	.type	xtensa_disable_cpuint, @function
	.align	4

xtensa_disable_cpuint:
	ENTRY(16)

	movi	a4, 0
	xsr		a4, INTENABLE		/* Disables all interrupts */
	rsync

	l32i	a4, a2, 0			  /* a4 = value of INTENABLE shadow */
	or		a5, a4, a3			/* a5 = shadow | mask */
	xor		a5, a5, a3			/* a5 = shadow & ~mask */
	s32i	a5, a2, 0			  /* shadow &= ~mask */

	wsr		a5, INTENABLE		/* Set CPU INTENABLE to shadow */
	mov		a3, a4				  /* Return previous shadow content */
	RET(16)

	.size	xtensa_disable_cpuint, . - xtensa_disable_cpuint

#endif /* XCHAL_HAVE_INTERRUPTS */
