/****************************************************************************
 * arch/xtensa/src/common/xtensa_irq.S
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "xtensa_specregs.h"
#include "xtensa_macros.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_XTENSA_HAVE_INTERRUPTS

/* INTENABLE virtualization information. */

	.data
	.global	_xtensa_intdata
	.align	8
_xtensa_intdata:

	.global	_xtensa_intenable
	.type	_xtensa_intenable, @object
_xtensa_intenable:
	.word	0						/* Virtual INTENABLE     */
	.size	_xtensa_intenable,4

	.global	_xtensa_vprimask
	.type	_xtensa_vprimask, @object
_xtensa_vprimask:
	.word	0xffffffff				/* Virtual priority mask */
	.size	_xtensa_vprimask, 4

#endif /* CONFIG_XTENSA_HAVE_INTERRUPTS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * C Prototype:
 *   irqstate_t xtensa_enable_interrupts(irqstate_t mask)
 *
 * Description:
 *   Enables a set of interrupts. Does not simply set INTENABLE directly,
 *   but computes it as a function of the current virtual priority.
 *   Can be called from interrupt handlers.
 *
 ****************************************************************************/

	.text
	.align	4
	.global	xtensa_enable_interrupts
	.type	xtensa_enable_interrupts, @function

xtensa_enable_interrupts:

	ENTRY0
#ifdef CONFIG_XTENSA_HAVE_INTERRUPTS
	movi	a3, 0
	movi	a4, _xtensa_intdata
	xsr		a3, INTENABLE			/* Disables all interrupts */
	rsync
	l32i	a3, a4, 0				/* a3 = _xtensa_intenable */
	l32i	a6, a4, 4				/* a6 = _xtensa_vprimask */
	or		a5, a3, a2				/* a5 = _xtensa_intenable | mask */
	s32i	a5, a4, 0				/* _xtensa_intenable |= mask */
	and		a5, a5, a6				/* a5 = _xtensa_intenable & _xtensa_vprimask */
	wsr		a5, INTENABLE			/* Reenable interrupts */
	mov		a2, a3					/* Previous mask */
#else
	movi	a2, 0					/* Return zero */
#endif
	RET0

	.size	xtensa_enable_interrupts, . - xtensa_enable_interrupts

/****************************************************************************
 * C Prototype:
 *   irqstate_t xtensa_disable_interrupts(irqstate_t mask)
 *
 * Description:
 *   Disables a set of interrupts. Does not simply set INTENABLE directly,
 *   but computes it as a function of the current virtual priority.
 *   Can be called from interrupt handlers.
 *
 ****************************************************************************/

	.text
	.align	4
	.global	xtensa_disable_interrupts
	.type	xtensa_disable_interrupts,@function

xtensa_disable_interrupts:

	ENTRY0
#ifdef CONFIG_XTENSA_HAVE_INTERRUPTS
	movi	a3, 0
	movi	a4, _xtensa_intdata
	xsr		a3, INTENABLE			/* Disables all interrupts */
	rsync
	l32i	a3, a4, 0				/* a3 = _xtensa_intenable */
	l32i	a6, a4, 4				/* a6 = _xtensa_vprimask */
	or		a5, a3, a2				/* a5 = _xtensa_intenable | mask */
	xor		a5, a5, a2				/* a5 = _xtensa_intenable & ~mask */
	s32i	a5, a4, 0				/* _xtensa_intenable &= ~mask */
	and		a5, a5, a6				/* a5 = _xtensa_intenable & _xtensa_vprimask */
	wsr		a5, INTENABLE			/* Reenable interrupts */
	mov		a2, a3					/* Previous mask */
#else
	movi	a2, 0					/* return zero */
#endif
	RET0

	.size	xtensa_disable_interrupts, . - xtensa_disable_interrupts
