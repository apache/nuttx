/****************************************************************************
 * arch/arm/src/arm/arm_nommuhead.S
 *
 *   Copyright (C) 2007, 2009-2010, 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "arm_internal.h"
#include "arm_arch.h"

/****************************************************************************
 * Macros
 ****************************************************************************/

	/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG_FEATURES
	.macro	showprogress, code
	mov	r0, #\code
	bl	arm_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * OS Entry Point
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */
	.text
        .global __start
	.type	__start, #function
__start:

	/* First, setup initial processor mode */

	mov		r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT )
	msr		cpsr, r0

	showprogress 'A'

	/* Setup system stack (and get the BSS range) */

	adr		r0, .Lbssinit
	ldmia	r0, {r4, r5, sp}

	/* Clear system BSS section */

	mov		r0, #0
1:	cmp		r4, r5
	strcc	r0, [r4], #4
	bcc		1b

	showprogress 'B'

	/* Copy system .data sections to new home in RAM. */

#ifdef CONFIG_BOOT_RUNFROMFLASH

	adr		r3, .Ldatainit
	ldmia	r3, {r0, r1, r2}

1:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp		r1, r2
	blt		1b

#endif

	/* Perform early serial initialization */

	mov		fp, #0

#ifdef USE_EARLYSERIALINIT
	bl		arm_earlyserialinit
#endif

#ifdef CONFIG_DEBUG_FEATURES
	mov		r0, #'C'
	bl		up_putc
	mov		r0, #'\n'
	bl		up_putc
#endif

	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	bl		board_autoled_initialize
#endif

#ifdef CONFIG_STACK_COLORATION
	/* Write a known value to the IDLE thread stack to support stack
	 * monitoring logic
	 */

	adr		r3, .Lstkinit
	ldmia	r3, {r0, r1, r2}	/* R0 = start of IDLE stack; R1 = Size of tack; R2 = coloration */

2:								/* Top of the loop */
	sub		r1, r1, #1			/* R1 = Number of words remaining */
	cmp		r1, #0				/* Check (nwords == 0) */
	str		r2, [r0], #4		/* Save stack color word, increment stack address */
	bne		2b					/* Bottom of the loop */

#endif

	/* Finally branch to the OS entry point */

	mov		lr, #0				/* LR = return address (none) */
	b		nx_start			/* Branch to nx_start */

	/* Variables:
	 * _sbss is the start of the BSS region (see ld.script)
	 * _ebss is the end of the BSS region (see ld.script)
	 * The idle task stack starts at the end of BSS and is
	 * of size CONFIG_IDLETHREAD_STACKSIZE.  The heap continues
	 * from there until the end of memory.  See g_idle_topstack
	 * below.
	 */

.Lbssinit:
	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

#ifdef CONFIG_BOOT_RUNFROMFLASH
.Ldatainit:
	.long	_eronly	/* Where .data defaults are stored in FLASH */
	.long	_sdata	/* Where .data needs to reside in SDRAM */
	.long	_edata
#endif

#ifdef CONFIG_STACK_COLORATION
	.type	.Lstkinit, %object
.Lstkinit:
	.long	_ebss					/* Beginning of the IDLE stack, then words of IDLE stack */
	.long	(CONFIG_IDLETHREAD_STACKSIZE >> 2)
	.long	STACK_COLOR				/* Stack coloration word */
	.size	.Lstkinit, . -.Lstkinit
#endif
	.size	__start, .-__start

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to LCO
	 * above.
	 */

	.data
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack

	.end
