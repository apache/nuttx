/****************************************************************************
 * arch/arm/src/arm/arm_nommuhead.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "arm_internal.h"

/****************************************************************************
 * Macros
 ****************************************************************************/

	/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG_FEATURES
	.macro	showprogress, code
	mov		r0, #\code
	bl		arm_lowputc
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
	.syntax	unified
	.arm
	.global __start
	.type	__start, #function
__start:

	/* First, setup initial processor mode */

	mov		r0, #(PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT )
	msr		cpsr, r0

	/* Setup system stack (and get the BSS range) */

	adr		r0, .Lbssinit
	ldmia		r0, {r4, r5, sp}

	showprogress	'A'

	/* Copy system .data sections to new home in RAM. */

#ifdef CONFIG_BOOT_RUNFROMFLASH

	adr		r3, .Ldatainit
	ldmia		r3, {r0, r1, r2}

1:	ldmia		r0!, {r3, r6 - r12}
	stmia		r1!, {r3, r6 - r12}
	cmp		r1, r2
	blt		1b

#endif

	showprogress	'B'

	/* Clear system BSS section */

	mov		r0, #0
1:	cmp		r4, r5
	strcc		r0, [r4], #4
	bcc		1b

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
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE

#ifdef CONFIG_BOOT_RUNFROMFLASH
.Ldatainit:
	.long	_eronly	/* Where .data defaults are stored in FLASH */
	.long	_sdata	/* Where .data needs to reside in SDRAM */
	.long	_edata
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
