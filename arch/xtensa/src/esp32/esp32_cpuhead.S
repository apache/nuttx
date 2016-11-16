/****************************************************************************
 * arch/xtensa/src/esp32/esp32_cpuhead.S
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

	.file	"esp32_cpuhead.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/xtensa/xtensa_corebits.h>
#include <arch/esp32/core-isa.h>

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

	.section .noinit, "aw"
	.global	g_cpu1_idlestack
	.type	g_cpu1_idlestack, @object
	.align	16

g_cpu1_idlestack:
	.space	(CONFIG_SMP_IDLETHREAD_STACKSIZE & ~15)
.Lcpu1_stacktop:
	.size	g_cpu1_idlestack, . - g_cpu1_idlestack

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: __cpu[n]_start
 *
 * Description:
 *   Boot functions for each CPU (other than CPU0).  These functions set up
 *   the ARM operating mode, the initial stack, and configure co-processor
 *   registers.  At the end of the boot, esp32_cpu_boot() is called.
 *
 *   These functions are provided by the common ARMv7-A logic.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   Do not return.
 *
 ****************************************************************************/

	.global	__cpu1_start
	.type	__cpu1_start, @function

	.align	4

.Lcpu1_bottomofstack:
	.long	.Lcpu1_stacktop
	.size	.Lcpu1_bottomofstack, . - .Lcpu1_bottomofstack

#ifdef CONFIG_STACK_COLORATION
.Lcpu1_bottomofstack:
	.long	.Lcpu1_stacktop
	.size	.Lcpu1_bottomofstack, . - .Lcpu1_bottomofstack

.Lcpu1_stackcolor:
	.long	STACK_COLOR
	.size	.Lcpu1_stackcolor, . - .Lcpu1_stackcolor
#endif

	.align	4

__cpu1_start:

	/* Set up the stack pointer and the CPU index */

	l32r	sp, .Lcpu1_bottomofstack

	/* REVIST: Does it make since to have co-processors enabled on the IDLE thread? */

#ifdef CONFIG_STACK_COLORATION
	/* Write a known value to the IDLE thread stack to support stack
	 * monitoring logic
	 */

	mov		a0, sp
	l32r	a1, .Lcpu1_bottomofstack
	l32r	a2, .Lcpu1_stackcolor

1:
	s32i	a2, a1, 0
	addi	a2, a2, 4
	bne		a1, a2, 1b
#endif

	/* Set up the intiali PS */

#ifdef __XTENSA_CALL0_ABI__
	movi	a0, (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_EXCM)
#else
	movi	a0, (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1))
#endif
	wsr		a0, PS

	/* Finish initialization in C */

	movi	a2, 1				/* Argument 1: CPU ID */
	call0	xtensa_start_handler

	/* xtensa_start_handler() does not return */

2:	j		2b
	.size	__cpu1_start, . - __cpu1_start
#endif /* CONFIG_SMP */
