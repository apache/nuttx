/************************************************************************************
 * arch/arm/src/armv7-a/arm_savefpu.S
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#ifdef CONFIG_ARCH_FPU

	.file		"arm_savefpu.S"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.globl		up_savefpu

/************************************************************************************
 * Public Functions
 ************************************************************************************/

	.text

/************************************************************************************
 * Name: up_savefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), save the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void up_savefpu(uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area in which to save the floating point
 *     registers
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

	.globl	up_savefpu
	.type	up_savefpu, function

up_savefpu:

	add		r1, r0, #(4*REG_S0)		/* R1=Address of FP register storage */

	/* Store all floating point registers.  Registers are stored in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

	vstmia	r1!, {s0-s31}			/* Save the full FP context */

	/* Store the floating point control and status register.  At the end of the
	 * vstmia, r1 will point to the FPCSR storage location.
	 */

	vmrs	r2, fpscr				/* Fetch the FPCSR */
	str		r2, [r1], #4			/* Save the floating point control and status register */
	bx		lr

	.size	up_savefpu, .-up_savefpu
#endif /* CONFIG_ARCH_FPU */
	.end
