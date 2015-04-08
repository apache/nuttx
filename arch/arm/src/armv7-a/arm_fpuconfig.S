/****************************************************************************
 * arch/arm/src/armv7-a/arm_fpuconfig.S
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "cp15.h"

	.file	"arm_fpuconfig.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	arm_fpuconfig

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_fpuconfig
 *
 * Description:
 *   Configure the FPU.  Enables access to CP10 and CP11
 *
 ****************************************************************************/

	.globl	arm_fpuconfig
	.type	arm_fpuconfig, %function

arm_fpuconfig:

	/* Enable access to CP10 and CP11 in CP15.CACR */

	mrc		CP15_CPACR(r0)
	orr		r0, r0, #0xf00000
	mcr		CP15_CPACR(r0)

	/* Enable access to CP10 and CP11 in CP15.NSACR */
	/* REVISIT: Do we need to do this? */

	/* Set FPEXC.EN (B30) */

	fmrx	r0, fpexc
	orr		r0, r0, #0x40000000
	fmxr	fpexc, r0
	bx		lr
	.size	arm_fpuconfig, . - arm_fpuconfig
	.end
