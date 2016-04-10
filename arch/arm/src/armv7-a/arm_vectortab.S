/****************************************************************************
 * arch/arm/src/arm7-a/arm_vectortab.S
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

	.file	"arm_vectortab.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		_vector_start
	.globl		_vector_end

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Name: _vector_start
 *
 * Description:
 *   Vector initialization block
 ****************************************************************************/

	.section	.vectors, "ax"
	.globl		_vector_start

/* These will be relocated to VECTOR_BASE. */

_vector_start:
	ldr		pc, .Lresethandler			/* 0x00: Reset */
	ldr		pc, .Lundefinedhandler		/* 0x04: Undefined instruction */
	ldr		pc, .Lsvchandler			/* 0x08: Software interrupt */
	ldr		pc, .Lprefetchaborthandler	/* 0x0c: Prefetch abort */
	ldr		pc, .Ldataaborthandler		/* 0x10: Data abort */
	ldr		pc, .Laddrexcptnhandler		/* 0x14: Address exception (reserved) */
	ldr		pc, .Lirqhandler			/* 0x18: IRQ */
	ldr		pc, .Lfiqhandler			/* 0x1c: FIQ */

	.globl   __start
	.globl	arm_vectorundefinsn
	.globl	arm_vectorsvc
	.globl	arm_vectorprefetch
	.globl	arm_vectordata
	.globl	arm_vectoraddrexcptn
	.globl	arm_vectorirq
	.globl	arm_vectorfiq

.Lresethandler:
	.long	__start
.Lundefinedhandler:
	.long	arm_vectorundefinsn
.Lsvchandler:
	.long	arm_vectorsvc
.Lprefetchaborthandler:
	.long	arm_vectorprefetch
.Ldataaborthandler:
	.long	arm_vectordata
.Laddrexcptnhandler:
	.long	arm_vectoraddrexcptn
.Lirqhandler:
	.long	arm_vectorirq
.Lfiqhandler:
	.long	arm_vectorfiq

	.globl	_vector_end
_vector_end:
	.size	_vector_start, . - _vector_start
	.end
