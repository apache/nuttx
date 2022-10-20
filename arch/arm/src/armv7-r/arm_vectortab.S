/****************************************************************************
 * arch/arm/src/arm7-r/arm_vectortab.S
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

	.file	"arm_vectortab.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

#ifdef CONFIG_ARM_TOOLCHAIN_ARMCLANG
	.eabi_attribute Tag_ABI_align_preserved, 1
#endif

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
	ldr		pc, .Lresethandler		/* 0x00: Reset */
	ldr		pc, .Lundefinedhandler		/* 0x04: Undefined instruction */
	ldr		pc, .Lsvchandler		/* 0x08: Software interrupt */
	ldr		pc, .Lprefetchaborthandler	/* 0x0c: Prefetch abort */
	ldr		pc, .Ldataaborthandler		/* 0x10: Data abort */
	ldr		pc, .Laddrexcptnhandler		/* 0x14: Address exception (reserved) */
	ldr		pc, .Lirqhandler		/* 0x18: IRQ */
	ldr		pc, .Lfiqhandler		/* 0x1c: FIQ */

	.globl	__start
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
