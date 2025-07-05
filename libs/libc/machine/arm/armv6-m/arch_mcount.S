/****************************************************************************
 * libs/libc/machine/arm/armv6-m/arch_mcount.S
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
 * Public Functions
 ****************************************************************************/

	.globl	__gnu_mcount_nc

	.syntax	unified
	.file	"mcount.S"

	.type	__gnu_mcount_nc, %function
__gnu_mcount_nc:
	/* ARMv6-M (Cortex-M0) compatible implementation */
	push	{r0, r1, r2, r3, lr}	/* Save registers */
	mov	r1, lr			/* R1 contains callee address */
	movs	r0, #1			/* Prepare mask for clearing thumb bit */
	mvns	r0, r0			/* Invert mask: 0xFFFFFFFE */
	ands	r1, r0			/* Clear thumb bit in callee address */
	ldr	r0, [sp, #20]		/* R0 contains caller address */
	movs	r2, #1			/* Prepare mask for caller address */
	mvns	r2, r2			/* Invert mask: 0xFFFFFFFE */
	ands	r0, r2			/* Clear thumb bit in caller address */
	bl	mcount_internal		/* Jump to internal _mcount() implementation */
	pop	{r0, r1, r2, r3, r4}	/* Restore registers, r4 gets lr value */
	mov	lr, r4			/* Restore lr */
	bx	lr			/* Return to callee */

	.size	__gnu_mcount_nc, .-__gnu_mcount_nc
	.end
