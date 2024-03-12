/****************************************************************************
 * libs/libc/machine/arm/gnu/mcount.S
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
	push	{r0, r1, r2, r3, lr}			/* Save registers */
	bic		r1, lr, #1			/* R1 contains callee address, with thumb bit cleared */
	ldr		r0, [sp, #20]			/* R0 contains caller address */
	bic		r0, r0, #1			/* Clear thumb bit */
	bl		mcount_internal			/* Jump to internal _mcount() implementation */
	pop		{r0, r1, r2, r3, ip, lr}	/* Restore saved registers */
	bx		ip				/* Return to callee */

	.size	__gnu_mcount_nc, .-__gnu_mcount_nc
	.end
