/****************************************************************************
 * libs/libc/machine/arm/armv8-r/arch_strnlen.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "libc.h"

#ifdef LIBC_BUILD_STRNLEN

/* Assumptions:
 *
 * ARMv7-A / ARMv7-R (ARM state).
 */

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(strnlen)
	.arm
	.type ARCH_LIBCFUN(strnlen), %function

ARCH_LIBCFUN(strnlen):
	push	{r4, r5, r6, lr}
	movw	r4, #0x0101
	movt	r4, #0x0101		/* const_m1 = 0x01010101 */
	movw	r5, #0x8080
	movt	r5, #0x8080		/* const_h  = 0x80808080 */
	mov	r6, r0			/* ptr = src */
	cmp	r1, #0
	beq	.Lzero
	/* Byte-scan the misaligned head until ptr is word-aligned.  */
	ands	r3, r6, #3
	beq	.Lcheck4
.Lhead:
	ldrb	r2, [r6]
	cmp	r2, #0
	beq	.Ldone		/* NUL found, ptr at NUL */
	add	r6, #1
	subs	r1, #1
	beq	.Ldone			/* maxlen exhausted */
	ands	r3, r6, #3
	bne	.Lhead
.Lcheck4:
	cmp	r1, #4
	blo	.Lbyte_tail
.Lwl:
	ldr	r2, [r6]		/* aligned word */
	sub	r3, r2, r4
	bic	r3, r3, r2
	tst	r3, r5
	bne	.Lnulword		/* NUL in this word */
	add	r6, #4
	subs	r1, #4
	cmp	r1, #4
	bhs	.Lwl
.Lbyte_tail:
	cmp	r1, #0
	beq	.Ldone
.Lbloop:
	ldrb	r2, [r6]
	cmp	r2, #0
	beq	.Ldone
	add	r6, #1
	subs	r1, #1
	bne	.Lbloop
	b	.Ldone
.Lnulword:
	/* NUL in this word (ptr at word start): rescan bytes to locate it.  */
.Lnul_scan:
	ldrb	r2, [r6]
	cmp	r2, #0
	beq	.Ldone
	add	r6, #1
	b	.Lnul_scan
.Ldone:
	sub	r0, r6, r0		/* len = ptr - src */
	pop	{r4, r5, r6, pc}
.Lzero:
	mov	r0, #0
	pop	{r4, r5, r6, pc}

	.size ARCH_LIBCFUN(strnlen), . - ARCH_LIBCFUN(strnlen)

#endif
