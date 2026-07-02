/****************************************************************************
 * libs/libc/machine/arm/armv7-a/arch_strchrnul.S
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

#ifdef LIBC_BUILD_STRCHRNUL

/* Assumptions:
 *
 * ARMv7-A / ARMv7-R (ARM state), unaligned word accesses supported.
 *
 * Like arch_strchr.S but returns a pointer to the NUL terminator when the
 * byte is not found (never returns NULL).
 */

#define src		r0
#define ch		r1
#define result		r0
#define data		r2
#define repc		r3
#define syndrome	r12
#define tmp		r4
#define const_m1	r5
#define const_h		r6

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(strchrnul)
	.arm
	.type ARCH_LIBCFUN(strchrnul), %function

ARCH_LIBCFUN(strchrnul):
	push	{r4, r5, r6, lr}
	and	ch, ch, #0xff

	movw	const_m1, #:lower16:0x01010101
	movt	const_m1, #:upper16:0x01010101
	movw	const_h, #:lower16:0x80808080
	movt	const_h, #:upper16:0x80808080
	mul	repc, ch, const_m1

	ands	tmp, src, #3
	beq	.Lword_loop
.Lhead:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lret
	cmp	data, #0
	beq	.Lret
	ands	tmp, src, #3
	bne	.Lhead

	.p2align 2
.Lword_loop:
	ldr	data, [src], #4
	sub	syndrome, data, const_m1
	bic	syndrome, syndrome, data
	tst	syndrome, const_h
	bne	.Lterm
	eor	tmp, data, repc
	sub	syndrome, tmp, const_m1
	bic	syndrome, syndrome, tmp
	tst	syndrome, const_h
	bne	.Lfound
	b	.Lword_loop

.Lfound:
	sub	src, src, #4
.Lfound_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lret
	cmp	data, #0
	bne	.Lfound_scan
	b	.Lret

.Lterm:
	sub	src, src, #4
.Lterm_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lret
	cmp	data, #0
	bne	.Lterm_scan

.Lret:
	sub	result, src, #1
	pop	{r4, r5, r6, pc}

	.size ARCH_LIBCFUN(strchrnul), . - ARCH_LIBCFUN(strchrnul)

#endif
