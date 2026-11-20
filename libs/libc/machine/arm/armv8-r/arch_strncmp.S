/****************************************************************************
 * libs/libc/machine/arm/armv8-r/arch_strncmp.S
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

#ifdef LIBC_BUILD_STRNCMP

/* Assumptions:
 *
 * ARMv7-A / ARMv7-R (ARM state), unaligned word accesses supported.
 *
 * Word-at-a-time comparison: 4 bytes per loop iteration using the
 * 0x01010101/0x80808080 NUL-detection trick, with a byte loop for the
 * misaligned head and the sub-word tail.
 */

#define src1		r0
#define src2		r1
#define limit		r2
#define result		r0
#define data1		r3
#define data2		r4
#define syndrome	r5
#define tmp1		r12
#define const_m1	r6		/* 0x01010101 */
#define const_h		r7		/* 0x80808080 */

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(strncmp)
	.arm
	.type ARCH_LIBCFUN(strncmp), %function

ARCH_LIBCFUN(strncmp):
	push	{r4, r5, r6, r7, lr}

	cmp	limit, #0
	beq	.Lret0

	movw	const_m1, #:lower16:0x01010101
	movt	const_m1, #:upper16:0x01010101
	movw	const_h, #:lower16:0x80808080
	movt	const_h, #:upper16:0x80808080

	/* Misaligned head: compare byte-by-byte until src1 is aligned or
	   the limit is exhausted.  */
	ands	tmp1, src1, #3
	beq	.Lword_loop

.Lhead_loop:
	ldrb	data1, [src1], #1
	ldrb	data2, [src2], #1
	cmp	data1, data2
	bne	.Ldiff_byte
	cmp	data1, #0
	beq	.Lret0
	subs	limit, limit, #1
	beq	.Lret0
	ands	tmp1, src1, #3
	bne	.Lhead_loop

	.align 2
.Lword_loop:
	cmp	limit, #4
	blo	.Lbyte_tail
	ldr	data1, [src1], #4
	ldr	data2, [src2], #4
	/* NUL check first: a NUL may end the string before any differing
	   byte within this word.  */
	sub	syndrome, data1, const_m1
	bic	syndrome, syndrome, data1
	tst	syndrome, const_h
	bne	.Lword_nul
	/* No NUL: a difference here is a real difference.  */
	cmp	data1, data2
	bne	.Ldiff_word
	subs	limit, limit, #4
	bne	.Lword_loop
	b	.Lret0		/* limit exhausted, equal */

.Lword_nul:
	/* NUL within this word: back up and finish byte-by-byte so that a
	   NUL preceding a difference correctly yields equality.  */
	sub	src1, src1, #4
	sub	src2, src2, #4

.Lbyte_tail:
	cmp	limit, #0
	beq	.Lret0
.Lbyte_loop:
	ldrb	data1, [src1], #1
	ldrb	data2, [src2], #1
	cmp	data1, data2
	bne	.Ldiff_byte
	cmp	data1, #0
	beq	.Lret0
	subs	limit, limit, #1
	bne	.Lbyte_loop

.Lret0:
	mov	result, #0
	pop	{r4, r5, r6, r7, pc}

	.align 2
.Ldiff_word:
	/* Find the first differing byte.  Reverse on little-endian so the
	   first byte dominates the compare.  */
#ifndef __ARM_BIG_ENDIAN
	rev	data1, data1
	rev	data2, data2
#endif
	cmp	data1, data2
	blo	.Lret_minus1
	b	.Lret_plus1

.Ldiff_byte:
	cmp	data1, data2
	blo	.Lret_minus1
	b	.Lret_plus1

.Lret_minus1:
	mvn	result, #0
	pop	{r4, r5, r6, r7, pc}

.Lret_plus1:
	mov	result, #1
	pop	{r4, r5, r6, r7, pc}

	.size ARCH_LIBCFUN(strncmp), . - ARCH_LIBCFUN(strncmp)

#endif
