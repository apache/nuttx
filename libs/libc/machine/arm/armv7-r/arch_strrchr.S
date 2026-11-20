/****************************************************************************
 * libs/libc/machine/arm/armv7-r/arch_strrchr.S
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

#ifdef LIBC_BUILD_STRRCHR

/* Assumptions:
 *
 * ARMv7-A / ARMv7-R (ARM state), unaligned word accesses supported.
 *
 * Word-at-a-time scan: the hot loop tests one word per iteration for a
 * NUL terminator and for the search byte.  The position of the last
 * match seen is tracked; on reaching NUL that position (or NULL) is
 * returned.
 */

#define src		r0
#define ch		r1
#define result		r0
#define data		r2
#define repc		r3
#define last		r4		/* last match pointer (or 0) */
#define syndrome	r12
#define tmp		r5
#define const_m1	r6
#define const_h		r7

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(strrchr)
	.arm
	.type ARCH_LIBCFUN(strrchr), %function

ARCH_LIBCFUN(strrchr):
	push	{r4, r5, r6, r7, lr}
	and	ch, ch, #0xff
	mov	last, #0

	movw	const_m1, #:lower16:0x01010101
	movt	const_m1, #:upper16:0x01010101
	movw	const_h, #:lower16:0x80808080
	movt	const_h, #:upper16:0x80808080
	mul	repc, ch, const_m1

	/* Byte-scan the misaligned head.  */
	ands	tmp, src, #3
	beq	.Lword_loop
.Lhead:
	ldrb	data, [src], #1
	cmp	data, ch
	subeq	last, src, #1		/* record match position */
	cmp	data, #0
	beq	.Lret
	ands	tmp, src, #3
	bne	.Lhead

	.p2align 2
.Lword_loop:
	ldr	data, [src], #4
	/* NUL test.  */
	sub	syndrome, data, const_m1
	bic	syndrome, syndrome, data
	tst	syndrome, const_h
	bne	.Lterm
	/* Search-byte test.  */
	eor	tmp, data, repc
	sub	syndrome, tmp, const_m1
	bic	syndrome, syndrome, tmp
	tst	syndrome, const_h
	bne	.Lmatch_word
	b	.Lword_loop

.Lmatch_word:
	/* A match exists in this word (no NUL): byte-scan it to update the
	   last-match pointer.  src points one past the word.  */
	sub	tmp, src, #4
.Lmatch_scan:
	ldrb	data, [tmp], #1
	cmp	data, ch
	subeq	last, tmp, #1
	cmp	tmp, src
	bne	.Lmatch_scan
	b	.Lword_loop

.Lterm:
	/* NUL in this word: byte-scan it tracking the last match until NUL.  */
	sub	src, src, #4
.Lterm_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	subeq	last, src, #1
	cmp	data, #0
	bne	.Lterm_scan

.Lret:
	mov	result, last
	pop	{r4, r5, r6, r7, pc}

	.size ARCH_LIBCFUN(strrchr), . - ARCH_LIBCFUN(strrchr)

#endif
