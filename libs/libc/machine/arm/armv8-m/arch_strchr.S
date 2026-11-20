/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_strchr.S
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

#ifdef LIBC_BUILD_STRCHR

/* Assumptions:
 *
 * ARMv7-M / ARMv8-M (Thumb-2).
 */

#ifndef __ARM_FEATURE_BTI_DEFAULT
#  define __ARM_FEATURE_BTI_DEFAULT 0
#endif

#ifndef __ARM_FEATURE_PAC_DEFAULT
#  define __ARM_FEATURE_PAC_DEFAULT 0
#endif

	.macro def_fn f p2align=0
	.text
	.p2align \p2align
	.global \f
	.type \f, %function
\f:
	.endm

#define src		r0
#define ch		r1
#define data		r2
#define tmp		r3
#define const_m1	r4
#define const_h		r5
#define repc		r6
#define syndrome	r7

	.thumb
	.syntax unified
def_fn ARCH_LIBCFUN(strchr)
	.cfi_sections .debug_frame
	.cfi_startproc
#if __ARM_FEATURE_PAC_DEFAULT
#  if __ARM_FEATURE_BTI_DEFAULT
	pacbti	ip, lr, sp
#  else
	pac	ip, lr, sp
#  endif
#endif
	push	{r4, r5, r6, r7, lr}
	.cfi_def_cfa_offset 20
	.cfi_offset lr, -4
	.cfi_offset r4, -20
	.cfi_offset r5, -16
	.cfi_offset r6, -12
	.cfi_offset r7, -8
	and	ch, ch, #0xff
	movw	const_m1, #:lower16:0x01010101
	movt	const_m1, #:upper16:0x01010101
	movw	const_h, #:lower16:0x80808080
	movt	const_h, #:upper16:0x80808080
	mul	repc, ch, const_m1	/* ch replicated to all 4 bytes */

	/* Byte-scan the misaligned head until src is word-aligned.  */
	ands	tmp, src, #3
	beq	.Lword_loop
.Lhead:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lret_found
	cmp	data, #0
	beq	.Lret_null
	ands	tmp, src, #3
	bne	.Lhead

	.p2align 2
.Lword_loop:
	ldr	data, [src], #4
	/* NUL test: syndrome = (data - const_m1) & ~data.  */
	sub	syndrome, data, const_m1
	bic	syndrome, syndrome, data
	tst	syndrome, const_h
	bne	.Lterm
	/* Search-byte test: zero-byte test on data ^ repc.  */
	eor	tmp, data, repc
	sub	syndrome, tmp, const_m1
	bic	syndrome, syndrome, tmp
	tst	syndrome, const_h
	bne	.Lfound
	b	.Lword_loop

.Lfound:
	/* Match present, no NUL in this word: rescan bytes to locate it.  */
	sub	src, src, #4
.Lfound_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lret_found
	cmp	data, #0
	bne	.Lfound_scan
	b	.Lret_null

.Lterm:
	/* NUL present: rescan bytes to see if the match precedes the NUL.  */
	sub	src, src, #4
.Lterm_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lret_found
	cmp	data, #0
	bne	.Lterm_scan

.Lret_null:
	mov	src, #0
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	pop	{r4, r5, r6, r7, pc}
.Lret_found:
	sub	src, src, #1		/* src was post-incremented */
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	pop	{r4, r5, r6, r7, pc}

	.cfi_endproc
	.size ARCH_LIBCFUN(strchr), . - ARCH_LIBCFUN(strchr)

#endif
