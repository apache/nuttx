/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_strrchr.S
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
#define last		r7

	.thumb
	.syntax unified
def_fn ARCH_LIBCFUN(strrchr)
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
	mov	last, #0
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
	bne	.Lh1
	sub	last, src, #1		/* record match (src post-incremented) */
.Lh1:
	cmp	data, #0
	beq	.Lret
	ands	tmp, src, #3
	bne	.Lhead

	.p2align 2
.Lword_loop:
	ldr	data, [src], #4
	/* NUL test: syndrome = (data - const_m1) & ~data.  */
	sub	tmp, data, const_m1
	bic	tmp, tmp, data
	tst	tmp, const_h
	bne	.Lterm
	/* Search-byte test: zero-byte test on data ^ repc.  */
	eor	data, data, repc
	sub	tmp, data, const_m1
	bic	tmp, tmp, data
	tst	tmp, const_h
	bne	.Lmatch_word
	b	.Lword_loop

.Lmatch_word:
	/* Match (no NUL): rescan the 4 bytes to update last.  */
	sub	tmp, src, #4
.Lmw_scan:
	ldrb	data, [tmp], #1
	cmp	data, ch
	bne	.Lmw1
	sub	last, tmp, #1
.Lmw1:
	cmp	tmp, src
	bne	.Lmw_scan
	b	.Lword_loop

.Lterm:
	/* NUL in this word: rescan bytes tracking last match until NUL.  */
	sub	src, src, #4
.Lterm_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	bne	.Lt1
	sub	last, src, #1
.Lt1:
	cmp	data, #0
	bne	.Lterm_scan

.Lret:
	mov	src, last
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	pop	{r4, r5, r6, r7, pc}

	.cfi_endproc
	.size ARCH_LIBCFUN(strrchr), . - ARCH_LIBCFUN(strrchr)

#endif
