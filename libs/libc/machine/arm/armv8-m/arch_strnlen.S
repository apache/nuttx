/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_strnlen.S
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
#define maxlen		r1
#define data		r2
#define tmp		r3
#define const_m1	r4
#define const_h		r5
#define ptr		r6

	.thumb
	.syntax unified
def_fn ARCH_LIBCFUN(strnlen)
	.cfi_sections .debug_frame
	.cfi_startproc
#if __ARM_FEATURE_PAC_DEFAULT
#  if __ARM_FEATURE_BTI_DEFAULT
	pacbti	ip, lr, sp
#  else
	pac	ip, lr, sp
#  endif
#endif
	push	{r4, r5, r6, lr}
	.cfi_def_cfa_offset 16
	.cfi_offset lr, -4
	.cfi_offset r4, -16
	.cfi_offset r5, -12
	.cfi_offset r6, -8
	movw	const_m1, #:lower16:0x01010101
	movt	const_m1, #:upper16:0x01010101
	movw	const_h, #:lower16:0x80808080
	movt	const_h, #:upper16:0x80808080
	mov	ptr, src			/* ptr = src */
	cbz	maxlen, .Lzero
	/* Byte-scan the misaligned head until ptr is word-aligned.  */
	ands	tmp, ptr, #3
	beq	.Lcheck4
.Lhead:
	ldrb	data, [ptr]
	cbz	data, .Ldone			/* NUL found, ptr at NUL */
	adds	ptr, #1
	subs	maxlen, #1
	beq	.Ldone				/* maxlen exhausted */
	ands	tmp, ptr, #3
	bne	.Lhead
.Lcheck4:
	cmp	maxlen, #4
	blo	.Lbyte_tail
.Lwl:
	ldr	data, [ptr]
	sub	tmp, data, const_m1
	bic	tmp, tmp, data
	tst	tmp, const_h
	bne	.Lnulword			/* NUL in this word */
	adds	ptr, #4
	subs	maxlen, #4
	cmp	maxlen, #4
	bhs	.Lwl
.Lbyte_tail:
	cbz	maxlen, .Ldone
.Lbloop:
	ldrb	data, [ptr]
	cbz	data, .Ldone
	adds	ptr, #1
	subs	maxlen, #1
	bne	.Lbloop
	b	.Ldone
.Lnulword:
	/* NUL in this word (ptr at word start): rescan bytes to locate it.  */
.Lnul_scan:
	ldrb	data, [ptr]
	cbz	data, .Ldone
	adds	ptr, #1
	b	.Lnul_scan
.Ldone:
	sub	src, ptr, src			/* len = ptr - src */
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	pop	{r4, r5, r6, pc}
.Lzero:
	mov	src, #0
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	pop	{r4, r5, r6, pc}

	.cfi_endproc
	.size ARCH_LIBCFUN(strnlen), . - ARCH_LIBCFUN(strnlen)

#endif
