/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_strlen.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2010-2011,2013 Linaro Limited, 2015 ARM Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of Linaro Limited nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Assumes:
 * ARMv6T2 or ARMv7E-M, AArch32
 *
 * Copyright (c) 2015 ARM Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Linaro nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_STRLEN

#include "acle-compat.h"

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

#ifdef __ARMEB__
#define S2LO		lsl
#define S2HI		lsr
#else
#define S2LO		lsr
#define S2HI		lsl
#endif

/* Pointer-authentication prologue shared by both variants.  */
	.macro pac_prologue
#if __ARM_FEATURE_PAC_DEFAULT
#  if __ARM_FEATURE_BTI_DEFAULT
	pacbti	ip, lr, sp
#  else
	pac	ip, lr, sp
#  endif
#endif
	.endm

#if defined(__ARM_FEATURE_MVE)

/* MVE path: scan one 16-byte aligned block per iteration.  The head is
 * scanned byte-by-byte until aligned (so the MVE loads never cross a page
 * boundary); the horizontal minimum is zero iff a NUL is present.  On a
 * hit the block is re-scanned byte-by-byte to locate the NUL.
 */

#define s		r0		/* start, kept for length */
#define p		r1		/* moving pointer */
#define data		r2
#define tmp		r3

	.thumb
	.syntax unified
	.global ARCH_LIBCFUN(strlen)
	.thumb_func
	.type ARCH_LIBCFUN(strlen), %function
ARCH_LIBCFUN(strlen):
	.cfi_sections .debug_frame
	.cfi_startproc
	pac_prologue
	mov	p, s			/* p = start of string */

	/* Byte-scan the misaligned head until aligned or NUL.  */
.Lmve_head:
	ldrb	data, [p]
	cmp	data, #0
	beq	.Lmve_ret
	adds	p, p, #1
	ands	tmp, p, #15
	bne	.Lmve_head

.Lmve_loop:
	vldrb.8	q0, [p], #16
	vminv.u8 data, q0		/* 0 iff a NUL is present */
	cmp	data, #0
	beq	.Lmve_found
	b	.Lmve_loop

.Lmve_found:
	subs	p, p, #16		/* back to the start of the block */
.Lmve_block_scan:
	ldrb	data, [p]
	cmp	data, #0
	beq	.Lmve_ret
	adds	p, p, #1
	b	.Lmve_block_scan

.Lmve_ret:
	subs	s, p, s			/* length = NUL position - start */
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	bx	lr

	.cfi_endproc
	.size ARCH_LIBCFUN(strlen), . - ARCH_LIBCFUN(strlen)

#else

	/* This code requires Thumb.  */
#if __ARM_ARCH_PROFILE == 'M'
	.arch   armv7e-m
#else
	.arch	armv6t2
#endif
	.eabi_attribute Tag_ARM_ISA_use, 0
	.thumb
	.syntax unified

/* Parameters and result.  */
#define srcin		r0
#define result		r0

/* Internal variables.  */
#define src		r1
#define data1a		r2
#define data1b		r3
#define const_m1	r12
#define const_0		r4
#define tmp1		r4		/* Overlaps const_0  */
#define tmp2		r5

def_fn	ARCH_LIBCFUN(strlen) p2align=6
	pac_prologue
	pld	[srcin, #0]
	strd	r4, r5, [sp, #-8]!
	bic	src, srcin, #7
	mvn	const_m1, #0
	ands	tmp1, srcin, #7		/* (8 - bytes) to alignment.  */
	pld	[src, #32]
	bne.w	.Lmisaligned8
	mov	const_0, #0
	mov	result, #-8
.Lloop_aligned:
	/* Bytes 0-7.  */
	ldrd	data1a, data1b, [src]
	pld	[src, #64]
	add	result, result, #8
.Lstart_realigned:
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cbnz	data1b, .Lnull_found

	/* Bytes 8-15.  */
	ldrd	data1a, data1b, [src, #8]
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	add	result, result, #8
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cbnz	data1b, .Lnull_found

	/* Bytes 16-23.  */
	ldrd	data1a, data1b, [src, #16]
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	add	result, result, #8
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cbnz	data1b, .Lnull_found

	/* Bytes 24-31.  */
	ldrd	data1a, data1b, [src, #24]
	add	src, src, #32
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	add	result, result, #8
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cmp	data1b, #0
	beq	.Lloop_aligned

.Lnull_found:
	cmp	data1a, #0
	itt	eq
	addeq	result, result, #4
	moveq	data1a, data1b
#ifndef __ARMEB__
	rev	data1a, data1a
#endif
	clz	data1a, data1a
	ldrd	r4, r5, [sp], #8
	add	result, result, data1a, lsr #3	/* Bits -> Bytes.  */
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr

.Lmisaligned8:
	ldrd	data1a, data1b, [src]
	and	tmp2, tmp1, #3
	rsb	result, tmp1, #0
	lsl	tmp2, tmp2, #3			/* Bytes -> bits.  */
	tst	tmp1, #4
	pld	[src, #64]
	S2HI	tmp2, const_m1, tmp2
	orn	data1a, data1a, tmp2
	itt	ne
	ornne	data1b, data1b, tmp2
	movne	data1a, const_m1
	mov	const_0, #0
	b	.Lstart_realigned
	.size	ARCH_LIBCFUN(strlen), . - ARCH_LIBCFUN(strlen)

#endif /* __ARM_FEATURE_MVE */

#endif
