/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_memchr.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2010-2011, Linaro Limited, 2015 ARM Ltd
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
 * Written by Dave Gilbert <david.gilbert@linaro.org>
 *
 * This memchr routine is optimised on a Cortex-A9 and should work on
 * all ARMv7 processors.   It has a fast path for short sizes, and has
 * an optimised path for large data sets; the worst case is finding the
 * match early in a large data set.
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

#ifdef LIBC_BUILD_MEMCHR

#ifndef __ARM_FEATURE_BTI_DEFAULT
#  define __ARM_FEATURE_BTI_DEFAULT 0
#endif

#ifndef __ARM_FEATURE_PAC_DEFAULT
#  define __ARM_FEATURE_PAC_DEFAULT 0
#endif

@ 2011-02-07 david.gilbert@linaro.org
@    Extracted from local git a5b438d861
@ 2011-07-14 david.gilbert@linaro.org
@    Import endianness fix from local git ea786f1b
@ 2011-10-11 david.gilbert@linaro.org
@    Import from cortex-strings bzr rev 63
@    Flip to ldrd (as suggested by Greta Yorsh)
@    Make conditional on CPU type
@    tidy

@ This code requires armv6t2 or later.  Uses Thumb2.

	.syntax unified

#include "acle-compat.h"

/* Pointer-authentication prologue shared by all variants.  */
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

/* MVE path: scan 16 bytes per iteration.  XOR each loaded vector with the
 * replicated search byte (matching bytes become zero), then take the
 * horizontal minimum.  VMINV computes min(Rda, min(Qm)), so Rda must be
 * initialized to 0xFF (the largest unsigned byte) before the loop so the
 * result reflects only the vector contents.
 */

#define src		r0
#define ch		r1
#define n		r2
#define chunks		r3
#define data		r4
#define tail		r5

	.thumb
	.syntax unified
	.global ARCH_LIBCFUN(memchr)
	.thumb_func
	.type ARCH_LIBCFUN(memchr), %function
ARCH_LIBCFUN(memchr):
	.cfi_sections .debug_frame
	.cfi_startproc
	pac_prologue
	push	{r4, r5, lr}
	.cfi_def_cfa_offset 12
	.cfi_offset lr, -4
	.cfi_offset r4, -12
	.cfi_offset r5, -8

	cbz	n, .Lmve_nf
	and	ch, ch, #0xff
	vdup.8	q0, ch
	lsrs	chunks, n, #4		/* number of 16-byte chunks */
	beq	.Lmve_tail
	movs	data, #0xff		/* VMINV starting value: max unsigned byte */
.Lmve_loop:
	vldrb.8	q1, [src], #16
	veor	q1, q1, q0		/* matching bytes become zero */
	vminv.u8 data, q1		/* min(0xff, min(q1)) = 0 iff match */
	cmp	data, #0
	beq	.Lmve_found
	subs	chunks, chunks, #1
	bne	.Lmve_loop
.Lmve_tail:
	ands	tail, n, #15
	beq	.Lmve_nf
.Lmve_tail_loop:
	ldrb	data, [src], #1
	cmp	data, ch
	beq	.Lmve_found_ret
	subs	tail, tail, #1
	bne	.Lmve_tail_loop
.Lmve_nf:
	mov	src, #0
	pop	{r4, r5, lr}
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	bx	lr
.Lmve_found:
	sub	src, src, #16		/* back to the start of the block */
.Lmve_found_scan:
	ldrb	data, [src], #1
	cmp	data, ch
	bne	.Lmve_found_scan
.Lmve_found_ret:
	sub	src, src, #1		/* src was post-incremented */
	pop	{r4, r5, lr}
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	bx	lr

	.cfi_endproc
	.size ARCH_LIBCFUN(memchr), . - ARCH_LIBCFUN(memchr)

#elif defined (__ARM_NEON__) || defined (__ARM_NEON)
@ NOTE: This ifdef MUST match the one in memchr-stub.c
#if __ARM_ARCH >= 8 && __ARM_ARCH_PROFILE == 'R'
	.arch	armv8-r
#else
	.arch	armv7-a
#endif
	.fpu	neon


/* Arguments */
#define srcin		r0
#define chrin		r1
#define cntin		r2

/* Retval */
#define result		r0	/* Live range does not overlap with srcin */

/* Working registers */
#define src		r1	/* Live range does not overlap with chrin */
#define tmp		r3
#define synd		r0	/* No overlap with srcin or result */
#define soff		r12

/* Working NEON registers */
#define vrepchr		q0
#define vdata0		q1
#define vdata0_0	d2	/* Lower half of vdata0 */
#define vdata0_1	d3	/* Upper half of vdata0 */
#define vdata1		q2
#define vdata1_0	d4	/* Lower half of vhas_chr0 */
#define vdata1_1	d5	/* Upper half of vhas_chr0 */
#define vrepmask	q3
#define vrepmask0	d6
#define vrepmask1	d7
#define vend		q4
#define vend0		d8
#define vend1		d9

/*
 * Core algorithm:
 *
 * For each 32-byte chunk we calculate a 32-bit syndrome value, with one bit per
 * byte. Each bit is set if the relevant byte matched the requested character
 * and cleared otherwise. Since the bits in the syndrome reflect exactly the
 * order in which things occur in the original string, counting trailing zeros
 * allows to identify exactly which byte has matched.
 */

	.text
	.thumb_func
	.align 4
	.p2align 4,,15
	.global ARCH_LIBCFUN(memchr)
	.type ARCH_LIBCFUN(memchr),%function

ARCH_LIBCFUN(memchr):
	.cfi_sections .debug_frame
	.cfi_startproc
	pac_prologue
	/* Use a simple loop if there are less than 8 bytes to search.  */
	cmp	cntin, #7
	bhi	.Llargestr
	and	chrin, chrin, #0xff

.Lsmallstr:
	subs	cntin, cntin, #1
	blo	.Lnotfound	/* Return not found if reached end.  */
	ldrb	tmp, [srcin], #1
	cmp	tmp, chrin
	bne	.Lsmallstr	/* Loop again if not found.  */
	/* Otherwise fixup address and return.  */
	sub	result, result, #1
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr


.Llargestr:
	vdup.8	vrepchr, chrin	/* Duplicate char across all lanes. */
	/*
	 * Magic constant 0x8040201008040201 allows us to identify which lane
	 * matches the requested byte.
	 */
	movw	tmp, #0x0201
	movt	tmp, #0x0804
	lsl	soff, tmp, #4
	vmov	vrepmask0, tmp, soff
	vmov	vrepmask1, tmp, soff
	/* Work with aligned 32-byte chunks */
	bic	src, srcin, #31
	ands	soff, srcin, #31
	beq	.Lloopintro	/* Go straight to main loop if it's aligned. */

	/*
	 * Input string is not 32-byte aligned. We calculate the syndrome
	 * value for the aligned 32 bytes block containing the first bytes
	 * and mask the irrelevant part.
	 */
	vld1.8		{vdata0, vdata1}, [src:256]!
	sub		tmp, soff, #32
	adds		cntin, cntin, tmp
	vceq.i8		vdata0, vdata0, vrepchr
	vceq.i8		vdata1, vdata1, vrepchr
	vand		vdata0, vdata0, vrepmask
	vand		vdata1, vdata1, vrepmask
	vpadd.i8	vdata0_0, vdata0_0, vdata0_1
	vpadd.i8	vdata1_0, vdata1_0, vdata1_1
	vpadd.i8	vdata0_0, vdata0_0, vdata1_0
	vpadd.i8	vdata0_0, vdata0_0, vdata0_0
	vmov		synd, vdata0_0[0]

	/* Clear the soff lower bits */
	lsr		synd, synd, soff
	lsl		synd, synd, soff
	/* The first block can also be the last */
	bls		.Lmasklast
	/* Have we found something already? */
	cbnz		synd, .Ltail


.Lloopintro:
	vpush	{vend}
	/* 264/265 correspond to d8/d9 for q4 */
	.cfi_adjust_cfa_offset	16
	.cfi_rel_offset	264, 0
	.cfi_rel_offset	265, 8
	.p2align 3,,7
.Lloop:
	vld1.8		{vdata0, vdata1}, [src:256]!
	subs		cntin, cntin, #32
	vceq.i8		vdata0, vdata0, vrepchr
	vceq.i8		vdata1, vdata1, vrepchr
	/* If we're out of data we finish regardless of the result. */
	bls		.Lend
	/* Use a fast check for the termination condition. */
	vorr		vend, vdata0, vdata1
	vorr		vend0, vend0, vend1
	vmov		synd, tmp, vend0
	orrs		synd, synd, tmp
	/* We're not out of data, loop if we haven't found the character. */
	beq		.Lloop

.Lend:
	vpop		{vend}
	.cfi_adjust_cfa_offset	-16
	.cfi_restore	264
	.cfi_restore	265

	/* Termination condition found, let's calculate the syndrome value. */
	vand		vdata0, vdata0, vrepmask
	vand		vdata1, vdata1, vrepmask
	vpadd.i8	vdata0_0, vdata0_0, vdata0_1
	vpadd.i8	vdata1_0, vdata1_0, vdata1_1
	vpadd.i8	vdata0_0, vdata0_0, vdata1_0
	vpadd.i8	vdata0_0, vdata0_0, vdata0_0
	vmov		synd, vdata0_0[0]
	cbz		synd, .Lnotfound
	bhi		.Ltail


.Lmasklast:
	/* Clear the (-cntin) upper bits to avoid out-of-bounds matches. */
	neg	cntin, cntin
	lsl	synd, synd, cntin
	lsrs	synd, synd, cntin
	it	eq
	moveq	src, #0	/* If no match, set src to 0 so the retval is 0. */


.Ltail:
	/* Count the trailing zeros using bit reversing */
	rbit	synd, synd
	/* Compensate the last post-increment */
	sub	src, src, #32
	/* Count the leading zeros */
	clz	synd, synd
	/* Compute the potential result and return */
	add	result, src, synd
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr


.Lnotfound:
	/* Set result to NULL if not found and return */
	mov	result, #0
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr

	.cfi_endproc
	.size	ARCH_LIBCFUN(memchr), . - ARCH_LIBCFUN(memchr)

#elif __ARM_ARCH_ISA_THUMB >= 2 && defined (__ARM_FEATURE_DSP)

#if __ARM_ARCH_PROFILE == 'M'
       .arch armv7e-m
#else
       .arch armv6t2
#endif

@ this lets us check a flag in a 00/ff byte easily in either endianness
#ifdef __ARMEB__
#define CHARTSTMASK(c) 1<<(31-(c*8))
#else
#define CHARTSTMASK(c) 1<<(c*8)
#endif
	.text
	.thumb

@ ---------------------------------------------------------------------------
	.thumb_func
	.align 2
	.p2align 4,,15
	.global ARCH_LIBCFUN(memchr)
	.type ARCH_LIBCFUN(memchr),%function
ARCH_LIBCFUN(memchr):
	.cfi_sections .debug_frame
	.cfi_startproc
	@ r0 = start of memory to scan
	@ r1 = character to look for
	@ r2 = length
	@ returns r0 = pointer to character or NULL if not found
	pac_prologue
	and	r1,r1,#0xff	@ Don't trust the caller to pass a char

	cmp	r2,#16		@ If short don't bother with anything clever
	blt	20f

	tst	r0, #7		@ If it's already aligned skip the next bit
	beq	10f

	@ Work up to an aligned point
5:
	ldrb	r3, [r0],#1
	subs	r2, r2, #1
	cmp	r3, r1
	beq	50f		@ If it matches exit found
	tst	r0, #7
	cbz	r2, 40f		@ If we run off the end, exit not found
	bne	5b		@ If not aligned yet then do next byte

10:
	@ We are aligned, we know we have at least 8 bytes to work with
	push	{r4,r5,r6,r7}
	.cfi_def_cfa_offset 16
	.cfi_offset r4, -16
	.cfi_offset r5, -12
	.cfi_offset r6, -8
	.cfi_offset r7, -4
	orr	r1, r1, r1, lsl #8	@ expand the match word across all bytes
	orr	r1, r1, r1, lsl #16
	bic	r4, r2, #7	@ Number of double words to work with * 8
	mvns	r7, #0		@ all F's
	movs	r3, #0

15:
	ldrd    r5,r6,[r0],#8
	subs	r4, r4, #8
	eor	r5,r5, r1	@ r5,r6 have 00's where bytes match the target
	eor	r6,r6, r1
	uadd8	r5, r5, r7	@ Par add 0xff - sets GE bits for bytes!=0
	sel	r5, r3, r7	@ bytes are 00 for none-00 bytes,
				@ or ff for 00 bytes - NOTE INVERSION
	uadd8	r6, r6, r7	@ Par add 0xff - sets GE bits for bytes!=0
	sel	r6, r5, r7	@ chained....bytes are 00 for none-00 bytes
				@ or ff for 00 bytes - NOTE INVERSION
	cbnz	r6, 60f
	bne	15b		@ (Flags from the subs above)

	pop	{r4,r5,r6,r7}
	.cfi_def_cfa_offset 0
	.cfi_restore r4
	.cfi_restore r5
	.cfi_restore r6
	.cfi_restore r7
	and	r1,r1,#0xff	@ r1 back to a single character
	and	r2,r2,#7	@ Leave the count remaining as the number
				@ after the double words have been done

20:
	cbz	r2, 40f		@ 0 length or hit the end already then not found

21:  @ Post aligned section, or just a short call
	ldrb	r3,[r0],#1
	subs	r2,r2,#1
	eor	r3,r3,r1	@ r3 = 0 if match - doesn't break flags from sub
	cbz	r3, 50f
	bne	21b		@ on r2 flags

40:
	movs	r0,#0		@ not found
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr

50:
	subs	r0,r0,#1	@ found
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr

60:  @ We're here because the fast path found a hit
     @ now we have to track down exactly which word it was
	@ r0 points to the start of the double word after the one tested
	@ r5 has the 00/ff pattern for the first word, r6 has the chained value
	cmp	r5, #0
	itte	eq
	moveq	r5, r6		@ the end is in the 2nd word
	subeq	r0,r0,#3	@ Points to 2nd byte of 2nd word
	subne	r0,r0,#7	@ or 2nd byte of 1st word

	@ r0 currently points to the 2nd byte of the word containing the hit
	tst	r5, # CHARTSTMASK(0)	@ 1st character
	bne	61f
	adds	r0,r0,#1
	tst	r5, # CHARTSTMASK(1)	@ 2nd character
	ittt	eq
	addeq	r0,r0,#1
	tsteq	r5, # (3<<15)		@ 2nd & 3rd character
	@ If not the 3rd must be the last one
	addeq	r0,r0,#1

61:
	pop	{r4,r5,r6,r7}
	.cfi_def_cfa_offset 0
	.cfi_restore r4
	.cfi_restore r5
	.cfi_restore r6
	.cfi_restore r7
	subs	r0,r0,#1
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif /* __ARM_FEATURE_PAC_DEFAULT */
	bx	lr
	.cfi_endproc
#else
  /* Defined in memchr-stub.c.  */
#endif

#endif
