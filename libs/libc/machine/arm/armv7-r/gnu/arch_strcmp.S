/****************************************************************************
 * libs/libc/machine/arm/armv7-r/gnu/arch_strcmp.S
 *
 * Copyright (c) 2011 The Android Open Source Project
 * Copyright (c) 2008 ARM Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ARM LTD ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ARM LTD BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifdef __ARMEB__
#define SHFT2LSB lsl
#define SHFT2LSBEQ lsleq
#define SHFT2MSB lsr
#define SHFT2MSBEQ lsreq
#define MSB 0x000000ff
#define LSB 0xff000000
#else
#define SHFT2LSB lsr
#define SHFT2LSBEQ lsreq
#define SHFT2MSB lsl
#define SHFT2MSBEQ lsleq
#define MSB 0xff000000
#define LSB 0x000000ff
#endif
#define magic1(REG) REG
#define magic2(REG) REG, lsl #7

	.arm
	.syntax unified
	.global strcmp
	.type	strcmp, %function
strcmp:
	pld	[r0, #0]
	pld	[r1, #0]
	eor	r2, r0, r1
	tst	r2, #3
	/* Strings not at same byte offset from a word boundary.  */
	bne	.Lstrcmp_unaligned
	ands	r2, r0, #3
	bic	r0, r0, #3
	bic	r1, r1, #3
	ldr	ip, [r0], #4
	it	eq
	ldreq	r3, [r1], #4
	beq	1f
	/* Although s1 and s2 have identical initial alignment, they are
	 * not currently word aligned.  Rather than comparing bytes,
	 * make sure that any bytes fetched from before the addressed
	 * bytes are forced to 0xff.  Then they will always compare
	 * equal.
	 */
	eor	r2, r2, #3
	lsl	r2, r2, #3
	mvn	r3, #MSB
	SHFT2LSB	r2, r3, r2
	ldr	r3, [r1], #4
	orr	ip, ip, r2
	orr	r3, r3, r2
1:
	/* Load the 'magic' constant 0x01010101. */
	str	r4, [sp, #-4]!
	mov	r4, #1
	orr	r4, r4, r4, lsl #8
	orr	r4, r4, r4, lsl #16
	.p2align	2
4:
	pld	[r0, #8]
	pld	[r1, #8]
	sub	r2, ip, magic1(r4)
	cmp	ip, r3
	itttt	eq
	/* check for any zero bytes in first word */
	biceq	r2, r2, ip
	tsteq	r2, magic2(r4)
	ldreq	ip, [r0], #4
	ldreq	r3, [r1], #4
	beq	4b
2:
	/* There's a zero or a different byte in the word */
	SHFT2MSB	r0, ip, #24
	SHFT2LSB	ip, ip, #8
	cmp	r0, #1
	it	cs
	cmpcs	r0, r3, SHFT2MSB #24
	it	eq
	SHFT2LSBEQ r3, r3, #8
	beq	2b
	/* On a big-endian machine, r0 contains the desired byte in bits
	 * 0-7; on a little-endian machine they are in bits 24-31.  In
	 * both cases the other bits in r0 are all zero.  For r3 the
	 * interesting byte is at the other end of the word, but the
	 * other bits are not necessarily zero.  We need a signed result
	 * representing the difference in the unsigned bytes, so for the
	 * little-endian case we can't just shift the interesting bits up.
	 */
#ifdef __ARMEB__
	sub	r0, r0, r3, lsr #24
#else
	and	r3, r3, #255
	/* No RSB instruction in Thumb2 */
#ifdef __thumb2__
	lsr	r0, r0, #24
	sub	r0, r0, r3
#else
	rsb	r0, r3, r0, lsr #24
#endif
#endif
	ldr	r4, [sp], #4
	bx	lr
.Lstrcmp_unaligned:
	wp1 .req r0
	wp2 .req r1
	b1  .req r2
	w1  .req r4
	w2  .req r5
	t1  .req ip
	@ r3 is scratch
	/* First of all, compare bytes until wp1(sp1) is word-aligned. */
1:
	tst	wp1, #3
	beq	2f
	ldrb	r2, [wp1], #1
	ldrb	r3, [wp2], #1
	cmp	r2, #1
	it	cs
	cmpcs	r2, r3
	beq	1b
	sub	r0, r2, r3
	bx	lr
2:
	str	r5, [sp, #-4]!
	str	r4, [sp, #-4]!
	mov	b1, #1
	orr	b1, b1, b1, lsl #8
	orr	b1, b1, b1, lsl #16
	and	t1, wp2, #3
	bic	wp2, wp2, #3
	ldr	w1, [wp1], #4
	ldr	w2, [wp2], #4
	cmp	t1, #2
	beq	2f
	bhi	3f
	/* Critical inner Loop: Block with 3 bytes initial overlap */
	.p2align	2
1:
	bic	t1, w1, #MSB
	cmp	t1, w2, SHFT2LSB #8
	sub	r3, w1, b1
	bic	r3, r3, w1
	bne	4f
	ands	r3, r3, b1, lsl #7
	it	eq
	ldreq	w2, [wp2], #4
	bne	5f
	eor	t1, t1, w1
	cmp	t1, w2, SHFT2MSB #24
	bne	6f
	ldr	w1, [wp1], #4
	b	1b
4:
	SHFT2LSB	w2, w2, #8
	b	8f
5:
#ifdef __ARMEB__
	/* The syndrome value may contain false ones if the string ends
	 * with the bytes 0x01 0x00
	 */
	tst	w1, #0xff000000
	itt	ne
	tstne	w1, #0x00ff0000
	tstne	w1, #0x0000ff00
	beq	7f
#else
	bics	r3, r3, #0xff000000
	bne	7f
#endif
	ldrb	w2, [wp2]
	SHFT2LSB	t1, w1, #24
#ifdef __ARMEB__
	lsl	w2, w2, #24
#endif
	b	8f
6:
	SHFT2LSB	t1, w1, #24
	and	w2, w2, #LSB
	b	8f
	/* Critical inner Loop: Block with 2 bytes initial overlap */
	.p2align	2
2:
	SHFT2MSB	t1, w1, #16
	sub	r3, w1, b1
	SHFT2LSB	t1, t1, #16
	bic	r3, r3, w1
	cmp	t1, w2, SHFT2LSB #16
	bne	4f
	ands	r3, r3, b1, lsl #7
	it	eq
	ldreq	w2, [wp2], #4
	bne	5f
	eor	t1, t1, w1
	cmp	t1, w2, SHFT2MSB #16
	bne	6f
	ldr	w1, [wp1], #4
	b	2b
5:
#ifdef __ARMEB__
	/* The syndrome value may contain false ones if the string ends
	 * with the bytes 0x01 0x00
	 */
	tst	w1, #0xff000000
	it	ne
	tstne	w1, #0x00ff0000
	beq	7f
#else
	lsls	r3, r3, #16
	bne	7f
#endif
	ldrh	w2, [wp2]
	SHFT2LSB	t1, w1, #16
#ifdef __ARMEB__
	lsl	w2, w2, #16
#endif
	b	8f
6:
	SHFT2MSB	w2, w2, #16
	SHFT2LSB	t1, w1, #16
4:
	SHFT2LSB	w2, w2, #16
	b	8f
	/* Critical inner Loop: Block with 1 byte initial overlap */
	.p2align	2
3:
	and	t1, w1, #LSB
	cmp	t1, w2, SHFT2LSB #24
	sub	r3, w1, b1
	bic	r3, r3, w1
	bne	4f
	ands	r3, r3, b1, lsl #7
	it	eq
	ldreq	w2, [wp2], #4
	bne	5f
	eor	t1, t1, w1
	cmp	t1, w2, SHFT2MSB #8
	bne	6f
	ldr	w1, [wp1], #4
	b	3b
4:
	SHFT2LSB	w2, w2, #24
	b	8f
5:
	/* The syndrome value may contain false ones if the string ends
	 * with the bytes 0x01 0x00
	 */
	tst	w1, #LSB
	beq	7f
	ldr	w2, [wp2], #4
6:
	SHFT2LSB	t1, w1, #8
	bic	w2, w2, #MSB
	b	8f
7:
	mov	r0, #0
	ldr	r4, [sp], #4
	ldr	r5, [sp], #4
	bx	lr
8:
	and	r2, t1, #LSB
	and	r0, w2, #LSB
	cmp	r0, #1
	it	cs
	cmpcs	r0, r2
	itt	eq
	SHFT2LSBEQ	t1, t1, #8
	SHFT2LSBEQ	w2, w2, #8
	beq	8b
	sub	r0, r2, r0
	ldr	r4, [sp], #4
	ldr	r5, [sp], #4
	bx	lr
	.size strcmp, . - strcmp
