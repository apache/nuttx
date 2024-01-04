/****************************************************************************
 * libs/libc/machine/arm/armv7-m/gnu/arch_memset.S
 *
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_MEMSET

	.thumb
	.syntax unified
	.text
	.section .text.memset
	.global memset
	.type	memset, %function
memset:
	stmfd	sp!, {r0, r4-r7, lr}
	rsb	r3, r0, #0
	ands	r3, r3, #3
	cmp	r3, r2
	movhi	r3, r2

	/* splat r1 */

	mov	r1, r1, lsl #24
	orr	r1, r1, r1, lsr #8
	orr	r1, r1, r1, lsr #16

	movs	r12, r3, lsl #31
	strbcs	r1, [r0], #1
	strbcs	r1, [r0], #1
	strbmi	r1, [r0], #1
	subs	r2, r2, r3
	popls	{r0, r4-r7, pc}

	/* align the destination to a cache-line */

	mov	r12, r1
	mov	lr, r1
	mov	r4, r1
	mov	r5, r1
	mov	r6, r1
	mov	r7, r1

	rsb	r3, r0, #0
	ands	r3, r3, #0x1C
	beq	3f
	cmp	r3, r2
	andhi	r3, r2, #0x1C
	sub	r2, r2, r3

	/* conditionally writes 0 to 7 words (length in r3) */

	movs	r3, r3, lsl #28
	stmcs	r0!, {r1, lr}
	stmcs	r0!, {r1, lr}
	stmmi	r0!, {r1, lr}
	movs	r3, r3, lsl #2
	strcs	r1, [r0], #4

3:
	subs	r2, r2, #32
	mov	r3, r1
	bmi	2f
1:
	subs	r2, r2, #32
	stmia	r0!, {r1,r3,r4,r5,r6,r7,r12,lr}
	bhs	1b
2:
	add	r2, r2, #32

	/* conditionally stores 0 to 31 bytes */

	movs	r2, r2, lsl #28
	stmcs	r0!, {r1,r3,r12,lr}
	stmmi	r0!, {r1, lr}
	movs	r2, r2, lsl #2
	strcs	r1, [r0], #4
	strhmi	r1, [r0], #2
	movs	r2, r2, lsl #2
	strbcs	r1, [r0]
	ldmfd	sp!, {r0, r4-r7, pc}
	.size memset, . - memset

#endif
