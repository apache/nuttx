/****************************************************************************
 * libs/libc/machine/arm/armv7-a/arch_memmove.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2015 ARM Ltd
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

#include "libc.h"

#ifdef LIBC_BUILD_MEMMOVE

/* Assumptions:
 *
 * ARMv7-A / ARMv7-R (ARM state), unaligned word accesses supported.
 * Word-at-a-time copy: forward when dst <= src, backward when dst > src
 * (overlapping).  The previous implementation was a naive byte loop.
 */

#define dst		r0
#define src		r1
#define n		r2
#define result		r0
#define wd		r3
#define cnt		r4
#define tail		r5
#define data		r12

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(memmove)
	.arm
	.type ARCH_LIBCFUN(memmove), %function

ARCH_LIBCFUN(memmove):
	push	{r4, r5, lr}

	cmp	n, #0
	beq	.Lret

	cmp	dst, src
	bcc	.Lforward		/* dst < src: forward */

	/* dst >= src: overlap only when dst < src + n */
	add	r3, src, n
	cmp	dst, r3
	bcs	.Lforward		/* dst >= src + n: no overlap */

	/* Backward copy (dst > src, overlapping).  Byte loop matches the
	 * generic C fallback so we never regress on the overlapping case.  */
	add	wd, dst, n
	add	src, src, n
.Lbw_loop:
	ldrb	data, [src, #-1]!
	strb	data, [wd, #-1]!
	subs	n, n, #1
	bne	.Lbw_loop
	b	.Lret

.Lforward:
	mov	wd, dst			/* preserve dst for return value */
#ifdef __ARM_NEON__
	/* NEON 16-byte block copy (d0/q0 is caller-saved scratch).  */
	lsrs	cnt, n, #4		/* number of 16-byte blocks */
	beq	.Lfw_byte_tail
.Lfw_neon_loop:
	vld1.8	{q0}, [src]!
	vst1.8	{q0}, [wd]!
	subs	cnt, cnt, #1
	bne	.Lfw_neon_loop
	ands	n, n, #15		/* remaining 0..15 bytes */
	beq	.Lret
#else
	ands	tail, wd, #3
	beq	.Lfw_word_loop
.Lfw_head:
	ldrb	data, [src], #1
	strb	data, [wd], #1
	subs	n, n, #1
	beq	.Lret
	ands	tail, wd, #3
	bne	.Lfw_head
.Lfw_word_loop:
	cmp	n, #4
	blo	.Lfw_byte_tail
	ldr	data, [src], #4
	str	data, [wd], #4
	subs	n, n, #4
	b	.Lfw_word_loop
#endif
.Lfw_byte_tail:
	cmp	n, #0
	beq	.Lret
	ldrb	data, [src], #1
	strb	data, [wd], #1
	subs	n, n, #1
	bne	.Lfw_byte_tail

.Lret:
	mov	result, dst		/* return original dst */
	pop	{r4, r5, pc}

	.size ARCH_LIBCFUN(memmove), . - ARCH_LIBCFUN(memmove)

#endif
