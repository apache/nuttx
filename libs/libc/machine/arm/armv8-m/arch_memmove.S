/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_memmove.S
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
 * ARMv7-M / ARMv8-M (Thumb-2), unaligned word accesses supported.
 * When __ARM_FEATURE_MVE is defined the forward (non-overlapping) path
 * uses an MVE low-overhead byte loop; the backward (overlapping) path
 * uses a word-at-a-time copy from the end.
 */

#ifndef __ARM_FEATURE_BTI_DEFAULT
#  define __ARM_FEATURE_BTI_DEFAULT 0
#endif

#ifndef __ARM_FEATURE_PAC_DEFAULT
#  define __ARM_FEATURE_PAC_DEFAULT 0
#endif

#define dst		r0
#define src		r1
#define n		r2
#define result		r0		/* overlaps dst; dst is preserved */
#define wd		r3		/* moving dst pointer (forward) */
#define cnt		r4		/* word/byte count */
#define tail		r5		/* trailing byte count */
#define data		r6

	.thumb
	.syntax unified
	.global ARCH_LIBCFUN(memmove)
	.type ARCH_LIBCFUN(memmove), %function
	.thumb_func
ARCH_LIBCFUN(memmove):
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
	.cfi_offset r6, -8
	.cfi_offset r5, -12
	.cfi_offset r4, -16

	cbz	n, .Lret

	cmp	dst, src
	bcc	.Lforward		/* dst < src: forward copy */
	/* dst >= src: overlap only when dst < src + n */
	adds	r3, src, n
	cmp	dst, r3
	bcs	.Lforward		/* dst >= src + n: no overlap, forward */

	/* Backward copy (dst > src, overlapping).  */
	add	wd, dst, n		/* dst end */
	add	src, src, n		/* src end */
	lsrs	cnt, n, #2		/* word count */
	lsls	tail, cnt, #2
	subs	tail, n, tail		/* trailing bytes (n % 4) */
.Lbw_byte_tail:
	cbz	tail, .Lbw_word
	ldrb	data, [src, #-1]!
	strb	data, [wd, #-1]!
	subs	tail, tail, #1
	bne	.Lbw_byte_tail
.Lbw_word:
	cbz	cnt, .Lret
	ldr	data, [src, #-4]!
	str	data, [wd, #-4]!
	subs	cnt, cnt, #1
	bne	.Lbw_word
	b	.Lret

.Lforward:
#ifdef __ARM_FEATURE_MVE
	wlstp.8	lr, n, 2f
	mov	wd, dst			/* preserve dst for return value */
1:
	vldrb.8	q0, [src], #16
	vstrb.8	q0, [wd], #16
	letp	lr, 1b
2:
	b	.Lret
#else
	mov	wd, dst			/* preserve dst for return value */
	/* Align wd (dst) to 4 bytes; byte head.  */
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
.Lfw_byte_tail:
	cbz	n, .Lret
	ldrb	data, [src], #1
	strb	data, [wd], #1
	subs	n, n, #1
	bne	.Lfw_byte_tail
#endif

.Lret:
	mov	result, dst		/* return original dst */
	pop	{r4, r5, r6, lr}
#if __ARM_FEATURE_PAC_DEFAULT
	aut	ip, lr, sp
#endif
	bx	lr

	.cfi_endproc
	.size ARCH_LIBCFUN(memmove), . - ARCH_LIBCFUN(memmove)

#endif
