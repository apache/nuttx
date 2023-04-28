/****************************************************************************
 * libs/libc/machine/arm64/gnu/arch_memcmp.S
 *
 * Copyright (c) 2018 Linaro Limited
 *  All rights reserved.
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
 *    * Neither the name of the Linaro nor the names of its
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
 * Copyright (c) 2017 ARM Ltd
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

#include "libc.h"

#ifdef LIBC_BUILD_STRING

/* Assumptions:
 *
 * ARMv8-a, AArch64, unaligned accesses.
 */

#define L(l) .L ## l

/* Parameters and result.  */
#define src1		x0
#define src2		x1
#define limit		x2
#define result		w0

/* Internal variables.  */
#define data1		x3
#define data1w		w3
#define data1h		x4
#define data2		x5
#define data2w		w5
#define data2h		x6
#define tmp1		x7
#define tmp2		x8

        .macro def_fn f p2align=0
        .text
        .p2align \p2align
        .global \f
        .type \f, %function
\f:
        .endm

def_fn memcmp p2align=6
	subs	limit, limit, 8
	b.lo	L(less8)

	ldr	data1, [src1], 8
	ldr	data2, [src2], 8
	cmp	data1, data2
	b.ne	L(return)

	subs	limit, limit, 8
	b.gt	L(more16)

	ldr	data1, [src1, limit]
	ldr	data2, [src2, limit]
	b	L(return)

L(more16):
	ldr	data1, [src1], 8
	ldr	data2, [src2], 8
	cmp	data1, data2
	bne	L(return)

	/* Jump directly to comparing the last 16 bytes for 32 byte (or less)
	   strings.  */
	subs	limit, limit, 16
	b.ls	L(last_bytes)

	/* We overlap loads between 0-32 bytes at either side of SRC1 when we
	   try to align, so limit it only to strings larger than 128 bytes.  */
	cmp	limit, 96
	b.ls	L(loop16)

	/* Align src1 and adjust src2 with bytes not yet done.  */
	and	tmp1, src1, 15
	add	limit, limit, tmp1
	sub	src1, src1, tmp1
	sub	src2, src2, tmp1

	/* Loop performing 16 bytes per iteration using aligned src1.
	   Limit is pre-decremented by 16 and must be larger than zero.
	   Exit if <= 16 bytes left to do or if the data is not equal.  */
	.p2align 4
L(loop16):
	ldp	data1, data1h, [src1], 16
	ldp	data2, data2h, [src2], 16
	subs	limit, limit, 16
	ccmp	data1, data2, 0, hi
	ccmp	data1h, data2h, 0, eq
	b.eq	L(loop16)

	cmp	data1, data2
	bne	L(return)
	mov	data1, data1h
	mov	data2, data2h
	cmp	data1, data2
	bne	L(return)

	/* Compare last 1-16 bytes using unaligned access.  */
L(last_bytes):
	add	src1, src1, limit
	add	src2, src2, limit
	ldp	data1, data1h, [src1]
	ldp	data2, data2h, [src2]
	cmp     data1, data2
	bne	L(return)
	mov	data1, data1h
	mov	data2, data2h
	cmp	data1, data2

	/* Compare data bytes and set return value to 0, -1 or 1.  */
L(return):
#ifndef __AARCH64EB__
	rev	data1, data1
	rev	data2, data2
#endif
	cmp     data1, data2
L(ret_eq):
	cset	result, ne
	cneg	result, result, lo
	ret

	.p2align 4
	/* Compare up to 8 bytes.  Limit is [-8..-1].  */
L(less8):
	adds	limit, limit, 4
	b.lo	L(less4)
	ldr	data1w, [src1], 4
	ldr	data2w, [src2], 4
	cmp	data1w, data2w
	b.ne	L(return)
	sub	limit, limit, 4
L(less4):
	adds	limit, limit, 4
	beq	L(ret_eq)
L(byte_loop):
	ldrb	data1w, [src1], 1
	ldrb	data2w, [src2], 1
	subs	limit, limit, 1
	ccmp	data1w, data2w, 0, ne	/* NZCV = 0b0000.  */
	b.eq	L(byte_loop)
	sub	result, data1w, data2w
	ret

	.size	memcmp, . - memcmp

#endif
