/****************************************************************************
 * libs/libc/machine/arm/armv8-r/arch_memcmp.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2018 Linaro Limited, 2017 ARM Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of Linaro nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_MEMCMP

/* Assumptions:
 *
 * ARMv7-A / ARMv7-R (ARM state), unaligned word accesses supported.
 */

#define src1		r0
#define src2		r1
#define limit		r2
#define result		r0
#define data1a		r4
#define data1b		r5
#define data2a		r6
#define data2b		r7
#define tmp1		r3

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(memcmp)
	.arm
	.type ARCH_LIBCFUN(memcmp), %function

ARCH_LIBCFUN(memcmp):
	.cfi_sections .debug_frame
	.cfi_startproc
	push	{r4, r5, r6, r7, lr}
	.cfi_def_cfa_offset 20
	.cfi_offset r4, -20
	.cfi_offset r5, -16
	.cfi_offset r6, -12
	.cfi_offset r7, -8
	.cfi_offset lr, -4

	cmp	limit, #0
	beq	.Lret0

	/* Align src1 to a 4-byte boundary, comparing the misaligned head
	   byte-by-byte.  src2 may stay unaligned.  */
	ands	tmp1, src1, #3
	beq	.Laligned

.Lhead_loop:
	ldrb	data1a, [src1], #1
	ldrb	data2a, [src2], #1
	cmp	data1a, data2a
	bne	.Lret_diff_byte
	subs	limit, limit, #1
	beq	.Lret0
	ands	tmp1, src1, #3
	bne	.Lhead_loop

.Laligned:
	/* ldrd needs a word-aligned address, so only take the 8-byte path
	   when src2 is also word-aligned; otherwise fall back to bytes.  */
	tst	src2, #3
	bne	.Lbyte_tail
.Lword_loop:
	cmp	limit, #8
	blo	.Lbyte_tail
	/* Compare 8 bytes per iteration via ldrd, halving the loop trip
	   count versus a 4-byte word loop.  */
	ldrd	data1a, data1b, [src1], #8
	ldrd	data2a, data2b, [src2], #8
	subs	limit, limit, #8
	cmp	data1a, data2a
	bne	.Ldiff_word_a
	cmp	data1b, data2b
	beq	.Lword_loop

	/* Difference is in the second word (bytes 4-7 of this 8-byte block).  */
.Ldiff_word_b:
#ifndef __ARM_BIG_ENDIAN
	rev	data1b, data1b
	rev	data2b, data2b
#endif
	cmp	data1b, data2b
	blo	.Lret_minus1
	b	.Lret_plus1

	/* Difference is in the first word (bytes 0-3 of this 8-byte block).  */
.Ldiff_word_a:
#ifndef __ARM_BIG_ENDIAN
	rev	data1a, data1a
	rev	data2a, data2a
#endif
	cmp	data1a, data2a
	blo	.Lret_minus1
	b	.Lret_plus1

.Lbyte_tail:
	cmp	limit, #0
	beq	.Lret0
.Lbyte_loop:
	ldrb	data1a, [src1], #1
	ldrb	data2a, [src2], #1
	cmp	data1a, data2a
	bne	.Lret_diff_byte
	subs	limit, limit, #1
	bne	.Lbyte_loop
	b	.Lret0

.Lret_diff_byte:
	cmp	data1a, data2a
	blo	.Lret_minus1
	b	.Lret_plus1

.Lret_minus1:
	mvn	result, #0		/* -1 */
	pop	{r4, r5, r6, r7, pc}

.Lret_plus1:
	mov	result, #1
	pop	{r4, r5, r6, r7, pc}

.Lret0:
	mov	result, #0
	pop	{r4, r5, r6, r7, pc}

	.cfi_endproc
	.size ARCH_LIBCFUN(memcmp), . - ARCH_LIBCFUN(memcmp)

#endif
