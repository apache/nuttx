/****************************************************************************
 * libs/libc/machine/arm/armv7-r/arch_memcmp.S
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
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define data1		r3
#define data2		r4
#define tmp1		r12

	.syntax unified
	.text
	.align 2
	.global ARCH_LIBCFUN(memcmp)
	.arm
	.type ARCH_LIBCFUN(memcmp), %function

ARCH_LIBCFUN(memcmp):
	.cfi_sections .debug_frame
	.cfi_startproc
	push	{r4, lr}
	.cfi_def_cfa_offset 8
	.cfi_offset lr, -4
	.cfi_offset r4, -8

	cmp	limit, #0
	beq	.Lret0

	/* Align src1 to a 4-byte boundary, comparing the misaligned head
	   byte-by-byte.  src2 may stay unaligned.  */
	ands	tmp1, src1, #3
	beq	.Laligned

.Lhead_loop:
	ldrb	data1, [src1], #1
	ldrb	data2, [src2], #1
	cmp	data1, data2
	bne	.Lret_diff_byte
	subs	limit, limit, #1
	beq	.Lret0
	ands	tmp1, src1, #3
	bne	.Lhead_loop

.Laligned:
.Lword_loop:
	cmp	limit, #4
	blo	.Lbyte_tail
	ldr	data1, [src1], #4
	ldr	data2, [src2], #4
	subs	limit, limit, #4
	cmp	data1, data2
	bne	.Ldiff_word
	b	.Lword_loop

.Lbyte_tail:
	cmp	limit, #0
	beq	.Lret0
.Lbyte_loop:
	ldrb	data1, [src1], #1
	ldrb	data2, [src2], #1
	cmp	data1, data2
	bne	.Lret_diff_byte
	subs	limit, limit, #1
	bne	.Lbyte_loop
	b	.Lret0

	.align 2
.Ldiff_word:
#ifndef __ARM_BIG_ENDIAN
	rev	data1, data1
	rev	data2, data2
#endif
	cmp	data1, data2
	blo	.Lret_minus1
	b	.Lret_plus1

.Lret_diff_byte:
	cmp	data1, data2
	blo	.Lret_minus1
	b	.Lret_plus1

.Lret_minus1:
	mvn	result, #0		/* -1 */
	pop	{r4, pc}

.Lret_plus1:
	mov	result, #1
	pop	{r4, pc}

.Lret0:
	mov	result, #0
	pop	{r4, pc}

	.cfi_endproc
	.size ARCH_LIBCFUN(memcmp), . - ARCH_LIBCFUN(memcmp)

#endif
