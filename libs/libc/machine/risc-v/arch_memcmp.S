/****************************************************************************
 * libs/libc/machine/risc-v/arch_memcmp.S
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

#ifdef LIBC_BUILD_MEMCMP

#include "asm.h"

.text
.global ARCH_LIBCFUN(memcmp)
.type	ARCH_LIBCFUN(memcmp), @function
.align	2

/************************************************************************************
 * Name: memcmp
 *
 * int memcmp(const void *s1, const void *s2, size_t n)
 *
 * Word-at-a-time comparison with 4-word XOR|OR folding to reduce branch
 * overhead in the main loop.
 ************************************************************************************/
ARCH_LIBCFUN(memcmp):
	.cfi_sections .debug_frame
	.cfi_startproc

	beqz	a2, .Lequal

	/* The loops below load from both pointers at a boundary, which asks
	 * that the two agree about where a boundary falls, not that either
	 * is already on one.  Test the difference of the pointers, not their
	 * union.
	 */

	xor	t0, a0, a1
	andi	t0, t0, SZREG-1
	bnez	t0, .Lbyte_cmp

	/* Same offset: walk both up to the boundary a byte at a time,
	 * stopping early on a difference.
	 */

.Lalign_head:
	andi	t0, a0, SZREG-1
	beqz	t0, .Laligned
	lbu	t1, 0(a0)
	lbu	t2, 0(a1)
	bne	t1, t2, .Ldiff
	addi	a0, a0, 1
	addi	a1, a1, 1
	addi	a2, a2, -1
	beqz	a2, .Lequal
	j	.Lalign_head

.Laligned:
	li	t0, SZREG
	bltu	a2, t0, .Lbyte_cmp

	/* Need at least 4*SZREG for unrolled loop */

	li	a3, 4*SZREG
	bltu	a2, a3, .Lword_loop

	/* 4-word unrolled loop: XOR each pair, OR the results.
	 * One branch per 4 words.  On difference, rewind and let
	 * the single-word loop find the exact mismatch word.
	 */

.Lword4_loop:
	REG_L	t1, 0*SZREG(a0)
	REG_L	t2, 0*SZREG(a1)
	REG_L	t3, 1*SZREG(a0)
	REG_L	t4, 1*SZREG(a1)
	xor	t1, t1, t2
	xor	t3, t3, t4
	or	t1, t1, t3

	REG_L	t3, 2*SZREG(a0)
	REG_L	t4, 2*SZREG(a1)
	REG_L	t5, 3*SZREG(a0)
	REG_L	t6, 3*SZREG(a1)
	xor	t3, t3, t4
	xor	t5, t5, t6
	or	t3, t3, t5
	or	t1, t1, t3

	bnez	t1, .Lword_loop
	addi	a0, a0, 4*SZREG
	addi	a1, a1, 4*SZREG
	sub	a2, a2, a3
	bgeu	a2, a3, .Lword4_loop

	/* Fall through to single-word loop for remainder */

.Lword_loop:
	bltu	a2, t0, .Lbyte_cmp
	REG_L	t1, 0(a0)
	REG_L	t2, 0(a1)
	bne	t1, t2, .Lfind_diff
	addi	a0, a0, SZREG
	addi	a1, a1, SZREG
	sub	a2, a2, t0
	bgeu	a2, t0, .Lword_loop
	beqz	a2, .Lequal

.Lbyte_cmp:
	beqz	a2, .Lequal
	lbu	t1, 0(a0)
	lbu	t2, 0(a1)
	bne	t1, t2, .Ldiff
	addi	a0, a0, 1
	addi	a1, a1, 1
	addi	a2, a2, -1
	bnez	a2, .Lbyte_cmp

.Lequal:
	li	a0, 0
	ret

.Ldiff:
	sub	a0, t1, t2
	ret

.Lfind_diff:
	/* Little-endian: first differing byte is at LSB side */

	andi	t3, t1, 0xff
	andi	t4, t2, 0xff
	bne	t3, t4, .Lfound
	srli	t1, t1, 8
	srli	t2, t2, 8
	j	.Lfind_diff
.Lfound:
	sub	a0, t3, t4
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(memcmp), .-ARCH_LIBCFUN(memcmp)

#endif
