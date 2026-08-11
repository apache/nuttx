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
 * Word-at-a-time comparison with byte-level mismatch detection.
 ************************************************************************************/
ARCH_LIBCFUN(memcmp):
	.cfi_sections .debug_frame
	.cfi_startproc

	beqz	a2, .Lequal

	or	t0, a0, a1
	andi	t0, t0, SZREG-1
	bnez	t0, .Lbyte_cmp

	li	t0, SZREG
	bltu	a2, t0, .Lbyte_cmp

.Lword_loop:
	REG_L	t1, 0(a0)
	REG_L	t2, 0(a1)
	bne	t1, t2, .Lfind_diff
	addi	a0, a0, SZREG
	addi	a1, a1, SZREG
	sub	a2, a2, t0
	bgeu	a2, t0, .Lword_loop
	beqz	a2, .Lequal

.Lbyte_cmp:
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
