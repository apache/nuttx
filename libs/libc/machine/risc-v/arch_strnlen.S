/****************************************************************************
 * libs/libc/machine/risc-v/arch_strnlen.S
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

#ifdef LIBC_BUILD_STRNLEN

#include "asm.h"

.text
.global ARCH_LIBCFUN(strnlen)
.type	ARCH_LIBCFUN(strnlen), @function
.align	2

/************************************************************************************
 * Name: strnlen
 *
 * size_t strnlen(const char *s, size_t maxlen)
 *
 * Word-at-a-time DETECTNULL with maxlen bound.
 ************************************************************************************/
ARCH_LIBCFUN(strnlen):
	.cfi_sections .debug_frame
	.cfi_startproc

	beqz	a1, .Lret_zero
	mv	t0, a0			/* t0 = current ptr */
	mv	t6, a1			/* t6 = remaining count */

	/* Align head */

.Lhead:
	andi	t1, t0, SZREG-1
	beqz	t1, .Laligned
	beqz	t6, .Lret_maxlen
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	addi	t6, t6, -1
	j	.Lhead

.Laligned:
#if SZREG == 8
	lla	t2, .Lsnl_mask01
	ld	t2, 0(t2)
	lla	t3, .Lsnl_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

	/* Word loop with count check */

.Lword_loop:
	li	t4, SZREG
	bltu	t6, t4, .Ltail		/* not enough bytes for a word */
	REG_L	t1, 0(t0)

	/* DETECTNULL */

	sub	t4, t1, t2
	not	t5, t1
	and	t4, t4, t5
	and	t4, t4, t3
	bnez	t4, .Lfind_byte
	addi	t0, t0, SZREG
	addi	t6, t6, -SZREG
	j	.Lword_loop

.Lfind_byte:
	/* Null in this word, find exact position */

.Ltail:
	beqz	t6, .Lret_maxlen
	lbu	t1, 0(t0)
	beqz	t1, .Ldone
	addi	t0, t0, 1
	addi	t6, t6, -1
	j	.Ltail

.Ldone:
	sub	a0, t0, a0
	ret
.Lret_maxlen:
	mv	a0, a1
	ret
.Lret_zero:
	li	a0, 0
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(strnlen), .-ARCH_LIBCFUN(strnlen)

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lsnl_mask01:
	.dword	0x0101010101010101
.Lsnl_mask80:
	.dword	0x8080808080808080
#endif

#endif
