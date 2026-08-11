/****************************************************************************
 * libs/libc/machine/risc-v/arch_memchr.S
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

#ifdef LIBC_BUILD_MEMCHR

#include "asm.h"

.text
.global ARCH_LIBCFUN(memchr)
.type	ARCH_LIBCFUN(memchr), @function
.align	2

/************************************************************************************
 * Name: memchr
 *
 * void *memchr(const void *s, int c, size_t n)
 *
 * Broadcast + XOR + DETECTNULL word-at-a-time search.
 ************************************************************************************/
ARCH_LIBCFUN(memchr):
	.cfi_sections .debug_frame
	.cfi_startproc

	beqz	a2, .Lnotfound
	andi	a1, a1, 0xff
	mv	t6, a2			/* t6 = remaining count */

	/* Byte-by-byte head until aligned */

.Lhead:
	andi	t0, a0, SZREG-1
	beqz	t0, .Laligned
	beqz	t6, .Lnotfound
	lbu	t0, 0(a0)
	beq	t0, a1, .Lfound
	addi	a0, a0, 1
	addi	t6, t6, -1
	j	.Lhead

.Laligned:
	/* Broadcast c to full XLEN register */

	slli	t0, a1, 8
	or	t0, t0, a1
	slli	t1, t0, 16
	or	a6, t0, t1
#if SZREG == 8
	slli	t0, a6, 32
	or	a6, a6, t0
	lla	t2, .Lmc_mask01
	ld	t2, 0(t2)
	lla	t3, .Lmc_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

	/* Word loop */

.Lword_loop:
	li	t0, SZREG
	bltu	t6, t0, .Ltail		/* not enough bytes for full word */
	REG_L	t0, 0(a0)
	xor	t0, t0, a6		/* matching bytes become 0x00 */

	/* DETECTNULL on xored value */

	sub	t1, t0, t2
	not	t4, t0
	and	t1, t1, t4
	and	t1, t1, t3
	bnez	t1, .Ltail		/* found match in this word */
	addi	a0, a0, SZREG
	addi	t6, t6, -SZREG
	j	.Lword_loop

	/* Byte-by-byte tail */

.Ltail:
	beqz	t6, .Lnotfound
	lbu	t0, 0(a0)
	beq	t0, a1, .Lfound
	addi	a0, a0, 1
	addi	t6, t6, -1
	j	.Ltail

.Lfound:
	ret
.Lnotfound:
	li	a0, 0
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(memchr), .-ARCH_LIBCFUN(memchr)

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lmc_mask01:
	.dword	0x0101010101010101
.Lmc_mask80:
	.dword	0x8080808080808080
#endif

#endif
