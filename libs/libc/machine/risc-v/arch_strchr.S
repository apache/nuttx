/****************************************************************************
 * libs/libc/machine/risc-v/arch_strchr.S
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

#ifdef LIBC_BUILD_STRCHR

#include "asm.h"

#ifndef STRCHR
#define STRCHR ARCH_LIBCFUN(strchr)
#endif

.text
.global STRCHR
.type	STRCHR, @function
.align	2

/************************************************************************************
 * Name: strchr
 *
 * char *strchr(const char *s, int c)
 *
 * Broadcast + DETECTCHAR + DETECTNULL. Supports strchrnul via macro.
 ************************************************************************************/
STRCHR:
	.cfi_sections .debug_frame
	.cfi_startproc

	andi	a1, a1, 0xff

	/* Byte-by-byte until aligned */

.Lalign_loop:
	andi	t0, a0, SZREG-1
	beqz	t0, .Laligned
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	j	.Lalign_loop

.Laligned:
	/* Broadcast c to all bytes */

	slli	t0, a1, 8
	or	t0, t0, a1
	slli	t1, t0, 16
	or	a6, t0, t1		/* a6 = broadcast_c (32-bit) */
#if SZREG == 8
	slli	t0, a6, 32
	or	a6, a6, t0		/* a6 = broadcast_c (64-bit) */
#endif

	/* Load masks */
#if SZREG == 8
	lla	t2, .Lchr_mask01
	ld	t2, 0(t2)
	lla	t3, .Lchr_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

.Lword_loop:
	REG_L	t0, 0(a0)

	/* DETECTNULL(word) - check for string end */

	sub	t1, t0, t2
	not	t4, t0
	and	t1, t1, t4
	and	t1, t1, t3

	/* DETECTCHAR(word, c) = DETECTNULL(word ^ broadcast_c) */

	xor	t4, t0, a6
	sub	t5, t4, t2
	not	a5, t4
	and	t5, t5, a5
	and	t5, t5, t3

	/* If either null or char found */

	or	a5, t1, t5
	bnez	a5, .Lbyte_scan
	addi	a0, a0, SZREG
	j	.Lword_loop

.Lbyte_scan:
	/* Scan byte-by-byte to determine if char or null came first */

	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
#if SZREG == 8
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
	lbu	t1, 0(a0)
	beq	t1, a1, .Lfound
	beqz	t1, .Lnot_found
	addi	a0, a0, 1
#endif
	j	.Lword_loop

.Lfound:
	ret
.Lnot_found:
#ifdef USE_AS_STRCHRNUL
	ret
#else
	li	a0, 0
	ret
#endif

	.cfi_endproc
	.size	STRCHR, .-STRCHR

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lchr_mask01:
	.dword	0x0101010101010101
.Lchr_mask80:
	.dword	0x8080808080808080
#endif

#endif
