/****************************************************************************
 * libs/libc/machine/risc-v/arch_strlcpy.S
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

#ifdef LIBC_BUILD_STRLCPY

#include "asm.h"

.text
.global ARCH_LIBCFUN(strlcpy)
.type	ARCH_LIBCFUN(strlcpy), @function
.align	2

/************************************************************************************
 * Name: strlcpy
 *
 * size_t strlcpy(char *dst, const char *src, size_t size)
 *
 * Copy at most size-1 bytes from src to dst, null-terminate (if size > 0),
 * and return strlen(src).  Word-at-a-time DETECTNULL for both the copy
 * phase and the strlen tail when truncated.
 ************************************************************************************/

/* DETECTNULL constants loaded from .srodata */

	.section .srodata, "a"
	.align	SZREG
.Lmask_01:
#if SZREG == 8
	.dword	0x0101010101010101
#else
	.word	0x01010101
#endif
.Lmask_80:
#if SZREG == 8
	.dword	0x8080808080808080
#else
	.word	0x80808080
#endif

	.text

ARCH_LIBCFUN(strlcpy):
	.cfi_sections .debug_frame
	.cfi_startproc

	mv	t6, a1			/* save src start for return value */

	/* size == 0: skip copy, just measure src */

	beqz	a2, .Lmeasure

	addi	a2, a2, -1		/* reserve space for null terminator */

	/* The word loop below aligns src and then stores a register at a
	 * time to dst, so it is safe only where the two pointers agree about
	 * where a boundary falls.  Where they do not, no single boundary
	 * serves both and the copy goes a byte at a time.
	 */

	xor	t0, a0, a1
	andi	t0, t0, SZREG-1
	bnez	t0, .Lcopy_tail

	/* Bytewise copy head: align src, and with it dst */

.Lcopy_head:
	beqz	a2, .Ltruncated
	andi	t0, a1, SZREG-1
	beqz	t0, .Lcopy_word_setup
	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Ldone
	addi	a0, a0, 1
	addi	a1, a1, 1
	addi	a2, a2, -1
	j	.Lcopy_head

.Lcopy_word_setup:
	/* Load DETECTNULL masks */

	lla	t2, .Lmask_01
	lla	t3, .Lmask_80
	REG_L	t2, 0(t2)		/* t2 = 0x0101...01 */
	REG_L	t3, 0(t3)		/* t3 = 0x8080...80 */

	/* Word copy loop: need at least SZREG bytes remaining in size */

	li	t4, SZREG

.Lcopy_word:
	bltu	a2, t4, .Lcopy_tail
	REG_L	t0, 0(a1)

	/* DETECTNULL: (word - 0x0101..01) & ~word & 0x8080..80 */

	sub	t1, t0, t2
	not	t5, t0
	and	t1, t1, t5
	and	t1, t1, t3
	bnez	t1, .Lcopy_last_word

	/* No null in this word, store it */

	REG_S	t0, 0(a0)
	addi	a0, a0, SZREG
	addi	a1, a1, SZREG
	sub	a2, a2, t4
	j	.Lcopy_word

.Lcopy_last_word:
	/* Null found within word: copy bytes until null */

	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Ldone
	addi	a0, a0, 1
	addi	a1, a1, 1
	j	.Lcopy_last_word

.Lcopy_tail:
	/* Less than SZREG bytes of size left: bytewise copy */

	beqz	a2, .Ltruncated
	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Ldone
	addi	a0, a0, 1
	addi	a1, a1, 1
	addi	a2, a2, -1
	j	.Lcopy_tail

.Ltruncated:
	/* Null-terminate dst */

	sb	zero, 0(a0)

	/* Measure remaining src length with word-at-a-time */

.Lmeasure:
	/* Bytewise to align src */

	andi	t0, a1, SZREG-1
	beqz	t0, .Lmeasure_word_setup

.Lmeasure_head:
	lbu	t0, 0(a1)
	beqz	t0, .Lreturn
	addi	a1, a1, 1
	andi	t0, a1, SZREG-1
	bnez	t0, .Lmeasure_head

.Lmeasure_word_setup:
	/* Load masks if not already loaded (from truncated path) */

	lla	t2, .Lmask_01
	lla	t3, .Lmask_80
	REG_L	t2, 0(t2)
	REG_L	t3, 0(t3)

.Lmeasure_word:
	REG_L	t0, 0(a1)
	sub	t1, t0, t2
	not	t5, t0
	and	t1, t1, t5
	and	t1, t1, t3
	bnez	t1, .Lmeasure_tail
	addi	a1, a1, SZREG
	j	.Lmeasure_word

.Lmeasure_tail:
	/* Find exact null position */

	lbu	t0, 0(a1)
	beqz	t0, .Lreturn
	addi	a1, a1, 1
	j	.Lmeasure_tail

.Ldone:
	/* Normal completion: null found during copy */

.Lreturn:
	/* Return strlen(src) = current src pos - original start */

	sub	a0, a1, t6
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(strlcpy), .-ARCH_LIBCFUN(strlcpy)

#endif
