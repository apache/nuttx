/****************************************************************************
 * libs/libc/machine/risc-v/arch_strrchr.S
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

#ifdef LIBC_BUILD_STRRCHR

#include "asm.h"

.text
.global ARCH_LIBCFUN(strrchr)
.type	ARCH_LIBCFUN(strrchr), @function
.align	2

/************************************************************************************
 * Name: strrchr
 *
 * char *strrchr(const char *s, int c)
 *
 * Word-at-a-time with last-match tracking.
 ************************************************************************************/
ARCH_LIBCFUN(strrchr):
	.cfi_sections .debug_frame
	.cfi_startproc

	andi	a1, a1, 0xff
	li	a7, 0			/* a7 = last_match = NULL */

	/* Byte-by-byte head until aligned */

.Lhead:
	andi	t0, a0, SZREG-1
	beqz	t0, .Laligned
	lbu	t0, 0(a0)
	bne	t0, a1, .Lhead_skip
	mv	a7, a0
.Lhead_skip:
	beqz	t0, .Ldone
	addi	a0, a0, 1
	j	.Lhead

.Laligned:
	/* Broadcast c and load masks */

	slli	t0, a1, 8
	or	t0, t0, a1
	slli	t1, t0, 16
	or	a6, t0, t1		/* a6 = broadcast_c (32-bit) */
#if SZREG == 8
	slli	t0, a6, 32
	or	a6, a6, t0
	lla	t2, .Lrc_mask01
	ld	t2, 0(t2)
	lla	t3, .Lrc_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

	/* Word loop: check for null and target char simultaneously */

.Lword_loop:
	REG_L	t0, 0(a0)

	/* DETECTNULL(word) - check string end */

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

	/* If null found, do final byte scan of this word */

	bnez	t1, .Lfinal_scan

	/* If target char found in this word, scan for last match */

	bnez	t5, .Lupdate_match

	addi	a0, a0, SZREG
	j	.Lword_loop

.Lupdate_match:
	/* Target char found, byte-scan this word to update last_match */
	/* Then continue to next word */

	li	a5, SZREG
.Lupdate_loop:
	lbu	t0, 0(a0)
	beqz	t0, .Ldone		/* shouldn't happen here, but safety */
	beq	t0, a1, .Lupdate_hit
.Lupdate_next:
	addi	a0, a0, 1
	addi	a5, a5, -1
	bnez	a5, .Lupdate_loop
	j	.Lword_loop		/* back to word loop (a0 now at next word) */

.Lupdate_hit:
	mv	a7, a0
	j	.Lupdate_next

.Lfinal_scan:
	/* This word contains null. Byte-scan to find last match before null. */

.Lfinal_loop:
	lbu	t0, 0(a0)
	beqz	t0, .Ldone
	beq	t0, a1, .Lfinal_hit
	addi	a0, a0, 1
	j	.Lfinal_loop

.Lfinal_hit:
	mv	a7, a0
	addi	a0, a0, 1
	j	.Lfinal_loop

.Ldone:
	/* Special: if c == 0, return pointer to the null terminator */
	bnez	a1, .Lret
	mv	a7, a0

.Lret:
	mv	a0, a7
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(strrchr), .-ARCH_LIBCFUN(strrchr)

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lrc_mask01:
	.dword	0x0101010101010101
.Lrc_mask80:
	.dword	0x8080808080808080
#endif

#endif
