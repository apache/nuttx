/****************************************************************************
 * libs/libc/machine/risc-v/arch_strcat.S
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

#ifdef LIBC_BUILD_STRCAT

#include "asm.h"

.text
.global ARCH_LIBCFUN(strcat)
.type	ARCH_LIBCFUN(strcat), @function
.align	2

/************************************************************************************
 * Name: strcat
 *
 * char *strcat(char *dst, const char *src)
 *
 * Word-at-a-time strlen + word-at-a-time copy.
 ************************************************************************************/
ARCH_LIBCFUN(strcat):
	.cfi_sections .debug_frame
	.cfi_startproc

	mv	t6, a0			/* save original dst for return */
	mv	a7, a1			/* save src */

	/* --- Phase 1: Find end of dst (word-at-a-time strlen) --- */

	andi	t0, a0, SZREG-1
	beqz	t0, .Lslen_aligned

.Lslen_head:
	lbu	t0, 0(a0)
	beqz	t0, .Lcopy_start
	addi	a0, a0, 1
	andi	t0, a0, SZREG-1
	bnez	t0, .Lslen_head

.Lslen_aligned:
#if SZREG == 8
	lla	t2, .Lct_mask01
	ld	t2, 0(t2)
	lla	t3, .Lct_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

.Lslen_word:
	REG_L	t0, 0(a0)
	sub	t1, t0, t2
	not	t4, t0
	and	t1, t1, t4
	and	t1, t1, t3
	bnez	t1, .Lslen_find
	addi	a0, a0, SZREG
	j	.Lslen_word

.Lslen_find:
	lbu	t0, 0(a0)
	beqz	t0, .Lcopy_start
	addi	a0, a0, 1
	j	.Lslen_find

.Lcopy_start:
	/* a0 = dst end (null position), a7 = src */
	/* --- Phase 2: Copy src to dst end (word-at-a-time) --- */

	mv	a1, a7

	/* Check alignment consistency */

	xor	t0, a0, a1
	andi	t0, t0, SZREG-1
	bnez	t0, .Lcopy_byte		/* misaligned: byte copy */

	/* Align */

	andi	t0, a0, SZREG-1
	beqz	t0, .Lcopy_word

.Lcopy_align:
	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Lcat_done
	addi	a0, a0, 1
	addi	a1, a1, 1
	andi	t0, a0, SZREG-1
	bnez	t0, .Lcopy_align

.Lcopy_word:
	/* Word-at-a-time copy with null detection */
	/* t2, t3 still hold masks from strlen phase */

	REG_L	t0, 0(a1)
	sub	t1, t0, t2
	not	t4, t0
	and	t1, t1, t4
	and	t1, t1, t3
	bnez	t1, .Lcopy_byte		/* has null: finish byte-by-byte */
	REG_S	t0, 0(a0)
	addi	a0, a0, SZREG
	addi	a1, a1, SZREG
	j	.Lcopy_word

.Lcopy_byte:
	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Lcat_done
	addi	a0, a0, 1
	addi	a1, a1, 1
	j	.Lcopy_byte

.Lcat_done:
	mv	a0, t6
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(strcat), .-ARCH_LIBCFUN(strcat)

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lct_mask01:
	.dword	0x0101010101010101
.Lct_mask80:
	.dword	0x8080808080808080
#endif

#endif
