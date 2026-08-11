/****************************************************************************
 * libs/libc/machine/risc-v/arch_memcpy.S
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_MEMCPY

#include "asm.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#  define SHL_H srl
#  define SHL_L sll
#else
#  define SHL_H sll
#  define SHL_L srl
#endif

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

.global ARCH_LIBCFUN(memcpy)
.type	ARCH_LIBCFUN(memcpy), @function
.file	"arch_memcpy.S"

/************************************************************************************
 * Name: memcpy
 *
 * void *memcpy(void *dst, const void *src, size_t n)
 *
 * Optimized for RISC-V using XLEN-sized load/store with 16×SZREG unrolling.
 * Handles unaligned src via shift-merge technique.
 ************************************************************************************/

	.text

	.align	2
ARCH_LIBCFUN(memcpy):
	.cfi_sections .debug_frame
	.cfi_startproc

	move	t6, a0			/* Preserve return value (dst) */

	/* Small copy: size < 3*SZREG → byte-by-byte */

	li	a3, 3*SZREG
	bltu	a2, a3, .Lbyte_copy

	/* Align dst to SZREG boundary */

	andi	a3, a0, SZREG-1
	beqz	a3, .Ldst_aligned

	/* Copy head bytes to align dst */

	sub	a3, zero, a3
	addi	a3, a3, SZREG		/* a3 = bytes to copy = SZREG - misalignment */
	sub	a2, a2, a3		/* Update remaining count */
.Lalign_head:
	lbu	a4, 0(a1)
	addi	a1, a1, 1
	sb	a4, 0(t6)
	addi	t6, t6, 1
	addi	a3, a3, -1
	bnez	a3, .Lalign_head

.Ldst_aligned:
	/* Now dst (t6) is SZREG-aligned. Check if src is also aligned */

	andi	a3, a1, SZREG-1
	bnez	a3, .Lunaligned

	/* === Aligned path: both src and dst are SZREG-aligned === */

	/* Main loop: 16×SZREG per iteration */

	andi	a4, a2, ~(16*SZREG-1)
	beqz	a4, .Laligned_tail
	add	a3, a1, a4

	.align	3
.Laligned_loop:
	REG_L	a4,  0*SZREG(a1)
	REG_L	a5,  1*SZREG(a1)
	REG_L	a6,  2*SZREG(a1)
	REG_L	a7,  3*SZREG(a1)
	REG_L	t0,  4*SZREG(a1)
	REG_L	t1,  5*SZREG(a1)
	REG_L	t2,  6*SZREG(a1)
	REG_L	t3,  7*SZREG(a1)
	REG_L	t4,  8*SZREG(a1)
	REG_L	t5,  9*SZREG(a1)
	REG_S	a4,  0*SZREG(t6)
	REG_S	a5,  1*SZREG(t6)
	REG_S	a6,  2*SZREG(t6)
	REG_S	a7,  3*SZREG(t6)
	REG_S	t0,  4*SZREG(t6)
	REG_S	t1,  5*SZREG(t6)
	REG_S	t2,  6*SZREG(t6)
	REG_S	t3,  7*SZREG(t6)
	REG_S	t4,  8*SZREG(t6)
	REG_S	t5,  9*SZREG(t6)
	REG_L	a4, 10*SZREG(a1)
	REG_L	a5, 11*SZREG(a1)
	REG_L	a6, 12*SZREG(a1)
	REG_L	a7, 13*SZREG(a1)
	REG_L	t0, 14*SZREG(a1)
	REG_L	t1, 15*SZREG(a1)
	addi	a1, a1, 16*SZREG
	REG_S	a4, 10*SZREG(t6)
	REG_S	a5, 11*SZREG(t6)
	REG_S	a6, 12*SZREG(t6)
	REG_S	a7, 13*SZREG(t6)
	REG_S	t0, 14*SZREG(t6)
	REG_S	t1, 15*SZREG(t6)
	addi	t6, t6, 16*SZREG
	bltu	a1, a3, .Laligned_loop

	andi	a2, a2, 16*SZREG-1	/* Update remaining count */

.Laligned_tail:
	/* Single-word copy for remainder */

	andi	a4, a2, ~(SZREG-1)
	beqz	a4, .Lbyte_copy_update
	add	a3, a1, a4
.Lword_loop:
	REG_L	a4, 0(a1)
	addi	a1, a1, SZREG
	REG_S	a4, 0(t6)
	addi	t6, t6, SZREG
	bltu	a1, a3, .Lword_loop

	andi	a2, a2, SZREG-1		/* Update remaining count */

.Lbyte_copy_update:
	/* Fall through to byte copy with updated a2 */

.Lbyte_copy:
	/* Byte-by-byte copy for small/tail */

	beqz	a2, .Ldone
	add	a3, a1, a2
.Lbyte_loop:
	lbu	a4, 0(a1)
	addi	a1, a1, 1
	sb	a4, 0(t6)
	addi	t6, t6, 1
	bltu	a1, a3, .Lbyte_loop
.Ldone:
	ret

	/* === Unaligned path: dst aligned, src not aligned === */
	/* Uses shift-merge to combine two aligned loads into one store */

.Lunaligned:
	/* a3 = src misalignment (already computed above) */

	slli	a6, a3, 3		/* a6 = shift_h = misalign * 8 bits */
	sub	a7, zero, a6
	addi	a7, a7, SZREG*8		/* a7 = shift_l = XLEN - shift_h */

	/* Save src misalignment for later restore */

	mv	t4, a3			/* t4 = original misalignment bytes */

	/* Align src down to SZREG boundary */

	andi	a1, a1, ~(SZREG-1)

	/* Preload first aligned word from src */

	REG_L	a5, 0(a1)

	/* Calculate loop count: process 2×SZREG per iteration */

	andi	a4, a2, ~(2*SZREG-1)
	beqz	a4, .Lunaligned_tail
	add	a3, t6, a4		/* a3 = end address for dst */

	.align	3
.Lunaligned_loop:
	REG_L	a4, SZREG(a1)		/* Load next aligned word */
	SHL_H	t0, a5, a6		/* High part from previous word */
	SHL_L	t1, a4, a7		/* Low part from current word */
	or	t0, t0, t1		/* Combine */
	REG_S	t0, 0(t6)		/* Store to dst */

	REG_L	a5, 2*SZREG(a1)		/* Load next aligned word */
	SHL_H	t0, a4, a6		/* High part */
	SHL_L	t1, a5, a7		/* Low part */
	or	t0, t0, t1		/* Combine */
	REG_S	t0, SZREG(t6)		/* Store to dst */

	addi	a1, a1, 2*SZREG
	addi	t6, t6, 2*SZREG
	bltu	t6, a3, .Lunaligned_loop

	andi	a2, a2, 2*SZREG-1	/* Update remaining */

.Lunaligned_tail:
	/* Restore real src pointer: aligned_src + misalignment */

	add	a1, a1, t4

	j	.Lbyte_copy

	.cfi_endproc
	.size	ARCH_LIBCFUN(memcpy), .-ARCH_LIBCFUN(memcpy)

#endif /* LIBC_BUILD_MEMCPY */
