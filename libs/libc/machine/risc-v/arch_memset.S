/****************************************************************************
 * libs/libc/machine/risc-v/arch_memset.S
 *
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 * SPDX-FileCopyrightText: 2017  SiFive Inc. All rights reserved.
 *
 * This copyrighted material is made available to anyone wishing to use,
 * modify, copy, or redistribute it subject to the terms and conditions
 * of the FreeBSD License.   This program is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY expressed or implied,
 * including the implied warranties of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  A copy of this license is available at
 * http://www.opensource.org/licenses.
 *
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_MEMSET

#include "asm.h"

.text
.global ARCH_LIBCFUN(memset)
.type	ARCH_LIBCFUN(memset), @function
.align	2

/************************************************************************************
 * Name: memset
 *
 * void *memset(void *s, int c, size_t n)
 *
 * Optimized with 32xSZREG unrolled loop and .option norvc.
 ************************************************************************************/
ARCH_LIBCFUN(memset):
	.cfi_sections .debug_frame
	.cfi_startproc

	mv	t0, a0

	li	t1, 2*SZREG
	bleu	a2, t1, .Lbyte_tail

	andi	a1, a1, 0xff
	slli	t1, a1, 8
	or	a1, a1, t1
	slli	t1, a1, 16
	or	a1, a1, t1
#ifdef CONFIG_ARCH_RV64
	slli	t1, a1, 32
	or	a1, a1, t1
#endif

	andi	t1, t0, SZREG-1
	beqz	t1, .Laligned
	sub	t2, zero, t1
	addi	t2, t2, SZREG
	sub	a2, a2, t2
.Lalign_head:
	sb	a1, 0(t0)
	addi	t0, t0, 1
	addi	t2, t2, -1
	bnez	t2, .Lalign_head

.Laligned:
	li	t3, 32*SZREG
	bltu	a2, t3, .Lword_tail

	.align	3
.Lblock_loop:
	.option push
	.option norvc
	REG_S	a1,  0*SZREG(t0)
	REG_S	a1,  1*SZREG(t0)
	REG_S	a1,  2*SZREG(t0)
	REG_S	a1,  3*SZREG(t0)
	REG_S	a1,  4*SZREG(t0)
	REG_S	a1,  5*SZREG(t0)
	REG_S	a1,  6*SZREG(t0)
	REG_S	a1,  7*SZREG(t0)
	REG_S	a1,  8*SZREG(t0)
	REG_S	a1,  9*SZREG(t0)
	REG_S	a1, 10*SZREG(t0)
	REG_S	a1, 11*SZREG(t0)
	REG_S	a1, 12*SZREG(t0)
	REG_S	a1, 13*SZREG(t0)
	REG_S	a1, 14*SZREG(t0)
	REG_S	a1, 15*SZREG(t0)
	REG_S	a1, 16*SZREG(t0)
	REG_S	a1, 17*SZREG(t0)
	REG_S	a1, 18*SZREG(t0)
	REG_S	a1, 19*SZREG(t0)
	REG_S	a1, 20*SZREG(t0)
	REG_S	a1, 21*SZREG(t0)
	REG_S	a1, 22*SZREG(t0)
	REG_S	a1, 23*SZREG(t0)
	REG_S	a1, 24*SZREG(t0)
	REG_S	a1, 25*SZREG(t0)
	REG_S	a1, 26*SZREG(t0)
	REG_S	a1, 27*SZREG(t0)
	REG_S	a1, 28*SZREG(t0)
	REG_S	a1, 29*SZREG(t0)
	REG_S	a1, 30*SZREG(t0)
	REG_S	a1, 31*SZREG(t0)
	.option pop
	addi	t0, t0, 32*SZREG
	sub	a2, a2, t3
	bgeu	a2, t3, .Lblock_loop

.Lword_tail:
	andi	t1, a2, ~(SZREG-1)
	beqz	t1, .Lbyte_tail
	add	t2, t0, t1
.Lword_loop:
	REG_S	a1, 0(t0)
	addi	t0, t0, SZREG
	bltu	t0, t2, .Lword_loop
	andi	a2, a2, SZREG-1

.Lbyte_tail:
	beqz	a2, .Ldone
.Lbyte_fill:
	sb	a1, 0(t0)
	addi	t0, t0, 1
	addi	a2, a2, -1
	bnez	a2, .Lbyte_fill
.Ldone:
	ret
	.cfi_endproc
	.size	ARCH_LIBCFUN(memset), .-ARCH_LIBCFUN(memset)

#endif
