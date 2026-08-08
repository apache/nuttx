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

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

.global ARCH_LIBCFUN(memcpy)
.type	ARCH_LIBCFUN(memcpy), @function
.file	"arch_memcpy.S"

/* Copy a register at a time.  On RV64 that is eight bytes rather than
 * four, which halves the loads and the stores for the same work; RV32
 * gets exactly the code it had.
 */

#if __riscv_xlen == 64
#  define LREG      ld
#  define SREG      sd
#  define REGBYTES  8
#  define XLENBITS  64
#else
#  define LREG      lw
#  define SREG      sw
#  define REGBYTES  4
#  define XLENBITS  32
#endif

/* One turn of the unrolled loop moves sixteen registers.  Below this
 * many bytes the alignment work costs more than it saves.
 */

#define BLKBYTES    (16 * REGBYTES)
#define SMALLSZ     (4 * REGBYTES)

/****************************************************************************
 * Name: memcpy
 ****************************************************************************/

	.text

ARCH_LIBCFUN(memcpy):
	.cfi_sections .debug_frame
	.cfi_startproc
	mv		t6, a0		/* Preserve the return value */

	/* Below the threshold, aligning costs more than it saves */

	li		a3, SMALLSZ
	bltu		a2, a3, .Lbytes

	/* Align the destination first.  Stores are the side worth
	 * aligning: a misaligned load can be fixed up by shifting what
	 * we already hold, a misaligned store cannot.
	 */

	neg		a3, t6
	andi		a3, a3, REGBYTES-1
	beqz		a3, .Ldstaligned
	sub		a2, a2, a3
1:
	lb		a4, 0(a1)
	addi		a1, a1, 1
	sb		a4, 0(t6)
	addi		t6, t6, 1
	addi		a3, a3, -1
	bnez		a3, 1b

.Ldstaligned:
	/* If the source came out aligned too, copy whole registers */

	andi		a3, a1, REGBYTES-1
	bnez		a3, .Lshifted

	andi		a4, a2, ~(BLKBYTES-1)
	beqz		a4, .Lwords
	add		a3, a1, a4
2:
	LREG		a4,           0(a1)
	LREG		a5,   1*REGBYTES(a1)
	LREG		a6,   2*REGBYTES(a1)
	LREG		a7,   3*REGBYTES(a1)
	SREG		a4,           0(t6)
	SREG		a5,   1*REGBYTES(t6)
	SREG		a6,   2*REGBYTES(t6)
	SREG		a7,   3*REGBYTES(t6)
	LREG		a4,   4*REGBYTES(a1)
	LREG		a5,   5*REGBYTES(a1)
	LREG		a6,   6*REGBYTES(a1)
	LREG		a7,   7*REGBYTES(a1)
	SREG		a4,   4*REGBYTES(t6)
	SREG		a5,   5*REGBYTES(t6)
	SREG		a6,   6*REGBYTES(t6)
	SREG		a7,   7*REGBYTES(t6)
	LREG		a4,   8*REGBYTES(a1)
	LREG		a5,   9*REGBYTES(a1)
	LREG		a6,  10*REGBYTES(a1)
	LREG		a7,  11*REGBYTES(a1)
	SREG		a4,   8*REGBYTES(t6)
	SREG		a5,   9*REGBYTES(t6)
	SREG		a6,  10*REGBYTES(t6)
	SREG		a7,  11*REGBYTES(t6)
	LREG		a4,  12*REGBYTES(a1)
	LREG		a5,  13*REGBYTES(a1)
	LREG		a6,  14*REGBYTES(a1)
	LREG		a7,  15*REGBYTES(a1)
	SREG		a4,  12*REGBYTES(t6)
	SREG		a5,  13*REGBYTES(t6)
	SREG		a6,  14*REGBYTES(t6)
	SREG		a7,  15*REGBYTES(t6)
	addi		a1, a1, BLKBYTES
	addi		t6, t6, BLKBYTES
	bltu		a1, a3, 2b
	andi		a2, a2, BLKBYTES-1

.Lwords:
	/* Whatever whole registers are left over */

	andi		a4, a2, ~(REGBYTES-1)
	beqz		a4, .Lbytes
	add		a3, a1, a4
	andi		a2, a2, REGBYTES-1
3:
	LREG		a5, 0(a1)
	addi		a1, a1, REGBYTES
	SREG		a5, 0(t6)
	addi		t6, t6, REGBYTES
	bltu		a1, a3, 3b
	j		.Lbytes

.Lshifted:
	/* The destination is aligned and the source is not, so no load
	 * and no store need ever be misaligned: read the two aligned
	 * words straddling each output word and shift them together.
	 *
	 * a3 holds how far past a boundary the source sits.  It is never
	 * zero here, so neither shift is ever by the register width,
	 * which would be undefined.
	 */

	slli		a4, a3, 3	/* Bits to shift down          */
	li		a5, XLENBITS
	sub		a5, a5, a4	/* Bits to shift the next up   */
	sub		a1, a1, a3	/* Round the source down       */
	li		a7, REGBYTES
	LREG		a6, 0(a1)	/* The word we are part way in */
	bltu		a2, a7, .Lshifttail
4:
	LREG		t0, REGBYTES(a1)
	srl		t1, a6, a4
	sll		t2, t0, a5
	or		t1, t1, t2
	SREG		t1, 0(t6)
	mv		a6, t0
	addi		a1, a1, REGBYTES
	addi		t6, t6, REGBYTES
	sub		a2, a2, a7
	bgeu		a2, a7, 4b

.Lshifttail:
	add		a1, a1, a3	/* Undo the rounding down      */

.Lbytes:
	/* The leftovers, and the whole copy when it was too small to be
	 * worth aligning.
	 */

	beqz		a2, .Ldone
	add		a3, a1, a2
5:
	lb		a4, 0(a1)
	addi		a1, a1, 1
	sb		a4, 0(t6)
	addi		t6, t6, 1
	bltu		a1, a3, 5b

.Ldone:
	ret

	.cfi_endproc
#endif
