/****************************************************************************
 * libs/libc/machine/risc-v/gnu/arch_memcpy.S
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

#ifdef LIBC_BUILD_STRING

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.globl		memcpy
	.file		"arch_memcpy.S"

/************************************************************************************
 * Name: memcpy
 ************************************************************************************/

	.text

memcpy:
	move		t6, a0  /* Preserve return value */

	/* Defer to byte-oriented copy for small sizes */
	sltiu		a3, a2, 128
	bnez		a3, 4f
	/* Use word-oriented copy only if low-order bits match */
	andi		a3, t6, 3
	andi		a4, a1, 3
	bne		a3, a4, 4f

	beqz		a3, 2f  /* Skip if already aligned */
	/*
	 * Round to nearest double word-aligned address
	 * greater than or equal to start address
	 */
	andi		a3, a1, ~3
	addi		a3, a3, 4
	/* Handle initial misalignment */
	sub		a4, a3, a1
1:
	lb		a5, 0(a1)
	addi		a1, a1, 1
	sb		a5, 0(t6)
	addi		t6, t6, 1
	bltu		a1, a3, 1b
	sub		a2, a2, a4  /* Update count */

2:
	andi		a4, a2, ~63
	beqz		a4, 4f
	add		a3, a1, a4
3:
	lw		a4,   0(a1)
	lw		a5,   4(a1)
	lw		a6, 2*4(a1)
	lw		a7, 3*4(a1)
	lw		t0, 4*4(a1)
	lw		t1, 5*4(a1)
	lw		t2, 6*4(a1)
	lw		t3, 7*4(a1)
	lw		t4, 8*4(a1)
	lw		t5, 9*4(a1)
	sw		a4,   0(t6)
	sw		a5,   4(t6)
	sw		a6, 2*4(t6)
	sw		a7, 3*4(t6)
	sw		t0, 4*4(t6)
	sw		t1, 5*4(t6)
	sw		t2, 6*4(t6)
	sw		t3, 7*4(t6)
	sw		t4, 8*4(t6)
	sw		t5, 9*4(t6)
	lw		a4, 10*4(a1)
	lw		a5, 11*4(a1)
	lw		a6, 12*4(a1)
	lw		a7, 13*4(a1)
	lw		t0, 14*4(a1)
	lw		t1, 15*4(a1)
	addi		a1, a1, 16*4
	sw		a4, 10*4(t6)
	sw		a5, 11*4(t6)
	sw		a6, 12*4(t6)
	sw		a7, 13*4(t6)
	sw		t0, 14*4(t6)
	sw		t1, 15*4(t6)
	addi		t6, t6, 16*4
	bltu		a1, a3, 3b
	andi		a2, a2, 63  /* Update count */

4:
	/* Handle trailing misalignment */
	beqz		a2, 6f
	add		a3, a1, a2

	/* Use word-oriented copy if co-aligned to word boundary */
	or		a5, a1, t6
	or		a5, a5, a3
	andi		a5, a5, 3
	bnez		a5, 5f
7:
	lw		a4, 0(a1)
	addi		a1, a1, 4
	sw		a4, 0(t6)
	addi		t6, t6, 4
	bltu		a1, a3, 7b

	ret

5:
	lb		a4, 0(a1)
	addi		a1, a1, 1
	sb		a4, 0(t6)
	addi		t6, t6, 1
	bltu		a1, a3, 5b
6:
	ret

#endif
