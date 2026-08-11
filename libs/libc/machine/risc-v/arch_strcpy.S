/****************************************************************************
 * libs/libc/machine/risc-v/arch_strcpy.S
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

#ifdef LIBC_BUILD_STRCPY

#include "asm.h"

#ifndef STRCPY
#define STRCPY ARCH_LIBCFUN(strcpy)
#endif

.text
.global STRCPY
.type	STRCPY, @function
.align	2

/************************************************************************************
 * Name: strcpy
 *
 * char *strcpy(char *dst, const char *src)
 *
 * Word-at-a-time with DETECTNULL. Supports stpcpy/strncpy/stpncpy via macros.
 ************************************************************************************/
STRCPY:
	.cfi_sections .debug_frame
	.cfi_startproc

	mv	t6, a0			/* save original dst */
#ifdef USE_AS_STRNCPY
	beqz	a2, .Ldone_ret
	mv	a7, a2			/* a7 = n */
#endif

	/* Check src/dst alignment consistency */

	xor	t0, a0, a1
	andi	t0, t0, SZREG-1
	bnez	t0, .Lbyte_loop

	/* Align to SZREG boundary */

	andi	t0, a0, SZREG-1
	beqz	t0, .Laligned

.Lalign_head:
#ifdef USE_AS_STRNCPY
	beqz	a7, .Lnull_pad
#endif
	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Lfound_null
	addi	a0, a0, 1
	addi	a1, a1, 1
#ifdef USE_AS_STRNCPY
	addi	a7, a7, -1
#endif
	andi	t0, a0, SZREG-1
	bnez	t0, .Lalign_head

.Laligned:
#if SZREG == 8
	lla	t2, .Lsc_mask01
	ld	t2, 0(t2)
	lla	t3, .Lsc_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

.Lword_loop:
#ifdef USE_AS_STRNCPY
	li	t0, SZREG
	bltu	a7, t0, .Lbyte_loop
#endif
	REG_L	t0, 0(a1)

	/* DETECTNULL */

	sub	t1, t0, t2
	not	t4, t0
	and	t1, t1, t4
	and	t1, t1, t3
	bnez	t1, .Lbyte_loop		/* has null: finish byte-by-byte */

	REG_S	t0, 0(a0)
	addi	a0, a0, SZREG
	addi	a1, a1, SZREG
#ifdef USE_AS_STRNCPY
	addi	a7, a7, -SZREG
#endif
	j	.Lword_loop

.Lbyte_loop:
#ifdef USE_AS_STRNCPY
	beqz	a7, .Lnull_pad
#endif
	lbu	t0, 0(a1)
	sb	t0, 0(a0)
	beqz	t0, .Lfound_null
	addi	a0, a0, 1
	addi	a1, a1, 1
#ifdef USE_AS_STRNCPY
	addi	a7, a7, -1
#endif
	j	.Lbyte_loop

.Lfound_null:
	/* a0 points to where we wrote the null byte */
#ifdef USE_AS_STRNCPY
	addi	a7, a7, -1		/* count the null */
.Lnull_pad:
	beqz	a7, .Ldone_ret
.Lpad_loop:
	addi	a0, a0, 1
	sb	zero, 0(a0)
	addi	a7, a7, -1
	bnez	a7, .Lpad_loop
#endif

.Ldone_ret:
#ifdef USE_AS_STPCPY
	/* stpcpy: return pointer to the null byte written */
	ret
#else
	mv	a0, t6
	ret
#endif

	.cfi_endproc
	.size	STRCPY, .-STRCPY

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lsc_mask01:
	.dword	0x0101010101010101
.Lsc_mask80:
	.dword	0x8080808080808080
#endif

#endif
