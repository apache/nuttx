/****************************************************************************
 * libs/libc/machine/risc-v/arch_strlen.S
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

#ifdef LIBC_BUILD_STRLEN

#include "asm.h"

.text
.global ARCH_LIBCFUN(strlen)
.type	ARCH_LIBCFUN(strlen), @function
.align	2

/************************************************************************************
 * Name: strlen
 *
 * size_t strlen(const char *s)
 *
 * Word-at-a-time DETECTNULL scanning.
 ************************************************************************************/
ARCH_LIBCFUN(strlen):
	.cfi_sections .debug_frame
	.cfi_startproc

	mv	t0, a0

.Lbyte_head:
	andi	t1, t0, SZREG-1
	beqz	t1, .Laligned
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	j	.Lbyte_head

.Laligned:
#if SZREG == 8
	lla	t3, .Lmask01
	ld	t3, 0(t3)
	lla	t4, .Lmask80
	ld	t4, 0(t4)
#else
	li	t3, 0x01010101
	li	t4, 0x80808080
#endif

.Lword_loop:
	REG_L	t1, 0(t0)
	sub	t2, t1, t3
	not	t5, t1
	and	t2, t2, t5
	and	t2, t2, t4
	bnez	t2, .Lfind_null
	addi	t0, t0, SZREG
	j	.Lword_loop

.Lfind_null:
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
#if SZREG == 8
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
	lbu	t2, 0(t0)
	beqz	t2, .Ldone
	addi	t0, t0, 1
#endif

.Ldone:
	sub	a0, t0, a0
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(strlen), .-ARCH_LIBCFUN(strlen)

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lmask01:
	.dword	0x0101010101010101
.Lmask80:
	.dword	0x8080808080808080
#endif

#endif
