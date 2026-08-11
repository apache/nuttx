/****************************************************************************
 * libs/libc/machine/risc-v/arch_memmove.S
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

#ifdef LIBC_BUILD_MEMMOVE

#include "asm.h"

.text
.global ARCH_LIBCFUN(memmove)
.type	ARCH_LIBCFUN(memmove), @function
.align	2

/************************************************************************************
 * Name: memmove
 *
 * void *memmove(void *dst, const void *src, size_t n)
 *
 * Direction-aware copy with reverse path for overlapping regions.
 ************************************************************************************/
ARCH_LIBCFUN(memmove):
	.cfi_sections .debug_frame
	.cfi_startproc

	beqz	a2, .Ldone
	bltu	a1, a0, .Lcheck_overlap
	tail	ARCH_LIBCFUN(memcpy)

.Lcheck_overlap:
	add	t0, a1, a2
	bleu	t0, a0, .Lfwd
	j	.Lreverse
.Lfwd:
	tail	ARCH_LIBCFUN(memcpy)

.Lreverse:
	mv	t6, a0
	add	t0, a0, a2
	add	t1, a1, a2

	li	t2, 2*SZREG
	bleu	a2, t2, .Lrev_byte

	andi	t2, t0, SZREG-1
	beqz	t2, .Lrev_aligned
	sub	a2, a2, t2
.Lrev_align:
	addi	t0, t0, -1
	addi	t1, t1, -1
	lbu	t3, 0(t1)
	sb	t3, 0(t0)
	addi	t2, t2, -1
	bnez	t2, .Lrev_align

.Lrev_aligned:
	andi	t2, t1, SZREG-1
	bnez	t2, .Lrev_byte_remain

	li	t3, SZREG
	bltu	a2, t3, .Lrev_byte_remain

.Lrev_word:
	addi	t0, t0, -SZREG
	addi	t1, t1, -SZREG
	REG_L	t2, 0(t1)
	REG_S	t2, 0(t0)
	sub	a2, a2, t3
	bgeu	a2, t3, .Lrev_word

.Lrev_byte_remain:
	beqz	a2, .Lrev_done
.Lrev_byte:
	addi	t0, t0, -1
	addi	t1, t1, -1
	lbu	t2, 0(t1)
	sb	t2, 0(t0)
	addi	a2, a2, -1
	bnez	a2, .Lrev_byte

.Lrev_done:
	mv	a0, t6
.Ldone:
	ret
	.cfi_endproc
	.size	ARCH_LIBCFUN(memmove), .-ARCH_LIBCFUN(memmove)

#endif
