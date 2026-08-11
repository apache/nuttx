/****************************************************************************
 * libs/libc/machine/risc-v/arch_strncmp.S
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

#ifdef LIBC_BUILD_STRNCMP

#include "asm.h"

.text
.global ARCH_LIBCFUN(strncmp)
.type	ARCH_LIBCFUN(strncmp), @function
.align	2

/************************************************************************************
 * Name: strncmp
 *
 * int strncmp(const char *s1, const char *s2, size_t n)
 *
 * Word-at-a-time comparison with count limit.
 ************************************************************************************/
ARCH_LIBCFUN(strncmp):
	.cfi_sections .debug_frame
	.cfi_startproc

	beqz	a2, .Lequal

	/* Check alignment consistency */

	or	t0, a0, a1
	andi	t0, t0, SZREG-1
	bnez	t0, .Lbyte_loop

	/* Both aligned - load masks */

#if SZREG == 8
	lla	t2, .Lsnc_mask01
	ld	t2, 0(t2)
	lla	t3, .Lsnc_mask80
	ld	t3, 0(t3)
#else
	li	t2, 0x01010101
	li	t3, 0x80808080
#endif

.Lword_loop:
	li	t0, SZREG
	bltu	a2, t0, .Lbyte_loop
	REG_L	t0, 0(a0)
	REG_L	t1, 0(a1)

	bne	t0, t1, .Lbyte_loop	/* words differ */

	/* DETECTNULL on t0 */

	sub	t4, t0, t2
	not	t5, t0
	and	t4, t4, t5
	and	t4, t4, t3
	bnez	t4, .Lequal		/* null found, words equal → return 0 */

	addi	a0, a0, SZREG
	addi	a1, a1, SZREG
	addi	a2, a2, -SZREG
	j	.Lword_loop

.Lbyte_loop:
	beqz	a2, .Lequal
	lbu	t0, 0(a0)
	lbu	t1, 0(a1)
	bne	t0, t1, .Ldiff
	beqz	t0, .Lequal
	addi	a0, a0, 1
	addi	a1, a1, 1
	addi	a2, a2, -1
	j	.Lbyte_loop

.Lequal:
	li	a0, 0
	ret
.Ldiff:
	sub	a0, t0, t1
	ret

	.cfi_endproc
	.size	ARCH_LIBCFUN(strncmp), .-ARCH_LIBCFUN(strncmp)

#if SZREG == 8
	.section .srodata.cst8,"aM",@progbits,8
	.align	3
.Lsnc_mask01:
	.dword	0x0101010101010101
.Lsnc_mask80:
	.dword	0x8080808080808080
#endif

#endif
