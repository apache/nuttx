/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_strlen.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_STRLEN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* size_t strlen(const char *s)
 *
 * TriCore EABI:
 *   s -> %a4   return (length) -> %d2
 *
 * Strategy (word-at-a-time null detection, DETECTNULL):
 *   - Byte prologue advances until the pointer is 4-byte aligned (checking
 *     each byte for the terminator on the way).
 *   - Main loop reads one aligned 4-byte word and tests for a zero byte with
 *     the classic bit trick:  hz = (w - 0x01010101) & ~w & 0x80808080.
 *     hz is non-zero iff w contains at least one zero byte.
 *   - On a hit, rewind to the start of that word and scan byte by byte to
 *     find the exact terminator position.
 *
 * All word reads are ld.w (4-byte single transfer), so this routine is safe
 * for memory-mapped flash (which rejects >8-byte burst reads).
 *
 * Scratch only (%d0-%d4, %a15); no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(strlen)
	.type	ARCH_LIBCFUN(strlen), %function

ARCH_LIBCFUN(strlen):
	mov.aa	%a15, %a4		/* %a15 = working pointer           */

	/* Byte prologue: advance until %a15 is 8-byte aligned. */

.Lpre:
	mov.d	%d0, %a15
	and	%d0, %d0, 7
	jz	%d0, .Lword_setup	/* 8-aligned -> dword loop          */
	ld.bu	%d0, [%a15]
	jz	%d0, .Ldone		/* terminator found in prologue     */
	add.a	%a15, 1
	j	.Lpre

.Lword_setup:
	/* Build DETECTNULL constants:
	 *   %d2 = 0x01010101, %d3 = 0x80808080 (= 0x01010101 << 7).
	 */

	movh	%d2, 0x0101
	addi	%d2, %d2, 0x0101	/* %d2 = 0x01010101                 */
	sh	%d3, %d2, 7		/* %d3 = 0x80808080                 */

.Lword_loop:
	/* 8 bytes/iter via ld.d (8-aligned).  Two 32-bit DETECTNULL SWARs on
	 * the low/high words, OR-combined (haszero needs no cross-word borrow).
	 */

	ld.d	%e0, [%a15+]8		/* d0:d1 = 8 bytes; ptr += 8        */
	sub	%d4, %d0, %d2		/* low: w - 0x01010101              */
	andn	%d4, %d4, %d0
	and	%d4, %d4, %d3		/* %d4 = hz(low)                    */
	sub	%d5, %d1, %d2		/* high: w - 0x01010101             */
	andn	%d5, %d5, %d1
	and	%d5, %d5, %d3		/* %d5 = hz(high)                   */
	or	%d4, %d4, %d5		/* any zero byte in these 8?        */
	jz	%d4, .Lword_loop	/* no zero byte -> keep scanning    */

	/* A zero byte is in this dword; rewind to its start and locate it. */

	add.a	%a15, -8
.Lbyte_find:
	ld.bu	%d0, [%a15]
	jz	%d0, .Ldone
	add.a	%a15, 1
	j	.Lbyte_find

.Ldone:
	/* length = %a15 - original s (%a4) */

	mov.d	%d0, %a15
	mov.d	%d2, %a4
	sub	%d2, %d0, %d2
	ret
	.size	ARCH_LIBCFUN(strlen), . - ARCH_LIBCFUN(strlen)

#endif /* LIBC_BUILD_STRLEN */
