/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_strnlen.S
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

#ifdef LIBC_BUILD_STRNLEN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* size_t strnlen(const char *s, size_t maxlen)
 *
 * TriCore EABI:
 *   s -> %a4   maxlen -> %d4   return (length) -> %d2
 *
 * Strategy (word-at-a-time null detection, DETECTNULL, bounded by maxlen):
 *   - Byte prologue advances until the pointer is 4-byte aligned, checking
 *     each byte for the terminator and decrementing the remaining budget.
 *   - Main loop reads one aligned 4-byte word while at least 4 bytes of
 *     budget remain, testing for a zero byte with the classic bit trick:
 *     hz = (w - 0x01010101) & ~w & 0x80808080.
 *   - On a hit (or when < 4 bytes of budget remain), fall to a bounded byte
 *     loop that stops at the terminator or when the budget is exhausted.
 *   - Result is min(strlen(s), maxlen) = working pointer - s.
 *
 * All word reads are ld.w (4-byte single transfer), so this routine is safe
 * for memory-mapped flash (which rejects >8-byte burst reads).  The word
 * loop never advances past the maxlen budget, so it never reads beyond the
 * bytes strnlen is permitted to examine.
 *
 * Scratch only (%d0-%d5, %a15); no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(strnlen)
	.type	ARCH_LIBCFUN(strnlen), %function

ARCH_LIBCFUN(strnlen):
	mov.aa	%a15, %a4		/* %a15 = working pointer           */
	jz	%d4, .Ldone		/* maxlen == 0 -> length 0          */

	/* Byte prologue: advance until %a15 is 8-byte aligned. */

.Lpre:
	mov.d	%d0, %a15
	and	%d0, %d0, 7
	jz	%d0, .Lword_setup	/* 8-aligned -> dword loop          */
	ld.bu	%d0, [%a15]
	jz	%d0, .Ldone		/* terminator found in prologue     */
	add.a	%a15, 1
	add	%d4, -1			/* one byte of budget consumed      */
	jnz	%d4, .Lpre
	j	.Ldone			/* budget exhausted                 */

.Lword_setup:
	/* Build DETECTNULL constants:
	 *   %d2 = 0x01010101, %d3 = 0x80808080 (= 0x01010101 << 7).
	 */

	movh	%d2, 0x0101
	addi	%d2, %d2, 0x0101	/* %d2 = 0x01010101                 */
	sh	%d3, %d2, 7		/* %d3 = 0x80808080                 */

	/* Main loop: 8 bytes/iter via ld.d over the whole-dword count using a
	 * hardware loop (zero branch overhead).  Two 32-bit DETECTNULL SWARs.
	 */

	sh	%d5, %d4, -3		/* dword count = budget >> 3        */
	jz	%d5, .Lbyte		/* < 8 bytes budget -> byte tail    */
	and	%d4, %d4, 7		/* d4 = tail bytes after whole dwords */
	mov.a	%a3, %d5
	add.a	%a3, -1
.Lword_loop:
	ld.d	%e0, [%a15+]8		/* d0:d1 = 8 bytes; ptr += 8        */
	sub	%d5, %d0, %d2		/* low: w - 0x01010101              */
	andn	%d5, %d5, %d0
	and	%d5, %d5, %d3		/* %d5 = hz(low)                    */
	sub	%d6, %d1, %d2		/* high: w - 0x01010101             */
	andn	%d6, %d6, %d1
	and	%d6, %d6, %d3		/* %d6 = hz(high)                   */
	or	%d5, %d5, %d6		/* any zero byte in these 8?        */
	jnz	%d5, .Lmatch		/* zero byte -> resolve byte-wise   */
	loop	%a3, .Lword_loop
	j	.Lbyte			/* no NUL in dwords -> byte tail    */

.Lmatch:
	/* NUL within the just-loaded 8 bytes; back up and resolve byte-wise. */

	lea	%a15, [%a15]-8
	mov	%d4, 8

.Lbyte:
	jz	%d4, .Ldone		/* budget exhausted                 */
	ld.bu	%d0, [%a15]
	jz	%d0, .Ldone		/* terminator found                 */
	add.a	%a15, 1
	add	%d4, -1
	j	.Lbyte

.Ldone:
	/* length = %a15 - original s (%a4) */

	mov.d	%d0, %a15
	mov.d	%d2, %a4
	sub	%d2, %d0, %d2
	ret
	.size	ARCH_LIBCFUN(strnlen), . - ARCH_LIBCFUN(strnlen)

#endif /* LIBC_BUILD_STRNLEN */
