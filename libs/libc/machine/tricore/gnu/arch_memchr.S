/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_memchr.S
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

#ifdef LIBC_BUILD_MEMCHR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* void *memchr(const void *s, int c, size_t n)
 *
 * TriCore EABI:
 *   s -> %a4   c -> %d4   n -> %d5   return -> %a2
 *
 * Strategy (word-at-a-time byte search, DETECTNULL on XOR):
 *   - Broadcast the low byte of c into all 4 lanes: cccc = 0x01010101 * c.
 *   - Byte prologue brings the pointer to a 4-byte boundary while there is
 *     budget, checking each byte directly.
 *   - Main loop reads one aligned word (while >= 4 bytes of budget remain),
 *     XORs it with cccc so any matching lane becomes 0x00, then applies the
 *     zero-byte test hz = (x - 0x01010101) & ~x & 0x80808080.  hz != 0 iff a
 *     matching byte exists in the word; on a hit, resolve byte-wise.
 *   - Remaining tail (< 4 bytes of budget) is scanned byte-wise.
 *   - Returns pointer to the first matching byte, or NULL if none within n.
 *
 * All word reads are ld.w (4-byte single transfer), so this routine is safe
 * for memory-mapped flash, and the word loop never advances past the budget.
 *
 * Scratch only (%d0-%d6, %a2/%a15); no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(memchr)
	.type	ARCH_LIBCFUN(memchr), %function

ARCH_LIBCFUN(memchr):
	mov.aa	%a15, %a4		/* %a15 = working pointer           */
	jz	%d5, .Lnotfound		/* n == 0 -> not found              */

	/* Target byte kept two ways:
	 *   %d4 = c & 0xff        (for direct byte comparisons)
	 *   %d6 = c broadcast     (0x01010101 * c, for word XOR) via shift-or
	 */

	and	%d4, %d4, 0xff		/* isolate low byte of c            */
	sh	%d6, %d4, 8
	or	%d6, %d6, %d4		/* %d6 = 0x0000cccc                 */
	sh	%d0, %d6, 16
	or	%d6, %d6, %d0		/* %d6 = 0xcccccccc (broadcast)     */

	/* Byte prologue: advance until %a15 is 8-byte aligned. */

.Lpre:
	mov.d	%d1, %a15
	and	%d1, %d1, 7
	jz	%d1, .Lword_setup	/* 8-aligned -> dword loop          */
	ld.bu	%d1, [%a15]
	jeq	%d1, %d4, .Lfound	/* byte == c                        */
	add.a	%a15, 1
	add	%d5, -1
	jnz	%d5, .Lpre
	j	.Lnotfound		/* budget exhausted                 */

.Lword_setup:
	/* DETECTNULL constants: %d2 = 0x01010101, %d3 = 0x80808080. */

	movh	%d2, 0x0101
	addi	%d2, %d2, 0x0101
	sh	%d3, %d2, 7

	/* Main loop: 8 bytes/iter via ld.d over the whole-dword count using a
	 * hardware loop (zero branch overhead).  Match detection is two 32-bit
	 * haszero SWARs on the low/high words, OR-combined.
	 */

	sh	%d15, %d5, -3		/* d15 = dword count = n >> 3       */
	jz	%d15, .Lbyte		/* < 8 bytes of budget -> byte tail */
	and	%d5, %d5, 7		/* d5 = tail bytes after whole dwords */
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lword_loop:
	ld.d	%e0, [%a15+]8		/* d0:d1 = 8 bytes (8-aligned, post-inc) */
	xor	%d0, %d0, %d6		/* low word: matching lanes -> 0x00 */
	sub	%d15, %d0, %d2		/* x - 0x01010101                   */
	andn	%d15, %d15, %d0
	and	%d15, %d15, %d3		/* %d15 = hasmatch(low)             */
	xor	%d1, %d1, %d6		/* high word                        */
	sub	%d0, %d1, %d2
	andn	%d0, %d0, %d1
	and	%d0, %d0, %d3		/* %d0 = hasmatch(high)             */
	or	%d15, %d15, %d0		/* any match in these 8 bytes?      */
	jnz	%d15, .Lmatch		/* match -> back up and resolve byte */
	loop	%a3, .Lword_loop
	j	.Lbyte			/* no match in dwords -> byte tail  */

.Lmatch:
	/* Match within the just-loaded 8 bytes; back up and resolve byte-wise.
	 * The matching byte is guaranteed in these 8, so the byte loop exits
	 * via .Lfound before the budget is exhausted.
	 */

	lea	%a15, [%a15]-8
	mov	%d5, 8

.Lbyte:
	jz	%d5, .Lnotfound		/* budget exhausted                 */
	ld.bu	%d1, [%a15]
	jeq	%d1, %d4, .Lfound	/* byte == c                        */
	add.a	%a15, 1
	add	%d5, -1
	j	.Lbyte

.Lfound:
	mov.aa	%a2, %a15		/* return pointer to match          */
	ret

.Lnotfound:
	mov.a	%a2, 0			/* return NULL                      */
	ret

	.size	ARCH_LIBCFUN(memchr), . - ARCH_LIBCFUN(memchr)

#endif /* LIBC_BUILD_MEMCHR */
