/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_strchr.S
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

#ifdef LIBC_BUILD_STRCHR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* char *strchr(const char *s, int c)
 *
 * TriCore EABI:
 *   s -> %a4   c -> %d4   return -> %a2
 *
 * Strategy (word-at-a-time dual detection):
 *   - Broadcast the low byte of c into all 4 lanes (cccc).
 *   - Byte prologue brings the pointer to a 4-byte boundary, checking each
 *     byte for c and for the terminator.
 *   - Main loop reads one aligned word and runs two DETECTNULL tests: one on
 *     the word itself (locate NUL) and one on (word ^ cccc) (locate c).  If
 *     either reports a hit, resolve byte-wise; otherwise advance a word.
 *   - Byte resolution returns the pointer on the first c, NULL on the NUL if
 *     c was not found first.  strchr(s, 0) returns a pointer to the NUL (the
 *     byte == c test fires because c == 0).
 *
 * All word reads are ld.w and 4-byte aligned, so a read never crosses a page
 * and this routine is safe for memory-mapped flash.
 *
 * Scratch only (%d0-%d7, %a2/%a15); no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(strchr)
	.type	ARCH_LIBCFUN(strchr), %function

ARCH_LIBCFUN(strchr):
	mov.aa	%a15, %a4		/* %a15 = working pointer           */

	/* Target byte kept two ways: %d4 = c & 0xff, %d6 = broadcast. */

	and	%d4, %d4, 0xff
	sh	%d6, %d4, 8
	or	%d6, %d6, %d4
	sh	%d0, %d6, 16
	or	%d6, %d6, %d0		/* %d6 = 0xcccccccc                 */

.Lpre:
	mov.d	%d1, %a15
	and	%d1, %d1, 7
	jz	%d1, .Lword_setup	/* 8-aligned -> dword loop          */
	ld.bu	%d1, [%a15]
	jeq	%d1, %d4, .Lfound	/* byte == c                        */
	jz	%d1, .Lnul		/* terminator, c not found          */
	add.a	%a15, 1
	j	.Lpre

.Lword_setup:
	/* DETECTNULL constants: %d2 = 0x01010101, %d3 = 0x80808080. */

	movh	%d2, 0x0101
	addi	%d2, %d2, 0x0101
	sh	%d3, %d2, 7

.Lword_loop:
	/* 8 bytes/iter via ld.d (8-aligned).  Dual SWAR (NUL + c) on both the
	 * low (%d0) and high (%d1) words, OR-combined into %d5.
	 */

	ld.d	%e0, [%a15]		/* d0:d1 = 8 bytes (8-aligned)      */
	sub	%d5, %d0, %d2		/* low: NUL test                    */
	andn	%d5, %d5, %d0
	and	%d5, %d5, %d3		/* %d5 = hz_nul(low)                */
	xor	%d7, %d0, %d6		/* x = low ^ cccc                   */
	sub	%d15, %d7, %d2		/* low: c test                      */
	andn	%d15, %d15, %d7
	and	%d15, %d15, %d3		/* %d15 = hz_c(low)                 */
	or	%d5, %d5, %d15		/* %d5 = hit(low)                   */
	sub	%d15, %d1, %d2		/* high: NUL test                   */
	andn	%d15, %d15, %d1
	and	%d15, %d15, %d3
	or	%d5, %d5, %d15		/* accumulate hz_nul(high)          */
	xor	%d7, %d1, %d6		/* x = high ^ cccc                  */
	sub	%d15, %d7, %d2		/* high: c test                     */
	andn	%d15, %d15, %d7
	and	%d15, %d15, %d3
	or	%d5, %d5, %d15		/* %d5 = any NUL or c in these 8 B? */
	jnz	%d5, .Lbyte		/* hit -> resolve byte-wise         */
	lea	%a15, [%a15]8		/* a15 += 8 (add.a const4 range -8..7) */
	j	.Lword_loop

.Lbyte:
	ld.bu	%d1, [%a15]
	jeq	%d1, %d4, .Lfound	/* byte == c                        */
	jz	%d1, .Lnul		/* terminator, c not found          */
	add.a	%a15, 1
	j	.Lbyte

.Lfound:
	mov.aa	%a2, %a15		/* return pointer to match          */
	ret

.Lnul:
	mov.a	%a2, 0			/* return NULL                      */
	ret

	.size	ARCH_LIBCFUN(strchr), . - ARCH_LIBCFUN(strchr)

#endif /* LIBC_BUILD_STRCHR */
