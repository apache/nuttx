/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_strncmp.S
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

#ifdef LIBC_BUILD_STRNCMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* int strncmp(const char *s1, const char *s2, size_t n)
 *
 * TriCore EABI:
 *   s1 -> %a4   s2 -> %a5   n -> %d4   return -> %d2
 *
 * Strategy (word-at-a-time, bounded by n, NUL-terminated):
 *   - When n >= 8 and s1/s2 share 4-byte congruence: a byte prologue brings
 *     both to a 4-byte boundary (comparing and NUL-checking each byte and
 *     decrementing n), then compare 4 bytes per iteration with ld.w while at
 *     least 4 bytes of budget remain.  A word iteration:
 *       * mismatch  -> back up 4 bytes and resolve byte-wise;
 *       * equal word containing a NUL -> both strings ended equally -> 0;
 *       * equal word, no NUL -> advance, n -= 4.
 *   - Otherwise (n < 8, < 4 budget, or different congruence) compare byte by
 *     byte: differ -> return diff; NUL -> return 0; else n--, continue.
 *   - Exhausting n returns 0 (first n chars equal).
 *
 * All word reads are ld.w (4-byte aligned), so a read never crosses a page,
 * never runs past the NUL into an unmapped page, and never past the n budget.
 *
 * Scratch only (%d0-%d5/%d15, %a3); no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(strncmp)
	.type	ARCH_LIBCFUN(strncmp), %function

ARCH_LIBCFUN(strncmp):
	jz	%d4, .Lret0		/* n == 0 -> equal                  */

	mov	%d15, 8
	jlt.u	%d4, %d15, .Lbyte	/* n < 8 -> byte compare            */

	/* Same 8-byte congruence?  (s1 ^ s2) & 7 == 0 */

	mov.d	%d0, %a4
	mov.d	%d1, %a5
	xor	%d0, %d0, %d1
	and	%d0, %d0, 7
	jnz	%d0, .Lbyte		/* different alignment -> byte      */

	/* Byte prologue to reach an 8-byte boundary (n >= 8, so safe). */

	mov.d	%d0, %a4
	and	%d0, %d0, 7
	jz	%d0, .Lword_setup
	rsub	%d0, %d0, 8		/* prologue bytes = 1..7            */
.Lpre:
	ld.bu	%d15, [%a4+]
	ld.bu	%d1, [%a5+]
	jne	%d15, %d1, .Lretbyte	/* differ -> return d15 - d1        */
	add	%d4, -1			/* one byte consumed                */
	jz	%d15, .Lret0		/* NUL -> equal                     */
	add	%d0, -1
	jnz	%d0, .Lpre

.Lword_setup:
.Lword:
	/* 8 bytes/iter: ld.d s1 -> d0:d1, ld.d s2 -> d2:d3 (8-aligned). */

	mov	%d15, 8
	jlt.u	%d4, %d15, .Lbyte	/* < 8 budget -> byte tail          */
	ld.d	%e0, [%a4+]8		/* d0:d1 = 8 bytes from s1          */
	ld.d	%e2, [%a5+]8		/* d2:d3 = 8 bytes from s2          */
	jne	%d0, %d2, .Lworddiff	/* low word differs                 */
	jne	%d1, %d3, .Lworddiff	/* high word differs                */
	add	%d4, -8			/* 8 bytes consumed                 */
	mov	%d15, 0			/* zero register for eq.b NUL test  */
	eq.b	%d6, %d0, %d15		/* NUL bytes in low word            */
	eq.b	%d7, %d1, %d15		/* NUL bytes in high word           */
	or	%d6, %d6, %d7		/* any NUL in these 8 equal bytes?  */
	jz	%d6, .Lword		/* no NUL -> keep scanning          */
	j	.Lret0			/* equal dword has NUL -> equal     */

.Lworddiff:
	/* Dwords differ within 8 bytes; back up and resolve byte-wise.  The
	 * first differing byte is within these 8 bytes and within budget
	 * (budget was >= 8 here), so the byte loop exits via .Lretbyte.
	 */

	lea	%a4, [%a4]-8
	lea	%a5, [%a5]-8
	/* fall through to byte compare */

.Lbyte:
	jz	%d4, .Lret0		/* budget exhausted -> equal        */
	ld.bu	%d15, [%a4+]
	ld.bu	%d1, [%a5+]
	jne	%d15, %d1, .Lretbyte
	add	%d4, -1
	jz	%d15, .Lret0		/* NUL (both equal) -> equal        */
	jnz	%d4, .Lbyte
	/* fall through: budget exhausted -> equal */

.Lret0:
	mov	%d2, 0
	ret

.Lretbyte:
	sub	%d2, %d15, %d1		/* return (unsigned char) diff      */
	ret

	.size	ARCH_LIBCFUN(strncmp), . - ARCH_LIBCFUN(strncmp)

#endif /* LIBC_BUILD_STRNCMP */
