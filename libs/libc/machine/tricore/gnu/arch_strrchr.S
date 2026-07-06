/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_strrchr.S
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

#ifdef LIBC_BUILD_STRRCHR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* char *strrchr(const char *s, int c)
 *
 * TriCore EABI:
 *   s -> %a4   c -> %d4   return -> %a2
 *
 * Strategy (word-at-a-time scan, remember last match):
 *   - Broadcast the low byte of c into all 4 lanes (cccc).
 *   - Byte prologue brings the pointer to a 4-byte boundary, remembering the
 *     last position where c was seen and stopping at the terminator.
 *   - Main loop reads one aligned word and runs two DETECTNULL tests: NUL on
 *     the word and c on (word ^ cccc).  When neither fires the whole word is
 *     skipped in one step (fast path).  When c is present (but no NUL) the
 *     four bytes are resolved to update the saved last-match pointer, then
 *     the word loop resumes.  When a NUL is present the final word is
 *     resolved byte-wise, updating the saved match up to the terminator.
 *   - Returns the last saved match pointer, or NULL if c never appeared.
 *     strrchr(s, 0) returns a pointer to the terminating NUL.
 *
 * All word reads are ld.w and 4-byte aligned, safe for memory-mapped flash.
 *
 * Scratch only (%d0-%d7, %a2/%a3/%a15); no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(strrchr)
	.type	ARCH_LIBCFUN(strrchr), %function

ARCH_LIBCFUN(strrchr):
	mov.aa	%a15, %a4		/* %a15 = working pointer           */
	mov	%d5, 0			/* %d5 = last match ptr (0 = none)  */

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
	jne	%d1, %d4, .Lpre_nonmatch
	mov.d	%d5, %a15		/* remember this match              */
.Lpre_nonmatch:
	jz	%d1, .Ldone		/* terminator -> stop               */
	add.a	%a15, 1
	j	.Lpre

.Lword_setup:
	/* DETECTNULL constants: %d2 = 0x01010101, %d3 = 0x80808080. */

	movh	%d2, 0x0101
	addi	%d2, %d2, 0x0101
	sh	%d3, %d2, 7

.Lword_loop:
	/* 8 bytes/iter via ld.d (8-aligned).  d7/d15 scratch keep d0:d1 raw for
	 * the c-test after the NUL-test.
	 */

	ld.d	%e0, [%a15]		/* d0:d1 = 8 bytes                  */
	sub	%d7, %d0, %d2		/* low: NUL test                    */
	andn	%d7, %d7, %d0
	and	%d7, %d7, %d3		/* %d7 = hz_nul(low)                */
	sub	%d15, %d1, %d2		/* high: NUL test                   */
	andn	%d15, %d15, %d1
	and	%d15, %d15, %d3		/* %d15 = hz_nul(high)              */
	or	%d7, %d7, %d15		/* NUL anywhere in 8?               */
	jnz	%d7, .Lbyte		/* NUL -> final resolve             */
	xor	%d7, %d0, %d6		/* low: c test (d0 no longer raw)   */
	sub	%d0, %d7, %d2
	andn	%d0, %d0, %d7
	and	%d0, %d0, %d3		/* %d0 = hz_c(low)                  */
	xor	%d7, %d1, %d6		/* high: c test                     */
	sub	%d1, %d7, %d2
	andn	%d1, %d1, %d7
	and	%d1, %d1, %d3		/* %d1 = hz_c(high)                 */
	or	%d0, %d0, %d1		/* c anywhere in 8?                 */
	jz	%d0, .Lword_next	/* no c, no NUL -> skip whole dword */

	/* c present in these 8 bytes (no NUL): resolve byte-wise, update. */

	mov.a	%a3, 7
.Lword_c:
	ld.bu	%d1, [%a15]
	jne	%d1, %d4, .Lword_c_next
	mov.d	%d5, %a15		/* remember match                   */
.Lword_c_next:
	add.a	%a15, 1
	loop	%a3, .Lword_c
	j	.Lword_loop

.Lword_next:
	lea	%a15, [%a15]8		/* fast path: advance whole dword   */
	j	.Lword_loop

.Lbyte:
	/* Final word contains the NUL: scan byte-wise, update match to NUL. */

	ld.bu	%d1, [%a15]
	jne	%d1, %d4, .Lbyte_nonmatch
	mov.d	%d5, %a15		/* remember match                   */
.Lbyte_nonmatch:
	jz	%d1, .Ldone		/* terminator -> stop               */
	add.a	%a15, 1
	j	.Lbyte

.Ldone:
	mov.a	%a2, %d5		/* return last match (or NULL)      */
	ret

	.size	ARCH_LIBCFUN(strrchr), . - ARCH_LIBCFUN(strrchr)

#endif /* LIBC_BUILD_STRRCHR */
