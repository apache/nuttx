/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_strcpy.S
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

#ifdef LIBC_BUILD_STRCPY

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* char *strcpy(char *dest, const char *src)
 *
 * TriCore EABI:
 *   dest -> %a4   src -> %a5   return (dest) -> %a2
 *
 * Strategy (word-at-a-time with NUL detection, shift-merge for misaligned
 * src, matching the memcpy shift-merge design):
 *   - Byte prologue copies until dest reaches a 4-byte boundary, stopping if
 *     a NUL is copied.  Keeping dest 4-aligned lets every bulk store be an
 *     aligned st.w (safe, fast).
 *   - Same congruence (src also 4-aligned): word loop reads an aligned word,
 *     tests for a zero byte with hz = (w - 0x01010101) & ~w & 0x80808080; if
 *     none, store the whole word and advance; on a hit, finish byte-wise so
 *     nothing past the NUL is written.
 *   - Different congruence: shift-merge.  Read aligned source words and
 *     funnel-merge (preload >> sb, next << (32 - sb)) into a logical word,
 *     run the same zero-byte test, and store aligned st.w until a word holds
 *     a NUL; then finish byte-wise from the logical src pointer.
 *   - The final (NUL-bearing) word is always resolved byte-by-byte so no
 *     bytes beyond the terminator are written to dest.
 *
 * All source reads are ld.w (4-byte, aligned), so this routine is safe for
 * memory-mapped flash (which rejects >8-byte burst reads) and never reads
 * across a page boundary.
 *
 * Uses only scratch registers (%d0-%d9/%d15, %a3/%a6/%a15) plus the %a2
 * return value; no CSA context save required.
 */

	.text
	.global	ARCH_LIBCFUN(strcpy)
	.type	ARCH_LIBCFUN(strcpy), %function

ARCH_LIBCFUN(strcpy):
	mov.aa	%a2, %a4		/* return value = original dest     */
	mov.aa	%a15, %a4		/* %a15 = working dst pointer       */

	/* DETECTNULL constants: %d2 = 0x01010101, %d3 = 0x80808080. */

	movh	%d2, 0x0101
	addi	%d2, %d2, 0x0101
	sh	%d3, %d2, 7

	/* Byte prologue: copy until dst (%a15) reaches an 8-byte boundary. */

.Lpre:
	mov.d	%d0, %a15
	and	%d0, %d0, 7
	jz	%d0, .Ldst_ok		/* dst 8-aligned -> bulk path       */
	ld.bu	%d0, [%a5+]
	st.b	[%a15+], %d0
	jnz	%d0, .Lpre		/* nonzero -> keep prologue         */
	ret				/* copied NUL in prologue -> done   */

.Ldst_ok:
	/* dst 8-aligned.  Is src also 8-aligned (same congruence)? */

	mov.d	%d0, %a5
	and	%d0, %d0, 7
	jnz	%d0, .Lsrc_lt8

	/* 8-byte aligned copy: ld.d + two 32-bit DETECTNULL, st.d. */

.Lword8:
	ld.d	%e0, [%a5]		/* d0:d1 = 8 bytes src (8-aligned)  */
	sub	%d4, %d0, %d2		/* low word zero-byte test          */
	andn	%d4, %d4, %d0
	and	%d4, %d4, %d3
	sub	%d5, %d1, %d2		/* high word zero-byte test         */
	andn	%d5, %d5, %d1
	and	%d5, %d5, %d3
	or	%d4, %d4, %d5		/* NUL anywhere in these 8 bytes?   */
	jnz	%d4, .Lbyte		/* NUL -> finish byte-wise (a5 kept) */
	st.d	[%a15+]8, %e0		/* store whole 8 bytes (dst 8-align) */
	lea	%a5, [%a5]8		/* a5 += 8 (add.a const4 range -8..7) */
	j	.Lword8

.Lsrc_lt8:
	/* src not 8-aligned.  4-aligned -> word copy; else shift-merge. */

	and	%d0, %d0, 3		/* d0 = src & 3 (d0 held src & 7)   */
	jnz	%d0, .Lshiftmerge

	/* Same 4-byte congruence: aligned word copy with NUL detection. */

.Lword:
	ld.w	%d1, [%a5]		/* w = *src (4-byte aligned)        */
	sub	%d4, %d1, %d2		/* zero-byte test                   */
	andn	%d4, %d4, %d1
	and	%d4, %d4, %d3
	jnz	%d4, .Lbyte		/* NUL in word -> finish byte-wise  */
	st.w	[%a15+]4, %d1		/* store whole word                 */
	add.a	%a5, 4
	j	.Lword

.Lbyte:
	/* Final word (or unaligned/short tail): copy byte-by-byte incl. NUL. */

	ld.bu	%d0, [%a5+]
	st.b	[%a15+], %d0
	jnz	%d0, .Lbyte
	ret

.Lshiftmerge:
	/* Misaligned src.  sb = (src & 3) * 8, sc = 32 - sb. */

	sh	%d8, %d0, 3		/* %d8 = shift_bits (misalign*8)    */
	rsub	%d9, %d8, 32		/* %d9 = 32 - shift_bits            */
	rsub	%d7, %d8, 0		/* %d7 = -shift_bits (logical >>)   */
	mov.d	%d0, %a5
	andn	%d0, %d0, 3		/* src_al = src & ~3                */
	mov.a	%a6, %d0
	ld.w	%d5, [%a6+]4		/* preload first aligned word       */

.Lsm_loop:
	ld.w	%d6, [%a6+]4		/* next aligned word                */
	sh	%d0, %d5, %d7		/* preload >> shift_bits            */
	sh	%d1, %d6, %d9		/* next    << (32 - shift_bits)     */
	or	%d1, %d0, %d1		/* %d1 = funnel-merged logical word */
	sub	%d4, %d1, %d2		/* zero-byte test                   */
	andn	%d4, %d4, %d1
	and	%d4, %d4, %d3
	jnz	%d4, .Lbyte		/* NUL -> finish byte-wise (%a5)    */
	st.w	[%a15+]4, %d1		/* store merged word                */
	mov	%d5, %d6		/* preload = next                   */
	add.a	%a5, 4			/* advance logical src for tail     */
	j	.Lsm_loop

	.size	ARCH_LIBCFUN(strcpy), . - ARCH_LIBCFUN(strcpy)

#endif /* LIBC_BUILD_STRCPY */
