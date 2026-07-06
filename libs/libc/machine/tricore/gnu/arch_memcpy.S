/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_memcpy.S
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

#ifdef LIBC_BUILD_MEMCPY

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* void *memcpy(void *dst, const void *src, size_t n)
 *
 * TriCore EABI:
 *   dst -> %a4   src -> %a5   n -> %d4   return (dst) -> %a2
 *
 * Strategy (quad-word block copy, matches vendor throughput):
 *   - n < 16, or src/dst not sharing 4-byte (word) congruence: copy byte by
 *     byte.
 *   - Otherwise: copy a byte prologue to reach a 4-byte boundary, then copy
 *     16 bytes per iteration with ld.dd/st.dd (quad-word Q0 = D0:D1:D2:D3)
 *     driven by the hardware loop instruction, then copy the < 16 byte tail.
 *   - Quad load/store are only issued once src and dst are 4-byte aligned
 *     (TC1.8 requires word alignment for ld.dd/st.dd, not 16-byte; this is the
 *     same minimum the vendor Tasking memcpy relies on), so no data alignment
 *     trap can occur.
 *
 * Requires the assembler to target TC1.8 for ld.dd/st.dd (see AFLAGS
 * -Wa,-mtc18 in Make.defs / CMakeLists.txt).
 *
 * Q0 is a lower-context register pair group; together with the other scratch
 * registers (%d15, %a2/%a3/%a15) it needs no CSA context save.
 */

	.text
	.global	ARCH_LIBCFUN(memcpy)
	.type	ARCH_LIBCFUN(memcpy), %function

ARCH_LIBCFUN(memcpy):
	mov.aa	%a2, %a4		/* return value = original dst      */
	jz	%d4, .Ldone		/* n == 0 -> nothing to do          */
	mov.aa	%a15, %a4		/* %a15 = working dst pointer        */

	mov	%d15, 16
	jlt.u	%d4, %d15, .Lbyte_setup	/* n < 16 -> byte copy              */

	/* Require src and dst to share the same 8-byte congruence so that
	 * aligning dst to 8 also aligns src to 8.  (src ^ dst) & 7 == 0 ?
	 *
	 * 8-byte alignment is mandatory for DLMU targets (0xB0/0x90): the DLMU
	 * has 64-bit ECC granularity, so a quad st.dd starting at a 4-mod-8
	 * offset would partially write two 64-bit ECC words as sub-doubleword
	 * read-modify-write, corrupting shared memory. Aligning to 8 makes every
	 * quad write two whole 64-bit ECC words.
	 */

	mov.d	%d0, %a5
	mov.d	%d1, %a15
	xor	%d0, %d0, %d1
	and	%d0, %d0, 7
	jnz	%d0, .Lshiftmerge	/* different congruence -> word shift-merge */

	/* Prologue: copy (8 - (dst & 7)) bytes to reach an 8-byte boundary. */

	mov.d	%d15, %a15
	and	%d15, %d15, 7
	jz	%d15, .Laligned
	rsub	%d15, %d15, 8		/* %d15 = 8 - misalign (1..7)       */
	sub	%d4, %d4, %d15		/* n -= prologue bytes              */
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lpre:
	ld.bu	%d0, [%a5+]
	st.b	[%a15+], %d0
	loop	%a3, .Lpre

.Laligned:
	/* Main loop: 32 bytes per iteration (two quad copies unrolled) to
	 * halve the hardware-loop branch overhead.  chunks = n >> 5.
	 */

	sh	%d15, %d4, -5		/* %d15 = n / 32                    */
	jz	%d15, .Lquad1
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lqloop:
	ld.d	%e0, [%a5+]8		/* 2x 8-byte (64-bit) block loads:  */
	ld.d	%e2, [%a5+]8		/* flash aperture rejects 16B burst */
	st.dd	[%a15+]16, %q0
	ld.d	%e0, [%a5+]8		/* 2x 8-byte (64-bit) block loads:  */
	ld.d	%e2, [%a5+]8		/* flash aperture rejects 16B burst */
	st.dd	[%a15+]16, %q0
	loop	%a3, .Lqloop

.Lquad1:
	/* Leftover single 16-byte block (n & 16), straight-line, no setup. */

	jz.t	%d4, 4, .Ltail		/* bit4 clear -> no 16-byte block   */
	ld.d	%e0, [%a5+]8		/* 2x 8-byte (64-bit) block loads:  */
	ld.d	%e2, [%a5+]8		/* flash aperture rejects 16B burst */
	st.dd	[%a15+]16, %q0

.Ltail:
	and	%d4, %d4, 15		/* remaining tail bytes             */
	jz	%d4, .Ldone

.Lbyte_setup:
	mov.a	%a3, %d4
	add.a	%a3, -1			/* loop counter = nbytes - 1        */
.Lbyte:
	ld.bu	%d0, [%a5+]
	st.b	[%a15+], %d0
	loop	%a3, .Lbyte

.Ldone:
	ret

	/* Word-granular copy for src/dst that do NOT share 8-byte congruence
	 * (previously fell back to slow byte copy).  Align dst to 4, then:
	 *   - src also 4-aligned  -> plain ld.w/st.w word copy
	 *   - src misaligned      -> shift-merge (funnel two aligned words)
	 * All loads are ld.w (4-byte single transfer), so this path is safe
	 * for memory-mapped flash (which rejects >8-byte burst reads).
	 * Scratch: %d0/%d1/%d5-%d9/%d15, %a3/%a6 (caller-saved, no CSA).
	 */

.Lshiftmerge:
	/* Prologue: byte-copy until dst (%a15) reaches a 4-byte boundary. */

	mov.d	%d15, %a15
	and	%d15, %d15, 3
	jz	%d15, .Lsm_dstok
	rsub	%d15, %d15, 4		/* bytes to reach 4-align (1..3)     */
	sub	%d4, %d4, %d15		/* n -= prologue                     */
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lsm_pre:
	ld.bu	%d0, [%a5+]
	st.b	[%a15+], %d0
	loop	%a3, .Lsm_pre

.Lsm_dstok:
	/* dst now 4-aligned.  Check whether src is also 4-aligned. */

	mov.d	%d0, %a5
	and	%d0, %d0, 3
	jz	%d0, .Lsm_aligned

	/* Misaligned src: shift-merge.  sb = misalign*8, sc = 32 - sb. */

	sh	%d8, %d0, 3		/* %d8 = shift_bits (misalign*8)     */
	rsub	%d9, %d8, 32		/* %d9 = 32 - shift_bits             */
	rsub	%d7, %d8, 0		/* %d7 = -shift_bits (logical >>)    */
	mov.d	%d0, %a5
	andn	%d0, %d0, 3		/* src_al = src & ~3                 */
	mov.a	%a6, %d0
	ld.w	%d5, [%a6+]4		/* preload first aligned word        */
	sh	%d15, %d4, -2		/* word count = n >> 2               */
	jz	%d15, .Lsm_tail
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lsm_mloop:
	ld.w	%d6, [%a6+]4		/* next aligned word                 */
	sh	%d0, %d5, %d7		/* preload >> shift_bits             */
	sh	%d1, %d6, %d9		/* next    << (32 - shift_bits)      */
	or	%d0, %d0, %d1		/* funnel-merged word                */
	st.w	[%a15+]4, %d0
	mov	%d5, %d6		/* preload = next                    */
	add.a	%a5, 4			/* advance logical src for tail      */
	loop	%a3, .Lsm_mloop
	j	.Lsm_tail

.Lsm_aligned:
	/* src also 4-aligned: plain word copy (ld.w advances %a5). */

	sh	%d15, %d4, -2		/* word count = n >> 2               */
	jz	%d15, .Lsm_tail
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lsm_aloop:
	ld.w	%d0, [%a5+]4
	st.w	[%a15+]4, %d0
	loop	%a3, .Lsm_aloop

.Lsm_tail:
	and	%d4, %d4, 3		/* remaining tail bytes (0..3)       */
	jz	%d4, .Ldone
	j	.Lbyte_setup		/* reuse byte-copy tail (%a5->%a15)  */

	.size	ARCH_LIBCFUN(memcpy), . - ARCH_LIBCFUN(memcpy)

#endif /* LIBC_BUILD_MEMCPY */
