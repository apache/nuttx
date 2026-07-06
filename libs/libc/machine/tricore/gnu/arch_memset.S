/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_memset.S
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

#ifdef LIBC_BUILD_MEMSET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* void *memset(void *s, int c, size_t n)
 *
 * TriCore EABI:
 *   s -> %a4   c -> %d4   n -> %d5   return (s) -> %a2
 *
 * Strategy (quad-word block fill, matches vendor throughput):
 *   - Broadcast the low byte of c across the quad-word Q0 = D0:D1:D2:D3.
 *   - n < 16, or dst not 8-byte aligned after a short byte prologue: the
 *     prologue brings dst to an 8-byte boundary, then fill 32 bytes per
 *     iteration (two st.dd unrolled) driven by the hardware loop, a single
 *     leftover 16-byte st.dd, then the < 16 byte tail.
 *   - Quad stores are only issued once dst is 8-byte aligned.  8-byte
 *     alignment is mandatory for DLMU targets (0xB0/0x90): the DLMU has
 *     64-bit ECC granularity, so a quad st.dd starting at a 4-mod-8 offset
 *     would partially write two 64-bit ECC words as sub-doubleword
 *     read-modify-write, corrupting shared memory.  Aligning to 8 makes every
 *     quad store write two whole 64-bit ECC words.
 *
 * Requires the assembler to target TC1.8 for st.dd (see AFLAGS -Wa,-mtc18 in
 * Make.defs / CMakeLists.txt).
 *
 * Q0 is a lower-context register pair group; together with the other scratch
 * registers (%d15, %a2/%a3/%a15) it needs no CSA context save.
 */

	.text
	.global	ARCH_LIBCFUN(memset)
	.type	ARCH_LIBCFUN(memset), %function

ARCH_LIBCFUN(memset):
	mov.aa	%a2, %a4		/* return value = original s        */
	jz	%d5, .Ldone		/* n == 0 -> nothing to do          */
	mov.aa	%a15, %a4		/* %a15 = working dst pointer        */

	/* Build a 16-byte fill pattern in the quad register Q0 (d0:d1:d2:d3). */

	and	%d0, %d4, 255		/* isolate the fill byte            */
	sh	%d1, %d0, 8
	or	%d0, %d0, %d1		/* pattern now 2 bytes wide         */
	sh	%d1, %d0, 16
	or	%d0, %d0, %d1		/* d0 = 4-byte fill word            */
	mov	%d1, %d0
	mov	%d2, %d0
	mov	%d3, %d0		/* Q0 = 16-byte fill                */

	mov	%d15, 16
	jlt.u	%d5, %d15, .Lbyte_setup	/* n < 16 -> byte fill              */

	/* Prologue: fill (8 - (dst & 7)) bytes to reach an 8-byte boundary. */

	mov.d	%d15, %a15
	and	%d15, %d15, 7
	jz	%d15, .Laligned
	rsub	%d15, %d15, 8		/* %d15 = 8 - misalign (1..7)       */
	sub	%d5, %d5, %d15		/* n -= prologue bytes              */
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lpre:
	st.b	[%a15+], %d4
	loop	%a3, .Lpre

.Laligned:
	/* Main loop: 32 bytes per iteration (two quad stores unrolled). */

	sh	%d15, %d5, -5		/* %d15 = n / 32                    */
	jz	%d15, .Lquad1
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lqloop:
	st.dd	[%a15+]16, %q0
	st.dd	[%a15+]16, %q0
	loop	%a3, .Lqloop

.Lquad1:
	/* Leftover single 16-byte block (n & 16), straight-line, no setup. */

	jz.t	%d5, 4, .Ltail
	st.dd	[%a15+]16, %q0

.Ltail:
	and	%d5, %d5, 15		/* remaining tail bytes             */
	jz	%d5, .Ldone

.Lbyte_setup:
	mov.a	%a3, %d5
	add.a	%a3, -1			/* loop counter = nbytes - 1        */
.Lbyte:
	st.b	[%a15+], %d4		/* store the original byte c        */
	loop	%a3, .Lbyte

.Ldone:
	ret
	.size	ARCH_LIBCFUN(memset), . - ARCH_LIBCFUN(memset)

#endif /* LIBC_BUILD_MEMSET */
