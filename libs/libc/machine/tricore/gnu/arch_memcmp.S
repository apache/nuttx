/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_memcmp.S
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

#ifdef LIBC_BUILD_MEMCMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* int memcmp(const void *s1, const void *s2, size_t n)
 *
 * TriCore EABI:
 *   s1 -> %a4   s2 -> %a5   n -> %d4   return -> %d2
 *
 * Strategy (word-at-a-time, matches the BSD word compare bar):
 *   - When n >= 8 and s1/s2 share 4-byte congruence: a byte prologue brings
 *     both to a 4-byte boundary, then compare 4 bytes per iteration with
 *     ld.w.  All ld.w accesses are 4-byte aligned so they never cross a page.
 *     On a word mismatch, back up 4 bytes and resolve byte-wise.
 *   - Otherwise (n < 8 or different congruence) compare byte by byte.
 *   - %d4 always holds the number of remaining bytes to compare.
 *   - Return value is the unsigned-char difference at the first mismatch (or
 *     0 if all n bytes are equal).
 *
 * Only scratch registers (%d0/%d1/%d2/%d15, %a3) are used, so no CSA context
 * save is required.
 */

	.text
	.global	ARCH_LIBCFUN(memcmp)
	.type	ARCH_LIBCFUN(memcmp), %function

ARCH_LIBCFUN(memcmp):
	jz	%d4, .Lret0		/* n == 0 -> equal                  */

	mov	%d15, 8
	jlt.u	%d4, %d15, .Lbyte	/* n < 8 -> byte compare            */

	/* Same 8-byte congruence?  (s1 ^ s2) & 7 == 0 */

	mov.d	%d0, %a4
	mov.d	%d1, %a5
	xor	%d0, %d0, %d1
	and	%d0, %d0, 7
	jnz	%d0, .Lbyte		/* different alignment -> byte compare */

	/* Byte prologue to reach an 8-byte boundary (n >= 8, so safe). */

	mov.d	%d0, %a4
	and	%d0, %d0, 7
	jz	%d0, .Lword
	rsub	%d0, %d0, 8		/* prologue bytes = 1..7            */
	mov.a	%a3, %d0
	add.a	%a3, -1
.Lpre:
	ld.bu	%d15, [%a4+]
	ld.bu	%d1, [%a5+]
	jne	%d15, %d1, .Lretbyte1	/* differ -> return d15 - d1        */
	add	%d4, -1
	loop	%a3, .Lpre

.Lword:
	/* Main loop: 8 bytes/iter via ld.d (64-bit single transfer, 8-aligned;
	 * flash aperture rejects 16B burst, so ld.d not ld.dd).
	 */

	sh	%d15, %d4, -3		/* d15 = dword count = n >> 3       */
	jz	%d15, .Lbyte		/* < 8 bytes left -> byte compare   */
	and	%d4, %d4, 7		/* d4 = tail bytes after whole dwords */
	mov.a	%a3, %d15
	add.a	%a3, -1
.Lwloop:
	ld.d	%e0, [%a4+]8		/* d0:d1 = 8 bytes from s1          */
	ld.d	%e2, [%a5+]8		/* d2:d3 = 8 bytes from s2          */
	jne	%d0, %d2, .Lworddiff	/* low word differs                 */
	jne	%d1, %d3, .Lworddiff	/* high word differs                */
	loop	%a3, .Lwloop
	j	.Lbyte			/* all dwords equal -> compare tail */

.Lworddiff:
	/* Dwords differ within 8 bytes; back up and resolve byte-wise.
	 * The first differing byte is guaranteed in these 8 bytes, so the
	 * byte loop exits via .Lretbyte1 before touching the tail.
	 */

	lea	%a4, [%a4]-8
	lea	%a5, [%a5]-8
	mov	%d4, 8
	/* fall through to byte compare */

.Lbyte:
	jz	%d4, .Lret0
	mov.a	%a3, %d4
	add.a	%a3, -1
.Lbyteloop:
	ld.bu	%d15, [%a4+]
	ld.bu	%d1, [%a5+]
	jne	%d15, %d1, .Lretbyte1
	loop	%a3, .Lbyteloop

.Lret0:
	mov	%d2, 0
	ret

.Lretbyte1:
	sub	%d2, %d15, %d1		/* return (unsigned char) diff      */
	ret

	.size	ARCH_LIBCFUN(memcmp), . - ARCH_LIBCFUN(memcmp)

#endif /* LIBC_BUILD_MEMCMP */
