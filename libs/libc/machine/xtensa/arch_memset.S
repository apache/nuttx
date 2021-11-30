/****************************************************************************
 * libs/libc/machine/xtensa/arch_memset.S
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

#include "xtensa_asm.h"

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* void *memset (void *dst, int c, size_t length)

   The algorithm is as follows:

   Create a word with c in all byte positions.

   If the destination is aligned, set 16B chunks with a loop, and then
   finish up with 8B, 4B, 2B, and 1B stores conditional on the length.

   If the destination is unaligned, align it by conditionally
   setting 1B and/or 2B and then go to aligned case.

   This code tries to use fall-through branches for the common
   case of an aligned destination (except for the branches to
   the alignment labels).  */


/* Byte-by-byte set.  */

	.section .text
	.begin schedule
	.literal_position

	.local	.Lbyteset
	.local	.Ldst1mod2
	.local	.Ldst2mod4

	.align	4
	.global	memset
	.type	memset, @function
memset:
  ENTRY(16)
	/* a2 = dst, a3 = c, a4 = length */

	/* Duplicate character into all bytes of word.  */
	extui	a3, a3, 0, 8
	slli	a7, a3, 8
	or	a3, a3, a7
	slli	a7, a3, 16
	or	a3, a3, a7

	mov	a5, a2		// copy dst so that a2 is return value

	/* Check if dst is unaligned.  */
	bbsi.l	a2, 0, .Ldst1mod2
	bbsi.l	a2, 1, .Ldst2mod4
	j	.Ldstaligned

.Ldst1mod2: // dst is only byte aligned

	/* Do short sizes byte-by-byte.  */
	bltui	a4, 8, .Lbyteset

	/* Set 1 byte.  */
	s8i	a3, a5, 0
	addi	a5, a5, 1
	addi	a4, a4, -1

	/* Now retest if dst is aligned.  */
	bbci.l	a5, 1, .Ldstaligned

.Ldst2mod4: // dst has 16-bit alignment

	/* Do short sizes byte-by-byte.  */
	bltui	a4, 8, .Lbyteset

	/* Set 2 bytes.  */
	s16i	a3, a5, 0
	addi	a5, a5, 2
	addi	a4, a4, -2

	/* dst is now aligned; fall through to main algorithm */

.Ldstaligned:

	/* Get number of loop iterations with 16B per iteration.  */
	srli	a7, a4, 4

	/* Destination is word-aligned.  */
#if XCHAL_HAVE_LOOPS
	loopnez	a7, 2f
#else
	beqz	a7, 2f
	slli	a6, a7, 4
	add	a6, a6, a5	// a6 = end of last 16B chunk
#endif
	/* Set 16 bytes per iteration.  */
1:	s32i	a3, a5, 0
	s32i	a3, a5, 4
	s32i	a3, a5, 8
	s32i	a3, a5, 12
	addi	a5, a5, 16
#if !XCHAL_HAVE_LOOPS
	bltu	a5, a6, 1b
#endif

	/* Set any leftover pieces smaller than 16B.  */
2:	bbci.l	a4, 3, 3f

	/* Set 8 bytes.  */
	s32i	a3, a5, 0
	s32i	a3, a5, 4
	addi	a5, a5, 8

3:	bbci.l	a4, 2, 4f

	/* Set 4 bytes.  */
	s32i	a3, a5, 0
	addi	a5, a5, 4

4:	bbci.l	a4, 1, 5f

	/* Set 2 bytes.  */
	s16i	a3, a5, 0
	addi	a5, a5, 2

5:	bbci.l	a4, 0, 6f

	/* Set 1 byte.  */
	s8i	a3, a5, 0
6:	RET(16)


	// .align	XCHAL_INST_FETCH_WIDTH
__memset_aux:

	/* Skip bytes to get proper alignment for three-byte loop */
// .skip XCHAL_INST_FETCH_WIDTH - 3

.Lbyteset:
#if XCHAL_HAVE_LOOPS
	loopnez	a4, 2f
#else
	beqz	a4, 2f
	add	a6, a5, a4	// a6 = ending address
#endif
1:	s8i	a3, a5, 0
	addi	a5, a5, 1
#if !XCHAL_HAVE_LOOPS
	bltu	a5, a6, 1b
#endif
2:	RET(16)

	.end schedule

	.size	memset, . - memset
