/****************************************************************************
 * libs/libc/machine/arm/armv7-a/arch_memset.S
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * SPDX-FileCopyrightText: 2013 The Android Open Source Project
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_MEMSET

	.arm
	.syntax unified
	.global ARCH_LIBCFUN(memset)
	.type	ARCH_LIBCFUN(memset), %function
ARCH_LIBCFUN(memset):
	.cfi_sections .debug_frame
	.cfi_startproc

#ifdef __ARM_NEON__
	mov         r3, r0
	// At this point only d0, d1 are going to be used below.
	vdup.8      q0, r1
	cmp         r2, #16
	blo         .L_set_less_than_16_unknown_align

.L_check_alignment:
	// Align destination to a double word to avoid the store crossing
	// a cache line boundary.
	ands        ip, r3, #7
	bne         .L_do_double_word_align

.L_double_word_aligned:
	// Duplicate since the less than 64 can use d2, d3.
	vmov        q1, q0
	subs        r2, #64
	blo         .L_set_less_than_64

	// Duplicate the copy value so that we can store 64 bytes at a time.
	vmov        q2, q0
	vmov        q3, q0

1:
	// Main loop stores 64 bytes at a time.
	subs        r2, #64
	vstmia      r3!, {d0 - d7}
	bge         1b

.L_set_less_than_64:
	// Restore r2 to the count of bytes left to set.
	add         r2, #64
	lsls        ip, r2, #27
	bcc         .L_set_less_than_32
	// Set 32 bytes.
	vstmia      r3!, {d0 - d3}

.L_set_less_than_32:
	bpl         .L_set_less_than_16
	// Set 16 bytes.
	vstmia      r3!, {d0, d1}

.L_set_less_than_16:
	// Less than 16 bytes to set.
	lsls        ip, r2, #29
	bcc         .L_set_less_than_8

	// Set 8 bytes.
	vstmia      r3!, {d0}

.L_set_less_than_8:
	bpl         .L_set_less_than_4
	// Set 4 bytes
	vst1.32     {d0[0]}, [r3]!

.L_set_less_than_4:
	lsls        ip, r2, #31
	it          ne
	strbne      r1, [r3], #1
	itt         cs
	strbcs      r1, [r3], #1
	strbcs      r1, [r3]
	bx          lr

.L_do_double_word_align:
	rsb         ip, ip, #8
	sub         r2, r2, ip

	// Do this comparison now, otherwise we'll need to save a
	// register to the stack since we've used all available
	// registers.
	cmp         ip, #4
	blo         1f

	// Need to do a four byte copy.
	movs        ip, ip, lsl #31
	it          mi
	strbmi      r1, [r3], #1
	itt         cs
	strbcs      r1, [r3], #1
	strbcs      r1, [r3], #1
	vst1.32     {d0[0]}, [r3]!
	b           .L_double_word_aligned

1:
	// No four byte copy.
	movs        ip, ip, lsl #31
	it          mi
	strbmi      r1, [r3], #1
	itt         cs
	strbcs      r1, [r3], #1
	strbcs      r1, [r3], #1
	b           .L_double_word_aligned

.L_set_less_than_16_unknown_align:
	// Set up to 15 bytes.
	movs        ip, r2, lsl #29
	bcc         1f
	vst1.8      {d0}, [r3]!
1:
	bge         2f
	vst1.32     {d0[0]}, [r3]!
2:
	movs        ip, r2, lsl #31
	it          mi
	strbmi      r1, [r3], #1
	itt         cs
	strbcs      r1, [r3], #1
	strbcs      r1, [r3], #1
	bx          lr
	.size ARCH_LIBCFUN(memset), . - ARCH_LIBCFUN(memset)

#else

	/* compute the offset to align the destination
	 * offset = (4-(src&3))&3 = -src & 3
	 */
	.save       {r0, r4-r7, lr}
	stmfd       sp!, {r0, r4-r7, lr}
	rsb         r3, r0, #0
	ands        r3, r3, #3
	cmp         r3, r2
	movhi       r3, r2

	/* splat r1 */
	mov         r1, r1, lsl #24
	orr         r1, r1, r1, lsr #8
	orr         r1, r1, r1, lsr #16

	movs        r12, r3, lsl #31
	strbcs      r1, [r0], #1    /* can't use strh (alignment unknown) */
	strbcs      r1, [r0], #1
	strbmi      r1, [r0], #1
	subs        r2, r2, r3
	popls       {r0, r4-r7, pc}    /* return */

	/* align the destination to a cache-line */
	mov         r12, r1
	mov         lr, r1
	mov         r4, r1
	mov         r5, r1
	mov         r6, r1
	mov         r7, r1

	rsb         r3, r0, #0
	ands        r3, r3, #0x1C
	beq         3f
	cmp         r3, r2
	andhi       r3, r2, #0x1C
	sub         r2, r2, r3

	/* conditionally writes 0 to 7 words (length in r3) */
	movs        r3, r3, lsl #28
	stmcs       r0!, {r1, lr}
	stmcs       r0!, {r1, lr}
	stmmi       r0!, {r1, lr}
	movs        r3, r3, lsl #2
	strcs       r1, [r0], #4

3:
	subs        r2, r2, #32
	mov         r3, r1
	bmi         2f
1:	subs        r2, r2, #32
	stmia       r0!, {r1,r3,r4,r5,r6,r7,r12,lr}
	bhs         1b
2:	add         r2, r2, #32

	/* conditionally stores 0 to 31 bytes */
	movs        r2, r2, lsl #28
	stmcs       r0!, {r1,r3,r12,lr}
	stmmi       r0!, {r1, lr}
	movs        r2, r2, lsl #2
	strcs       r1, [r0], #4
	strhmi      r1, [r0], #2
	movs        r2, r2, lsl #2
	strbcs      r1, [r0]
	ldmfd       sp!, {r0, r4-r7, pc}

#endif
	.cfi_endproc

#endif
