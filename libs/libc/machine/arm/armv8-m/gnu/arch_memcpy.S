/****************************************************************************
 * libs/libc/machine/arm/armv8-m/gnu/arch_memcpy.S
 *
 * Copyright (c) 2013 ARM Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ARM LTD ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ARM LTD BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARM_FEATURE_MVE
/* This memcpy routine is optimised for Cortex-M3/M4 cores with/without
   unaligned access.

   If compiled with GCC, this file should be enclosed within following
   pre-processing check:
   if defined (__ARM_ARCH_7M__) || defined (__ARM_ARCH_7EM__)

   Prototype: void *memcpy (void *dst, const void *src, size_t count);

   The job will be done in 5 steps.
   Step 1: Align src/dest pointers, copy mis-aligned if fail to align both
   Step 2: Repeatedly copy big block size of __OPT_BIG_BLOCK_SIZE
   Step 3: Repeatedly copy big block size of __OPT_MID_BLOCK_SIZE
   Step 4: Copy word by word
   Step 5: Copy byte-to-byte

   Tunable options:
     __OPT_BIG_BLOCK_SIZE: Size of big block in words.  Default to 64.
     __OPT_MID_BLOCK_SIZE: Size of big block in words.  Default to 16.
 */
#ifndef __OPT_BIG_BLOCK_SIZE
#define __OPT_BIG_BLOCK_SIZE (4 * 16)
#endif

#ifndef __OPT_MID_BLOCK_SIZE
#define __OPT_MID_BLOCK_SIZE (4 * 4)
#endif

#if __OPT_BIG_BLOCK_SIZE == 16
#define BEGIN_UNROLL_BIG_BLOCK \
  .irp offset, 0,4,8,12
#elif __OPT_BIG_BLOCK_SIZE == 32
#define BEGIN_UNROLL_BIG_BLOCK \
  .irp offset, 0,4,8,12,16,20,24,28
#elif __OPT_BIG_BLOCK_SIZE == 64
#define BEGIN_UNROLL_BIG_BLOCK \
  .irp offset, 0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60
#else
#error "Illegal __OPT_BIG_BLOCK_SIZE"
#endif

#if __OPT_MID_BLOCK_SIZE == 8
#define BEGIN_UNROLL_MID_BLOCK \
  .irp offset, 0,4
#elif __OPT_MID_BLOCK_SIZE == 16
#define BEGIN_UNROLL_MID_BLOCK \
  .irp offset, 0,4,8,12
#else
#error "Illegal __OPT_MID_BLOCK_SIZE"
#endif

#define END_UNROLL .endr
#endif

	.syntax unified
	.text
	.align	2
	.global	memcpy
	.thumb
	.thumb_func
	.type	memcpy, %function
memcpy:
	@ r0: dst
	@ r1: src
	@ r2: len
#ifdef __ARM_FEATURE_MVE
	mov	r3, lr
	wlstp.8	lr, r2, 2f
	mov	r2, r0
1:
	vldrb.8	q0, [r1], #16
	vstrb.8	q0, [r2], #16
	letp	lr, 1b
2:
	bx	r3
#else
#ifdef __ARM_FEATURE_UNALIGNED
	/* In case of UNALIGNED access supported, ip is not used in
	   function body.  */
	mov	ip, r0
#else
	push	{r0}
#endif
	orr	r3, r1, r0
	ands	r3, r3, #3
	bne	.Lmisaligned_copy

.Lbig_block:
	subs	r2, __OPT_BIG_BLOCK_SIZE
	blo	.Lmid_block

	/* Kernel loop for big block copy */
	.align 2
.Lbig_block_loop:
	BEGIN_UNROLL_BIG_BLOCK
#ifdef __ARM_ARCH_7EM__
	ldr	r3, [r1], #4
	str	r3, [r0], #4
	END_UNROLL
#else /* __ARM_ARCH_7M__ */
	ldr	r3, [r1, \offset]
	str	r3, [r0, \offset]
	END_UNROLL
	adds	r0, __OPT_BIG_BLOCK_SIZE
	adds	r1, __OPT_BIG_BLOCK_SIZE
#endif
	subs	r2, __OPT_BIG_BLOCK_SIZE
	bhs .Lbig_block_loop

.Lmid_block:
	adds	r2, __OPT_BIG_BLOCK_SIZE - __OPT_MID_BLOCK_SIZE
	blo	.Lcopy_word_by_word

	/* Kernel loop for mid-block copy */
	.align 2
.Lmid_block_loop:
	BEGIN_UNROLL_MID_BLOCK
#ifdef __ARM_ARCH_7EM__
	ldr	r3, [r1], #4
	str	r3, [r0], #4
	END_UNROLL
#else /* __ARM_ARCH_7M__ */
	ldr	r3, [r1, \offset]
	str	r3, [r0, \offset]
	END_UNROLL
	adds    r0, __OPT_MID_BLOCK_SIZE
	adds    r1, __OPT_MID_BLOCK_SIZE
#endif
	subs	r2, __OPT_MID_BLOCK_SIZE
	bhs	.Lmid_block_loop

.Lcopy_word_by_word:
	adds	r2, __OPT_MID_BLOCK_SIZE - 4
	blo	.Lcopy_less_than_4

	/* Kernel loop for small block copy */
	.align 2
.Lcopy_word_by_word_loop:
	ldr	r3, [r1], #4
	str	r3, [r0], #4
	subs	r2, #4
	bhs	.Lcopy_word_by_word_loop

.Lcopy_less_than_4:
	adds	r2, #4
	beq	.Ldone

	lsls	r2, r2, #31
	itt ne
	ldrbne  r3, [r1], #1
	strbne  r3, [r0], #1

	bcc	.Ldone
#ifdef __ARM_FEATURE_UNALIGNED
	ldrh	r3, [r1]
	strh	r3, [r0]
#else
	ldrb	r3, [r1]
	strb	r3, [r0]
	ldrb	r3, [r1, #1]
	strb	r3, [r0, #1]
#endif /* __ARM_FEATURE_UNALIGNED */

.Ldone:
#ifdef __ARM_FEATURE_UNALIGNED
	mov	r0, ip
#else
	pop	{r0}
#endif
	bx	lr

	.align 2
.Lmisaligned_copy:
#ifdef __ARM_FEATURE_UNALIGNED
	/* Define label DST_ALIGNED to BIG_BLOCK.  It will go to aligned copy
	   once destination is adjusted to aligned.  */
#define Ldst_aligned Lbig_block

	/* Copy word by word using LDR when alignment can be done in hardware,
	i.e., SCTLR.A is set, supporting unaligned access in LDR and STR.  */

	cmp	r2, #8
	blo	.Lbyte_copy

	/* if src is aligned, just go to the big block loop.  */
	lsls	r3, r1, #30
	beq	.Ldst_aligned
#else
	/* if len < 12, misalignment adjustment has more overhead than
	just byte-to-byte copy.  Also, len must >=8 to guarantee code
	afterward work correctly.  */
	cmp	r2, #12
	blo	.Lbyte_copy
#endif /* __ARM_FEATURE_UNALIGNED */

	/* Align dst only, not trying to align src.  That is the because
	handling of aligned src and misaligned dst need more overhead than
	otherwise.  By doing this the worst case is when initial src is aligned,
	additional up to 4 byte additional copy will executed, which is
	acceptable.  */

	ands	r3, r0, #3
	beq	.Ldst_aligned

	rsb	r3, #4
	subs	r2, r3

	lsls    r3, r3, #31
	itt ne
	ldrbne  r3, [r1], #1
	strbne  r3, [r0], #1

	bcc .Ldst_aligned

#ifdef __ARM_FEATURE_UNALIGNED
	ldrh    r3, [r1], #2
	strh    r3, [r0], #2
	b	.Ldst_aligned
#else
	ldrb    r3, [r1], #1
	strb    r3, [r0], #1
	ldrb    r3, [r1], #1
	strb    r3, [r0], #1
	/* Now that dst is aligned */
.Ldst_aligned:
	/* if r1 is aligned now, it means r0/r1 has the same misalignment,
	and they are both aligned now.  Go aligned copy.  */
	ands	r3, r1, #3
	beq	.Lbig_block

	/* dst is aligned, but src isn't.  Misaligned copy.  */

	push	{r4, r5}
	subs	r2, #4

	/* Backward r1 by misaligned bytes, to make r1 aligned.
	Since we need to restore r1 to unaligned address after the loop,
	we need keep the offset bytes to ip and sub it from r1 afterward.  */
	subs	r1, r3
	rsb	ip, r3, #4

	/* Pre-load on word */
	ldr	r4, [r1], #4

	cmp	r3, #2
	beq	.Lmisaligned_copy_2_2
	cmp	r3, #3
	beq	.Lmisaligned_copy_3_1

	.macro mis_src_copy shift
1:
#ifdef __ARM_BIG_ENDIAN
	lsls	r4, r4, \shift
#else
	lsrs	r4, r4, \shift
#endif
	ldr	r3, [r1], #4
#ifdef __ARM_BIG_ENDIAN
	lsrs	r5, r3, 32-\shift
#else
	lsls	r5, r3, 32-\shift
#endif
	orr	r4, r4, r5
	str	r4, [r0], #4
	mov	r4, r3
	subs	r2, #4
	bhs	1b
	.endm

.Lmisaligned_copy_1_3:
	mis_src_copy shift=8
	b	.Lsrc_misaligned_tail

.Lmisaligned_copy_3_1:
	mis_src_copy shift=24
	b	.Lsrc_misaligned_tail

.Lmisaligned_copy_2_2:
	/* For 2_2 misalignment, ldr is still faster than 2 x ldrh.  */
	mis_src_copy shift=16

.Lsrc_misaligned_tail:
	adds	r2, #4
	subs	r1, ip
	pop	{r4, r5}

#endif /* __ARM_FEATURE_UNALIGNED */

.Lbyte_copy:
	subs	r2, #4
	blo	.Lcopy_less_than_4

.Lbyte_copy_loop:
	subs    r2, #1
	ldrb    r3, [r1], #1
	strb    r3, [r0], #1
	bhs	.Lbyte_copy_loop

	ldrb	r3, [r1]
	strb	r3, [r0]
	ldrb	r3, [r1, #1]
	strb	r3, [r0, #1]
	ldrb	r3, [r1, #2]
	strb	r3, [r0, #2]

#ifdef __ARM_FEATURE_UNALIGNED
	mov	r0, ip
#else
	pop	{r0}
#endif
	bx	lr
#endif
	.size	memcpy, .-memcpy
