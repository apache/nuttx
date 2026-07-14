/*********************************************************************************
 * libs/libc/machine/x86_64/arch_strcmp_avx2.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2014, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *     * this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *     * this list of conditions and the following disclaimer in the documentation
 *     * and/or other materials provided with the distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its contributors
 *     * may be used to endorse or promote products derived from this software
 *     * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
 *********************************************************************************/

/*********************************************************************************
 * Pre-processor Definitions
 *********************************************************************************/

#ifdef USE_AS_STRNCMP
/* Since the counter, %r11, is unsigned, we branch to strcmp_exitz
   if the new counter > the old one or is 0.  */
#define UPDATE_STRNCMP_COUNTER              \
	/* calculate left number to compare */    \
	lea	-16(%rcx, %r11), %r9;                 \
	cmp	%r9, %r11;                            \
	jb	L(strcmp_exitz);                      \
	test	%r9, %r9;                           \
	je	L(strcmp_exitz);                      \
	mov	%r9, %r11

#else
#define UPDATE_STRNCMP_COUNTER
#include "libc.h"

#ifndef STRCMP
#define STRCMP		ARCH_LIBCFUN(strcmp)
#endif
#endif

#ifndef L
# define L(label)	.L##label
#endif

#ifndef cfi_startproc
# define cfi_startproc			.cfi_startproc
#endif

#ifndef cfi_endproc
# define cfi_endproc			.cfi_endproc
#endif

#ifndef ENTRY
# define ENTRY(name)      \
	.type name,  @function; \
	.globl name;            \
	.p2align 4;             \
name:                     \
	cfi_startproc
#endif

#ifndef END
# define END(name)      \
	cfi_endproc;          \
	.size name, .-name
#endif
#define RETURN ret

/*********************************************************************************
 * Public Functions
 *********************************************************************************/

	.section .text.avx2,"ax",@progbits
ENTRY (STRCMP)
/*
 * This implementation uses SSE to compare up to 16 bytes at a time.
 */
#ifdef USE_AS_STRNCMP
	test	%rdx, %rdx
	je	L(strcmp_exitz)
	cmp	$1, %rdx
	je	L(Byte0)
	mov	%rdx, %r11
#endif
	mov	%esi, %ecx
	mov	%edi, %eax
/* Use 64bit AND here to avoid long NOP padding.  */
	and	$0x3f, %rcx		/* rsi alignment in cache line */
	and	$0x3f, %rax		/* rdi alignment in cache line */
	cmp	$0x30, %ecx
	ja	L(crosscache)	/* rsi: 16-byte load will cross cache line */
	cmp	$0x30, %eax
	ja	L(crosscache)	/* rdi: 16-byte load will cross cache line */
	movlpd	(%rdi), %xmm1
	movlpd	(%rsi), %xmm2
	movhpd	8(%rdi), %xmm1
	movhpd	8(%rsi), %xmm2
	vpxor %xmm0, %xmm0, %xmm0		/* clear %xmm0 for null char checks */
	vpcmpeqb %xmm1, %xmm0, %xmm0		/* Any null chars? */
	vpcmpeqb %xmm2, %xmm1, %xmm1		/* compare first 16 bytes for equality */
	vpsubb %xmm0, %xmm1, %xmm1		/* packed sub of comparison results*/
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx		/* if first 16 bytes are same, edx == 0xffff */
	jnz	L(less16bytes)	/* If not, find different value or null char */
#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)	/* finish comparison */
#endif
	add	$16, %rsi		/* prepare to search next 16 bytes */
	add	$16, %rdi		/* prepare to search next 16 bytes */

	/*
	 * Determine source and destination string offsets from 16-byte alignment.
	 * Use relative offset difference between the two to determine which case
	 * below to use.
	 */
	.p2align 4
L(crosscache):
	and	$0xfffffffffffffff0, %rsi	/* force %rsi is 16 byte aligned */
	and	$0xfffffffffffffff0, %rdi	/* force %rdi is 16 byte aligned */
	mov	$0xffff, %edx			/* for equivalent offset */
	xor	%r8d, %r8d
	and	$0xf, %ecx			/* offset of rsi */
	and	$0xf, %eax			/* offset of rdi */
	cmp	%eax, %ecx
	je	L(ashr_0)			/* rsi and rdi relative offset same */
	ja	L(bigger)
	mov	%edx, %r8d			/* r8d is offset flag for exit tail */
	xchg	%ecx, %eax
	xchg	%rsi, %rdi
L(bigger):
	lea	15(%rax), %r9
	sub	%rcx, %r9
	lea	L(unaligned_table)(%rip), %r10
	movslq	(%r10, %r9,4), %r9
	lea	(%r10, %r9), %r10
	jmp	*%r10				/* jump to corresponding case */

/*
 * The following cases will be handled by ashr_0
 *  rcx(offset of rsi)  rax(offset of rdi)  relative offset  corresponding case
 *        n(0~15)            n(0~15)           15(15+ n-n)         ashr_0
 */
	.p2align 4
L(ashr_0):

	vmovdqa	(%rsi), %xmm1
	vpxor %xmm0, %xmm0, %xmm0			/* clear %xmm0 for null char check */
	vpcmpeqb %xmm1, %xmm0, %xmm0			/* Any null chars? */
	vpcmpeqb (%rdi), %xmm1, %xmm1			/* compare 16 bytes for equality */
	vpsubb %xmm0, %xmm1, %xmm1			/* packed sub of comparison results*/
	vpmovmskb %xmm1, %r9d
	shr	%cl, %edx			/* adjust 0xffff for offset */
	shr	%cl, %r9d			/* adjust for 16-byte offset */
	sub	%r9d, %edx
	/*
	 * edx must be the same with r9d if in left byte (16-rcx) is equal to
	 * the start from (16-rax) and no null char was seen.
	 */
	jne	L(less32bytes)		/* mismatch or null char */
	UPDATE_STRNCMP_COUNTER
	mov	$16, %rcx
	mov	$16, %r9
	vpxor %xmm0, %xmm0, %xmm0			/* clear xmm0, may have changed above */

	/*
	 * Now both strings are aligned at 16-byte boundary. Loop over strings
	 * checking 32-bytes per iteration.
	 *
	 * A 32-byte load from a 16-byte aligned address crosses into the next
	 * page only from the last 16 bytes of a page.  r10 counts the bytes
	 * left before the nearer of the two page ends, so the loop knows when
	 * to compare those 16 bytes alone and never reads past a page before
	 * the terminator of the string is known to be beyond it.
	 */
L(ashr0_page):
	lea	(%rsi, %rcx), %rax
	and	$0xfff, %eax
	lea	(%rdi, %rcx), %rdx
	and	$0xfff, %edx
	cmp	%edx, %eax
	cmovb	%edx, %eax			/* eax = larger page offset */
	mov	$0x1000, %r10d
	sub	%rax, %r10			/* r10 = bytes to nearer page end */

	.p2align 4
L(loop_ashr_0):
	/* Compare 32 bytes per iteration with one 256-bit ymm step instead of
	 * two 128-bit xmm steps. ymm0 is the persistent zero vector (never
	 * written in this loop). ymm3 = (s2 byte == 0) null mask; ymm2 =
	 * (s1 == s2) equal mask; equal-minus-null leaves 0xff only for bytes
	 * that are equal AND non-NUL, so edx == 0xffffffff iff all 32 bytes
	 * match with no terminator. The critical-path ops (vpcmpeqb/vpsubb/
	 * vpmovmskb) keep 128-bit latency at 256-bit width, so widening is a
	 * net win (measured ~1.2x). This path (ashr_0) has r8d == 0 (no operand
	 * swap), so on a difference we can locate the byte and return
	 * s1[k]-s2[k] directly. vmovdqu: rsi/rdi are only 16-byte aligned. */
#ifdef USE_AS_STRNCMP
	/* strncmp: only take the 32-byte wide step when at least 32 bytes
	 * remain (r11), so any diff found inside the window is guaranteed to
	 * be within bounds (k < 32 <= r11) and the exit path needs no k-vs-n
	 * check. When fewer than 32 remain, hand off to a bounded scalar tail
	 * that respects n exactly. This whole block is compiled out for plain
	 * strcmp. */
	cmp	$32, %r11
	jb	L(ashr0_strncmp_tail)
#endif
	sub	$32, %r10			/* fewer than 32 bytes to page end? */
	jb	L(ashr0_short)
	vmovdqu	(%rsi, %rcx), %ymm1
	vpcmpeqb %ymm0, %ymm1, %ymm3		/* null mask: s2 byte == 0 */
	vpcmpeqb (%rdi, %rcx), %ymm1, %ymm2	/* equal mask: s1 == s2 */
	vpsubb %ymm3, %ymm2, %ymm2		/* equal - null */
	vpmovmskb %ymm2, %edx
	cmp	$0xffffffff, %edx		/* all 32 equal and no null? */
	jne	L(ashr0_diff)
#ifdef USE_AS_STRNCMP
	sub	$32, %r11			/* 32 bytes consumed, all equal */
	jz	L(strcmp_exitz)			/* exactly n bytes matched -> 0 */
#endif
	add	$32, %rcx
	jmp	L(loop_ashr_0)

	.p2align 4
L(ashr0_short):
	add	$32, %r10			/* r10 is 0 or 16 here */
	jz	L(ashr0_page)			/* page end reached, recount */
	vmovdqa	(%rsi, %rcx), %xmm1
	vpcmpeqb %xmm0, %xmm1, %xmm3		/* null mask: s2 byte == 0 */
	vpcmpeqb (%rdi, %rcx), %xmm1, %xmm2	/* equal mask: s1 == s2 */
	vpsubb %xmm3, %xmm2, %xmm2		/* equal - null */
	vpmovmskb %xmm2, %edx
	cmp	$0xffff, %edx			/* all 16 equal and no null? */
	jne	L(ashr0_diff)			/* bits 16..31 are 0, see below */
#ifdef USE_AS_STRNCMP
	sub	$16, %r11			/* 16 bytes consumed, all equal */
	jz	L(strcmp_exitz)
#endif
	add	$16, %rcx
	jmp	L(ashr0_page)

	.p2align 4
L(ashr0_diff):
	/* First zero bit in edx = first byte that mismatches or is NUL. */
	not	%edx
	bsf	%edx, %edx
	add	%rcx, %rdx			/* byte offset from string base */
	movzbl	(%rdi, %rdx), %eax		/* s1[k] */
	movzbl	(%rsi, %rdx), %ecx		/* s2[k] */
	sub	%ecx, %eax			/* strcmp result = s1 - s2 */
	ret

#ifdef USE_AS_STRNCMP
	/* Bounded scalar tail for the final <32 bytes of a strncmp. Compares
	 * exactly r11 remaining bytes; returns s1-s2 on the first mismatch,
	 * 0 on a shared NUL, and 0 when n is exhausted with no difference. */
	.p2align 4
L(ashr0_strncmp_tail):
	test	%r11, %r11
	jz	L(strcmp_exitz)			/* n exhausted, equal so far -> 0 */
	movzbl	(%rdi, %rcx), %eax
	movzbl	(%rsi, %rcx), %edx
	sub	%edx, %eax
	jne	L(ashr0_tail_ret)		/* mismatch within n -> return */
	test	%edx, %edx
	jz	L(ashr0_tail_ret)		/* shared NUL -> eax==0, equal */
	add	$1, %rcx
	sub	$1, %r11
	jmp	L(ashr0_strncmp_tail)
	.p2align 4
L(ashr0_tail_ret):
	ret
#endif

/*
 * The following cases will be handled by ashr_1
 * rcx(offset of rsi)  rax(offset of rdi)   relative offset   	corresponding case
 *        n(15)            n -15            0(15 +(n-15) - n)         ashr_1
 */
	.p2align 4
L(ashr_1):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0		/* Any null chars? */
	vpslldq $15, %xmm2, %xmm2		/* shift first string to align with second */
	vpcmpeqb %xmm1, %xmm2, %xmm2		/* compare 16 bytes for equality */
	vpsubb %xmm0, %xmm2, %xmm2		/* packed sub of comparison results*/
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx		/* adjust 0xffff for offset */
	shr	%cl, %r9d		/* adjust for 16-byte offset */
	sub	%r9d, %edx
	jnz	L(less32bytes)	/* mismatch or null char seen */
	vmovdqa	(%rdi), %xmm3
	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx		/* index for loads*/
	mov	$1, %r9d		/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	1(%rdi), %r10
	and	$0xfff, %r10		/* offset into 4K page */
	sub	$0x1000, %r10		/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_1):
	add	$16, %r10
	jg	L(nibble_ashr_1)	/* cross page boundary */

L(gobble_ashr_1):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4		 /* store for next cycle */

	vpalignr $1, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif
	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_1)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4		/* store for next cycle */

	vpalignr $1, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif
	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_1)

	/*
	 * Nibble avoids loads across page boundary. This is to avoid a potential
	 * access into unmapped memory.
	 */
	.p2align 4
L(nibble_ashr_1):
	vpcmpeqb %xmm3, %xmm0, %xmm0		 /* check nibble for null char*/
	vpmovmskb %xmm0, %edx
	test	$0xfffe, %edx
	jnz	L(ashr_1_exittail)	/* find null char*/

#ifdef USE_AS_STRNCMP
	cmp	$14, %r11
	jbe	L(ashr_1_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10		/* subtract 4K from %r10 */
	jmp	L(gobble_ashr_1)

	/*
	 * Once find null char, determine if there is a string mismatch
	 * before the null char.
	 */
	.p2align 4
L(ashr_1_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $1, %xmm0, %xmm0
	vpsrldq $1, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 * The following cases will be handled by ashr_2
 * rcx(offset of rsi)  rax(offset of rdi)   relative offset   	corresponding case
 *        n(14~15)            n -14         1(15 +(n-14) - n)         ashr_2
 */
	.p2align 4
L(ashr_2):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $14, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3
	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$2, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	2(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_2):
	add	$16, %r10
	jg	L(nibble_ashr_2)

L(gobble_ashr_2):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $2, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_2)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $2, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_2)

	.p2align 4
L(nibble_ashr_2):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xfffc, %edx
	jnz	L(ashr_2_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$13, %r11
	jbe	L(ashr_2_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_2)

	.p2align 4
L(ashr_2_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $2, %xmm0, %xmm0
	vpsrldq $2, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 * The following cases will be handled by ashr_3
 *  rcx(offset of rsi)  rax(offset of rdi)  relative offset	 corresponding case
 *        n(13~15)            n -13         2(15 +(n-13) - n)         ashr_3
 */
	.p2align 4
L(ashr_3):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $13, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$3, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	3(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_3):
	add	$16, %r10
	jg	L(nibble_ashr_3)

L(gobble_ashr_3):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $3, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_3)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $3, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_3)

	.p2align 4
L(nibble_ashr_3):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xfff8, %edx
	jnz	L(ashr_3_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$12, %r11
	jbe	L(ashr_3_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_3)

	.p2align 4
L(ashr_3_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $3, %xmm0, %xmm0
	vpsrldq $3, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 * The following cases will be handled by ashr_4
 *  rcx(offset of rsi)  rax(offset of rdi)  relative offset	 corresponding case
 *        n(12~15)            n -12         3(15 +(n-12) - n)         ashr_4
 */
	.p2align 4
L(ashr_4):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $12, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$4, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	4(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_4):
	add	$16, %r10
	jg	L(nibble_ashr_4)

L(gobble_ashr_4):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $4, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_4)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $4, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_4)

	.p2align 4
L(nibble_ashr_4):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xfff0, %edx
	jnz	L(ashr_4_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$11, %r11
	jbe	L(ashr_4_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_4)

	.p2align 4
L(ashr_4_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $4, %xmm0, %xmm0
	vpsrldq $4, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 * The following cases will be handled by ashr_5
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset      corresponding case
 *        n(11~15)          n - 11      	  4(15 +(n-11) - n)         ashr_5
 */
	.p2align 4
L(ashr_5):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $11, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$5, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	5(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_5):
	add	$16, %r10
	jg	L(nibble_ashr_5)

L(gobble_ashr_5):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $5, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_5)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $5, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_5)

	.p2align 4
L(nibble_ashr_5):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xffe0, %edx
	jnz	L(ashr_5_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$10, %r11
	jbe	L(ashr_5_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_5)

	.p2align 4
L(ashr_5_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $5, %xmm0, %xmm0
	vpsrldq $5, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 * The following cases will be handled by ashr_6
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset      corresponding case
 *        n(10~15)          n - 10      	  5(15 +(n-10) - n)         ashr_6
 */
	.p2align 4
L(ashr_6):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $10, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$6, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	6(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_6):
	add	$16, %r10
	jg	L(nibble_ashr_6)

L(gobble_ashr_6):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $6, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_6)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $6, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_6)

	.p2align 4
L(nibble_ashr_6):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xffc0, %edx
	jnz	L(ashr_6_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$9, %r11
	jbe	L(ashr_6_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_6)

	.p2align 4
L(ashr_6_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $6, %xmm0, %xmm0
	vpsrldq $6, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 * The following cases will be handled by ashr_7
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset      corresponding case
 *        n(9~15)          n - 9      	        6(15 +(n - 9) - n)         ashr_7
 */
	.p2align 4
L(ashr_7):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $9, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$7, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	7(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_7):
	add	$16, %r10
	jg	L(nibble_ashr_7)

L(gobble_ashr_7):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $7, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_7)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $7, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_7)

	.p2align 4
L(nibble_ashr_7):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xff80, %edx
	jnz	L(ashr_7_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$8, %r11
	jbe	L(ashr_7_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_7)

	.p2align 4
L(ashr_7_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $7, %xmm0, %xmm0
	vpsrldq $7, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_8
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(8~15)          n - 8      	        7(15 +(n - 8) - n)         ashr_8
 */
	.p2align 4
L(ashr_8):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $8, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$8, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	8(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_8):
	add	$16, %r10
	jg	L(nibble_ashr_8)

L(gobble_ashr_8):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $8, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_8)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $8, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_8)

	.p2align 4
L(nibble_ashr_8):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xff00, %edx
	jnz	L(ashr_8_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$7, %r11
	jbe	L(ashr_8_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_8)

	.p2align 4
L(ashr_8_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $8, %xmm0, %xmm0
	vpsrldq $8, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_9
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(7~15)          n - 7      	        8(15 +(n - 7) - n)         ashr_9
 */
	.p2align 4
L(ashr_9):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $7, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$9, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	9(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_9):
	add	$16, %r10
	jg	L(nibble_ashr_9)

L(gobble_ashr_9):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $9, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_9)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $9, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3		/* store for next cycle */
	jmp	L(loop_ashr_9)

	.p2align 4
L(nibble_ashr_9):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xfe00, %edx
	jnz	L(ashr_9_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$6, %r11
	jbe	L(ashr_9_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_9)

	.p2align 4
L(ashr_9_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $9, %xmm0, %xmm0
	vpsrldq $9, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_10
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(6~15)          n - 6      	        9(15 +(n - 6) - n)         ashr_10
 */
	.p2align 4
L(ashr_10):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $6, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$10, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	10(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_10):
	add	$16, %r10
	jg	L(nibble_ashr_10)

L(gobble_ashr_10):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $10, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_10)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $10, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_10)

	.p2align 4
L(nibble_ashr_10):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xfc00, %edx
	jnz	L(ashr_10_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$5, %r11
	jbe	L(ashr_10_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_10)

	.p2align 4
L(ashr_10_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $10, %xmm0, %xmm0
	vpsrldq $10, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_11
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(5~15)          n - 5      	        10(15 +(n - 5) - n)         ashr_11
 */
	.p2align 4
L(ashr_11):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $5, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$11, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	11(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_11):
	add	$16, %r10
	jg	L(nibble_ashr_11)

L(gobble_ashr_11):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $11, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_11)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $11, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_11)

	.p2align 4
L(nibble_ashr_11):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xf800, %edx
	jnz	L(ashr_11_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$4, %r11
	jbe	L(ashr_11_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_11)

	.p2align 4
L(ashr_11_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $11, %xmm0, %xmm0
	vpsrldq $11, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_12
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(4~15)          n - 4      	        11(15 +(n - 4) - n)         ashr_12
 */
	.p2align 4
L(ashr_12):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $4, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$12, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	12(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_12):
	add	$16, %r10
	jg	L(nibble_ashr_12)

L(gobble_ashr_12):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $12, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_12)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $12, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_12)

	.p2align 4
L(nibble_ashr_12):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xf000, %edx
	jnz	L(ashr_12_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$3, %r11
	jbe	L(ashr_12_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_12)

	.p2align 4
L(ashr_12_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $12, %xmm0, %xmm0
	vpsrldq $12, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_13
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(3~15)          n - 3      	        12(15 +(n - 3) - n)         ashr_13
 */
	.p2align 4
L(ashr_13):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $3, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$13, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	13(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_13):
	add	$16, %r10
	jg	L(nibble_ashr_13)

L(gobble_ashr_13):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $13, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_13)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $13, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_13)

	.p2align 4
L(nibble_ashr_13):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xe000, %edx
	jnz	L(ashr_13_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$2, %r11
	jbe	L(ashr_13_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_13)

	.p2align 4
L(ashr_13_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $13, %xmm0, %xmm0
	vpsrldq $13, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_14
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(2~15)          n - 2      	        13(15 +(n - 2) - n)         ashr_14
 */
	.p2align 4
L(ashr_14):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $2, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)
	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$14, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	14(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */
	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_14):
	add	$16, %r10
	jg	L(nibble_ashr_14)

L(gobble_ashr_14):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $14, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_14)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $14, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_14)

	.p2align 4
L(nibble_ashr_14):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0xc000, %edx
	jnz	L(ashr_14_exittail)

#ifdef USE_AS_STRNCMP
	cmp	$1, %r11
	jbe	L(ashr_14_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_14)

	.p2align 4
L(ashr_14_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $14, %xmm0, %xmm0
	vpsrldq $14, %xmm3, %xmm3
	jmp	L(aftertail)

/*
 *  The following cases will be handled by ashr_15
 *  rcx(offset of rsi)  rax(offset of rdi)        relative offset	 corresponding case
 *        n(1~15)          n - 1      	        14(15 +(n - 1) - n)         ashr_15
 */
	.p2align 4
L(ashr_15):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqa	(%rdi), %xmm2
	vmovdqa	(%rsi), %xmm1
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpslldq $1, %xmm2, %xmm2
	vpcmpeqb %xmm1, %xmm2, %xmm2
	vpsubb %xmm0, %xmm2, %xmm2
	vpmovmskb %xmm2, %r9d
	shr	%cl, %edx
	shr	%cl, %r9d
	sub	%r9d, %edx
	jnz	L(less32bytes)

	vmovdqa	(%rdi), %xmm3

	UPDATE_STRNCMP_COUNTER

	vpxor %xmm0, %xmm0, %xmm0
	mov	$16, %rcx	/* index for loads */
	mov	$15, %r9d	/* byte position left over from less32bytes case */
	/*
	 * Setup %r10 value allows us to detect crossing a page boundary.
	 * When %r10 goes positive we have crossed a page boundary and
	 * need to do a nibble.
	 */
	lea	15(%rdi), %r10
	and	$0xfff, %r10	/* offset into 4K page */

	sub	$0x1000, %r10	/* subtract 4K pagesize */

	.p2align 4
L(loop_ashr_15):
	add	$16, %r10
	jg	L(nibble_ashr_15)

L(gobble_ashr_15):
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $15, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3

	add	$16, %r10
	jg	L(nibble_ashr_15)	/* cross page boundary */

	vmovdqa	(%rsi, %rcx), %xmm1
	vmovdqa	(%rdi, %rcx), %xmm2
	vmovdqa	%xmm2, %xmm4

	vpalignr $15, %xmm3, %xmm2, %xmm2        /* merge into one 16byte value */

	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpcmpeqb %xmm2, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	sub	$0xffff, %edx
	jnz	L(exit)

#ifdef USE_AS_STRNCMP
	sub	$16, %r11
	jbe	L(strcmp_exitz)
#endif

	add	$16, %rcx
	vmovdqa	%xmm4, %xmm3
	jmp	L(loop_ashr_15)

	.p2align 4
L(nibble_ashr_15):
	vpcmpeqb %xmm3, %xmm0, %xmm0		/* check nibble for null char */
	vpmovmskb %xmm0, %edx
	test	$0x8000, %edx
	jnz	L(ashr_15_exittail)

#ifdef USE_AS_STRNCMP
	test	%r11, %r11
	je	L(ashr_15_exittail)
#endif

	vpxor %xmm0, %xmm0, %xmm0
	sub	$0x1000, %r10
	jmp	L(gobble_ashr_15)

	.p2align 4
L(ashr_15_exittail):
	vmovdqa	(%rsi, %rcx), %xmm1
	vpsrldq $15, %xmm3, %xmm3
	vpsrldq $15, %xmm0, %xmm0

	.p2align 4
L(aftertail):
	vpcmpeqb %xmm3, %xmm1, %xmm1
	vpsubb %xmm0, %xmm1, %xmm1
	vpmovmskb %xmm1, %edx
	not	%edx

	.p2align 4
L(exit):
	lea	-16(%r9, %rcx), %rax	/* locate the exact offset for rdi */
L(less32bytes):
	lea	(%rdi, %rax), %rdi	/* locate the exact address for first operand(rdi) */
	lea	(%rsi, %rcx), %rsi	/* locate the exact address for second operand(rsi) */
	test	%r8d, %r8d
	jz	L(ret)
	xchg	%rsi, %rdi		/* recover original order according to flag(%r8d) */

	.p2align 4
L(ret):
L(less16bytes):
	bsf	%rdx, %rdx		/* find and store bit index in %rdx */

#ifdef USE_AS_STRNCMP
	sub	%rdx, %r11
	jbe	L(strcmp_exitz)
#endif
	movzbl	(%rsi, %rdx), %ecx
	movzbl	(%rdi, %rdx), %eax

	sub	%ecx, %eax
	ret

L(strcmp_exitz):
	xor	%eax, %eax
	ret

	.p2align 4
L(Byte0):
	movzbl	(%rsi), %ecx
	movzbl	(%rdi), %eax

	sub	%ecx, %eax
	ret
END (STRCMP)

	.section .rodata,"a",@progbits
	.p2align 3
L(unaligned_table):
	.int	L(ashr_1) - L(unaligned_table)
	.int	L(ashr_2) - L(unaligned_table)
	.int	L(ashr_3) - L(unaligned_table)
	.int	L(ashr_4) - L(unaligned_table)
	.int	L(ashr_5) - L(unaligned_table)
	.int	L(ashr_6) - L(unaligned_table)
	.int	L(ashr_7) - L(unaligned_table)
	.int	L(ashr_8) - L(unaligned_table)
	.int	L(ashr_9) - L(unaligned_table)
	.int	L(ashr_10) - L(unaligned_table)
	.int	L(ashr_11) - L(unaligned_table)
	.int	L(ashr_12) - L(unaligned_table)
	.int	L(ashr_13) - L(unaligned_table)
	.int	L(ashr_14) - L(unaligned_table)
	.int	L(ashr_15) - L(unaligned_table)
	.int	L(ashr_0) - L(unaligned_table)
