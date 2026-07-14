/*********************************************************************************
 * libs/libc/machine/x86_64/arch_strcpy_avx2.S
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

#ifndef USE_AS_STRCAT

# ifndef STRCPY
#  define STRCPY	strcpy
# endif

# ifndef L
#  define L(label)	.L##label
# endif

# ifndef cfi_startproc
#  define cfi_startproc	.cfi_startproc
# endif

# ifndef cfi_endproc
#  define cfi_endproc	.cfi_endproc
# endif

# ifndef ENTRY
#  define ENTRY(name)     \
	.type name, @function;  \
	.globl name;            \
	.p2align 4;             \
name:                     \
	cfi_startproc
# endif

# ifndef END
#  define END(name) \
	cfi_endproc;      \
	.size name, .-name
# endif

#endif

#define JMPTBL(I, B)	I - B
#define BRANCH_TO_JMPTBL_ENTRY(TABLE, INDEX, SCALE) \
	lea	TABLE(%rip), %r11;                            \
	movslq	(%r11, INDEX, SCALE), %rcx;               \
	lea	(%r11, %rcx), %rcx;                           \
	jmp	*%rcx

#ifndef USE_AS_STRCAT

# define RETURN ret

/*********************************************************************************
 * Public Functions
 *********************************************************************************/

.text
ENTRY (STRCPY)
# ifdef USE_AS_STRNCPY
	mov	%rdx, %r8
	test	%r8, %r8
	jz	L(ExitZero)
# endif
	mov	%rsi, %rcx
# ifndef USE_AS_STPCPY
	mov	%rdi, %rax      /* save result */
# endif

#endif
	and	$63, %rcx
	cmp	$32, %rcx
	jbe	L(SourceStringAlignmentLess32)

	and	$-16, %rsi
	and	$15, %rcx
	vpxor %xmm0, %xmm0, %xmm0
	vpxor %xmm1, %xmm1, %xmm1

	vpcmpeqb (%rsi), %xmm1, %xmm1
	vpmovmskb %xmm1, %rdx
	shr	%cl, %rdx
#ifdef USE_AS_STRNCPY
# if defined USE_AS_STPCPY || defined USE_AS_STRCAT
	mov	$16, %r10
	sub	%rcx, %r10
	cmp	%r10, %r8
# else
	mov	$17, %r10
	sub	%rcx, %r10
	cmp	%r10, %r8
# endif
	jbe	L(CopyFrom1To16BytesTailCase2OrCase3)
#endif
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesTail)

	vpcmpeqb 16(%rsi), %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
#ifdef USE_AS_STRNCPY
	add	$16, %r10
	cmp	%r10, %r8
	jbe	L(CopyFrom1To32BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
	jnz	L(CopyFrom1To32Bytes)

	vmovdqu	(%rsi, %rcx), %xmm1   /* copy 16 bytes */
	vmovdqu	%xmm1, (%rdi)

/* If source address alignment != destination address alignment */
	.p2align 4
L(Unalign16Both):
	sub	%rcx, %rdi
#ifdef USE_AS_STRNCPY
	add	%rcx, %r8
#endif
	mov	$16, %rcx
	vmovdqa	(%rsi, %rcx), %xmm1
	vmovaps	16(%rsi, %rcx), %xmm2
	vmovdqu	%xmm1, (%rdi, %rcx)
	vpcmpeqb %xmm2, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$16, %rcx
#ifdef USE_AS_STRNCPY
	sub	$48, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm2)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vmovaps	16(%rsi, %rcx), %xmm3
	vmovdqu	%xmm2, (%rdi, %rcx)
	vpcmpeqb %xmm3, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$16, %rcx
#ifdef USE_AS_STRNCPY
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm3)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vmovaps	16(%rsi, %rcx), %xmm4
	vmovdqu	%xmm3, (%rdi, %rcx)
	vpcmpeqb %xmm4, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$16, %rcx
#ifdef USE_AS_STRNCPY
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm4)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vmovaps	16(%rsi, %rcx), %xmm1
	vmovdqu	%xmm4, (%rdi, %rcx)
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$16, %rcx
#ifdef USE_AS_STRNCPY
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm1)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vmovaps	16(%rsi, %rcx), %xmm2
	vmovdqu	%xmm1, (%rdi, %rcx)
	vpcmpeqb %xmm2, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$16, %rcx
#ifdef USE_AS_STRNCPY
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm2)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vmovaps	16(%rsi, %rcx), %xmm3
	vmovdqu	%xmm2, (%rdi, %rcx)
	vpcmpeqb %xmm3, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$16, %rcx
#ifdef USE_AS_STRNCPY
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
#endif
	test	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm3)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vmovdqu	%xmm3, (%rdi, %rcx)
	mov	%rsi, %rdx
	lea	16(%rsi, %rcx), %rsi
	and	$-0x40, %rsi
	sub	%rsi, %rdx
	sub	%rdx, %rdi
#ifdef USE_AS_STRNCPY
	lea	128(%r8, %rdx), %r8
#endif
L(Unaligned64Loop):
	vmovaps	(%rsi), %xmm2
	vmovaps	%xmm2, %xmm4
	vmovaps	16(%rsi), %xmm5
	vmovaps	32(%rsi), %xmm3
	vmovaps	%xmm3, %xmm6
	vmovaps	48(%rsi), %xmm7
	vpminub %xmm5, %xmm2, %xmm2
	vpminub %xmm7, %xmm3, %xmm3
	vpminub %xmm2, %xmm3, %xmm3
	vpcmpeqb %xmm0, %xmm3, %xmm3
	vpmovmskb %xmm3, %rdx
#ifdef USE_AS_STRNCPY
	sub	$64, %r8
	jbe	L(UnalignedLeaveCase2OrCase3)
#endif
	test	%rdx, %rdx
	jnz	L(Unaligned64Leave)

L(Unaligned64Loop_start):
	add	$64, %rdi
	add	$64, %rsi
	vmovdqu	%xmm4, -64(%rdi)
	vmovaps	(%rsi), %xmm2
	vmovdqa	%xmm2, %xmm4
	vmovdqu	%xmm5, -48(%rdi)
	vmovaps	16(%rsi), %xmm5
	vpminub %xmm5, %xmm2, %xmm2
	vmovaps	32(%rsi), %xmm3
	vmovdqu	%xmm6, -32(%rdi)
	vmovaps	%xmm3, %xmm6
	vmovdqu	%xmm7, -16(%rdi)
	vmovaps	48(%rsi), %xmm7
	vpminub %xmm7, %xmm3, %xmm3
	vpminub %xmm2, %xmm3, %xmm3
	vpcmpeqb %xmm0, %xmm3, %xmm3
	vpmovmskb %xmm3, %rdx
#ifdef USE_AS_STRNCPY
	sub	$64, %r8
	jbe	L(UnalignedLeaveCase2OrCase3)
#endif
	test	%rdx, %rdx
	jz	L(Unaligned64Loop_start)

L(Unaligned64Leave):
	vpxor %xmm1, %xmm1, %xmm1

	vpcmpeqb %xmm4, %xmm0, %xmm0
	vpcmpeqb %xmm5, %xmm1, %xmm1
	vpmovmskb %xmm0, %rdx
	vpmovmskb %xmm1, %rcx
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesUnaligned_0)
	test	%rcx, %rcx
	jnz	L(CopyFrom1To16BytesUnaligned_16)

	vpcmpeqb %xmm6, %xmm0, %xmm0
	vpcmpeqb %xmm7, %xmm1, %xmm1
	vpmovmskb %xmm0, %rdx
	vpmovmskb %xmm1, %rcx
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesUnaligned_32)

	bsf	%rcx, %rdx
	vmovdqu	%xmm4, (%rdi)
	vmovdqu	%xmm5, 16(%rdi)
	vmovdqu	%xmm6, 32(%rdi)
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
# ifdef USE_AS_STPCPY
	lea	48(%rdi, %rdx), %rax
# endif
	vmovdqu	%xmm7, 48(%rdi)
	add	$15, %r8
	sub	%rdx, %r8
	lea	49(%rdi, %rdx), %rdi
	jmp	L(StrncpyFillTailWithZero)
#else
	add	$48, %rsi
	add	$48, %rdi
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)
#endif

/* If source address alignment == destination address alignment */

L(SourceStringAlignmentLess32):
	vpxor %xmm0, %xmm0, %xmm0
	vmovdqu	(%rsi), %xmm1
	vmovdqu	16(%rsi), %xmm2
	vpcmpeqb %xmm1, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx

#ifdef USE_AS_STRNCPY
# if defined USE_AS_STPCPY || defined USE_AS_STRCAT
	cmp	$16, %r8
# else
	cmp	$17, %r8
# endif
	jbe	L(CopyFrom1To16BytesTail1Case2OrCase3)
#endif
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesTail1)

	vpcmpeqb %xmm2, %xmm0, %xmm0
	vmovdqu	%xmm1, (%rdi)
	vpmovmskb %xmm0, %rdx

#ifdef USE_AS_STRNCPY
# if defined USE_AS_STPCPY || defined USE_AS_STRCAT
	cmp	$32, %r8
# else
	cmp	$33, %r8
# endif
	jbe	L(CopyFrom1To32Bytes1Case2OrCase3)
#endif
	test	%rdx, %rdx
	jnz	L(CopyFrom1To32Bytes1)

	and	$15, %rcx
	and	$-16, %rsi

	jmp	L(Unalign16Both)

/*------End of main part with loops---------------------*/

/* Case1 */

#if (!defined USE_AS_STRNCPY) || (defined USE_AS_STRCAT)
	.p2align 4
L(CopyFrom1To16Bytes):
	add	%rcx, %rdi
	add	%rcx, %rsi
	bsf	%rdx, %rdx
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)
#endif
	.p2align 4
L(CopyFrom1To16BytesTail):
	add	%rcx, %rsi
	bsf	%rdx, %rdx
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)

	.p2align 4
L(CopyFrom1To32Bytes1):
	add	$16, %rsi
	add	$16, %rdi
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$16, %r8
#endif
L(CopyFrom1To16BytesTail1):
	bsf	%rdx, %rdx
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)

	.p2align 4
L(CopyFrom1To32Bytes):
	bsf	%rdx, %rdx
	add	%rcx, %rsi
	add	$16, %rdx
	sub	%rcx, %rdx
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)

	.p2align 4
L(CopyFrom1To16BytesUnaligned_0):
	bsf	%rdx, %rdx
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
# ifdef USE_AS_STPCPY
	lea	(%rdi, %rdx), %rax
# endif
	vmovdqu	%xmm4, (%rdi)
	add	$63, %r8
	sub	%rdx, %r8
	lea	1(%rdi, %rdx), %rdi
	jmp	L(StrncpyFillTailWithZero)
#else
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)
#endif

	.p2align 4
L(CopyFrom1To16BytesUnaligned_16):
	bsf	%rcx, %rdx
	vmovdqu	%xmm4, (%rdi)
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
# ifdef USE_AS_STPCPY
	lea	16(%rdi, %rdx), %rax
# endif
	vmovdqu	%xmm5, 16(%rdi)
	add	$47, %r8
	sub	%rdx, %r8
	lea	17(%rdi, %rdx), %rdi
	jmp	L(StrncpyFillTailWithZero)
#else
	add	$16, %rsi
	add	$16, %rdi
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)
#endif

	.p2align 4
L(CopyFrom1To16BytesUnaligned_32):
	bsf	%rdx, %rdx
	vmovdqu	%xmm4, (%rdi)
	vmovdqu	%xmm5, 16(%rdi)
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
# ifdef USE_AS_STPCPY
	lea	32(%rdi, %rdx), %rax
# endif
	vmovdqu	%xmm6, 32(%rdi)
	add	$31, %r8
	sub	%rdx, %r8
	lea	33(%rdi, %rdx), %rdi
	jmp	L(StrncpyFillTailWithZero)
#else
	add	$32, %rsi
	add	$32, %rdi
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)
#endif

#ifdef USE_AS_STRNCPY
# ifndef USE_AS_STRCAT
	.p2align 4
L(CopyFrom1To16BytesUnalignedXmm6):
	vmovdqu	%xmm6, (%rdi, %rcx)
	jmp	L(CopyFrom1To16BytesXmmExit)

	.p2align 4
L(CopyFrom1To16BytesUnalignedXmm5):
	vmovdqu	%xmm5, (%rdi, %rcx)
	jmp	L(CopyFrom1To16BytesXmmExit)

	.p2align 4
L(CopyFrom1To16BytesUnalignedXmm4):
	vmovdqu	%xmm4, (%rdi, %rcx)
	jmp	L(CopyFrom1To16BytesXmmExit)

	.p2align 4
L(CopyFrom1To16BytesUnalignedXmm3):
	vmovdqu	%xmm3, (%rdi, %rcx)
	jmp	L(CopyFrom1To16BytesXmmExit)

	.p2align 4
L(CopyFrom1To16BytesUnalignedXmm1):
	vmovdqu	%xmm1, (%rdi, %rcx)
	jmp	L(CopyFrom1To16BytesXmmExit)
# endif

	.p2align 4
L(CopyFrom1To16BytesExit):
	BRANCH_TO_JMPTBL_ENTRY (L(ExitTable), %rdx, 4)

/* Case2 */

	.p2align 4
L(CopyFrom1To16BytesCase2):
	add	$16, %r8
	add	%rcx, %rdi
	add	%rcx, %rsi
	bsf	%rdx, %rdx
	cmp	%r8, %rdx
	jb	L(CopyFrom1To16BytesExit)
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

	.p2align 4
L(CopyFrom1To32BytesCase2):
	add	%rcx, %rsi
	bsf	%rdx, %rdx
	add	$16, %rdx
	sub	%rcx, %rdx
	cmp	%r8, %rdx
	jb	L(CopyFrom1To16BytesExit)
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

L(CopyFrom1To16BytesTailCase2):
	add	%rcx, %rsi
	bsf	%rdx, %rdx
	cmp	%r8, %rdx
	jb	L(CopyFrom1To16BytesExit)
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

L(CopyFrom1To16BytesTail1Case2):
	bsf	%rdx, %rdx
	cmp	%r8, %rdx
	jb	L(CopyFrom1To16BytesExit)
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

/* Case2 or Case3,  Case3 */

	.p2align 4
L(CopyFrom1To16BytesCase2OrCase3):
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesCase2)
L(CopyFrom1To16BytesCase3):
	add	$16, %r8
	add	%rcx, %rdi
	add	%rcx, %rsi
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

	.p2align 4
L(CopyFrom1To32BytesCase2OrCase3):
	test	%rdx, %rdx
	jnz	L(CopyFrom1To32BytesCase2)
	add	%rcx, %rsi
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

	.p2align 4
L(CopyFrom1To16BytesTailCase2OrCase3):
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesTailCase2)
	add	%rcx, %rsi
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

	.p2align 4
L(CopyFrom1To32Bytes1Case2OrCase3):
	add	$16, %rdi
	add	$16, %rsi
	sub	$16, %r8
L(CopyFrom1To16BytesTail1Case2OrCase3):
	test	%rdx, %rdx
	jnz	L(CopyFrom1To16BytesTail1Case2)
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

#endif

/*------------End labels regarding with copying 1-16 bytes--and 1-32 bytes----*/

	.p2align 4
L(Exit1):
	mov	%dh, (%rdi)
#ifdef USE_AS_STPCPY
	lea	(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$1, %r8
	lea	1(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit2):
	mov	(%rsi), %dx
	mov	%dx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	1(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$2, %r8
	lea	2(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit3):
	mov	(%rsi), %cx
	mov	%cx, (%rdi)
	mov	%dh, 2(%rdi)
#ifdef USE_AS_STPCPY
	lea	2(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$3, %r8
	lea	3(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit4):
	mov	(%rsi), %edx
	mov	%edx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	3(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$4, %r8
	lea	4(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit5):
	mov	(%rsi), %ecx
	mov	%dh, 4(%rdi)
	mov	%ecx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	4(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$5, %r8
	lea	5(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit6):
	mov	(%rsi), %ecx
	mov	4(%rsi), %dx
	mov	%ecx, (%rdi)
	mov	%dx, 4(%rdi)
#ifdef USE_AS_STPCPY
	lea	5(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$6, %r8
	lea	6(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit7):
	mov	(%rsi), %ecx
	mov	3(%rsi), %edx
	mov	%ecx, (%rdi)
	mov	%edx, 3(%rdi)
#ifdef USE_AS_STPCPY
	lea	6(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$7, %r8
	lea	7(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit8):
	mov	(%rsi), %rdx
	mov	%rdx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	7(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$8, %r8
	lea	8(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit9):
	mov	(%rsi), %rcx
	mov	%dh, 8(%rdi)
	mov	%rcx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	8(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$9, %r8
	lea	9(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit10):
	mov	(%rsi), %rcx
	mov	8(%rsi), %dx
	mov	%rcx, (%rdi)
	mov	%dx, 8(%rdi)
#ifdef USE_AS_STPCPY
	lea	9(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$10, %r8
	lea	10(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit11):
	mov	(%rsi), %rcx
	mov	7(%rsi), %edx
	mov	%rcx, (%rdi)
	mov	%edx, 7(%rdi)
#ifdef USE_AS_STPCPY
	lea	10(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$11, %r8
	lea	11(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit12):
	mov	(%rsi), %rcx
	mov	8(%rsi), %edx
	mov	%rcx, (%rdi)
	mov	%edx, 8(%rdi)
#ifdef USE_AS_STPCPY
	lea	11(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$12, %r8
	lea	12(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit13):
	mov	(%rsi), %rcx
	mov	5(%rsi), %rdx
	mov	%rcx, (%rdi)
	mov	%rdx, 5(%rdi)
#ifdef USE_AS_STPCPY
	lea	12(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$13, %r8
	lea	13(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit14):
	mov	(%rsi), %rcx
	mov	6(%rsi), %rdx
	mov	%rcx, (%rdi)
	mov	%rdx, 6(%rdi)
#ifdef USE_AS_STPCPY
	lea	13(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$14, %r8
	lea	14(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit15):
	mov	(%rsi), %rcx
	mov	7(%rsi), %rdx
	mov	%rcx, (%rdi)
	mov	%rdx, 7(%rdi)
#ifdef USE_AS_STPCPY
	lea	14(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$15, %r8
	lea	15(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit16):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	%xmm0, (%rdi)
#ifdef USE_AS_STPCPY
	lea	15(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$16, %r8
	lea	16(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit17):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	%xmm0, (%rdi)
	mov	%dh, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	16(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$17, %r8
	lea	17(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit18):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %cx
	vmovdqu	%xmm0, (%rdi)
	mov	%cx, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	17(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$18, %r8
	lea	18(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit19):
	vmovdqu	(%rsi), %xmm0
	mov	15(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%ecx, 15(%rdi)
#ifdef USE_AS_STPCPY
	lea	18(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$19, %r8
	lea	19(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit20):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%ecx, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	19(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$20, %r8
	lea	20(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit21):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%ecx, 16(%rdi)
	mov	%dh, 20(%rdi)
#ifdef USE_AS_STPCPY
	lea	20(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$21, %r8
	lea	21(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit22):
	vmovdqu	(%rsi), %xmm0
	mov	14(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 14(%rdi)
#ifdef USE_AS_STPCPY
	lea	21(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$22, %r8
	lea	22(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit23):
	vmovdqu	(%rsi), %xmm0
	mov	15(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 15(%rdi)
#ifdef USE_AS_STPCPY
	lea	22(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$23, %r8
	lea	23(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit24):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	23(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$24, %r8
	lea	24(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit25):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 16(%rdi)
	mov	%dh, 24(%rdi)
#ifdef USE_AS_STPCPY
	lea	24(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$25, %r8
	lea	25(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit26):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	24(%rsi), %cx
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%cx, 24(%rdi)
#ifdef USE_AS_STPCPY
	lea	25(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$26, %r8
	lea	26(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit27):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	23(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%ecx, 23(%rdi)
#ifdef USE_AS_STPCPY
	lea	26(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$27, %r8
	lea	27(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit28):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	24(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%ecx, 24(%rdi)
#ifdef USE_AS_STPCPY
	lea	27(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$28, %r8
	lea	28(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit29):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	13(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 13(%rdi)
#ifdef USE_AS_STPCPY
	lea	28(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$29, %r8
	lea	29(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit30):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	14(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 14(%rdi)
#ifdef USE_AS_STPCPY
	lea	29(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$30, %r8
	lea	30(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit31):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	15(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 15(%rdi)
#ifdef USE_AS_STPCPY
	lea	30(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$31, %r8
	lea	31(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

	.p2align 4
L(Exit32):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	31(%rdi), %rax
#endif
#if defined USE_AS_STRNCPY && !defined USE_AS_STRCAT
	sub	$32, %r8
	lea	32(%rdi), %rdi
	jnz	L(StrncpyFillTailWithZero)
#endif
	RETURN

#ifdef USE_AS_STRNCPY

	.p2align 4
L(StrncpyExit0):
#ifdef USE_AS_STPCPY
	mov	%rdi, %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, (%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit1):
	mov	(%rsi), %dl
	mov	%dl, (%rdi)
#ifdef USE_AS_STPCPY
	lea	1(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 1(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit2):
	mov	(%rsi), %dx
	mov	%dx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	2(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 2(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit3):
	mov	(%rsi), %cx
	mov	2(%rsi), %dl
	mov	%cx, (%rdi)
	mov	%dl, 2(%rdi)
#ifdef USE_AS_STPCPY
	lea	3(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 3(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit4):
	mov	(%rsi), %edx
	mov	%edx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	4(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 4(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit5):
	mov	(%rsi), %ecx
	mov	4(%rsi), %dl
	mov	%ecx, (%rdi)
	mov	%dl, 4(%rdi)
#ifdef USE_AS_STPCPY
	lea	5(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 5(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit6):
	mov	(%rsi), %ecx
	mov	4(%rsi), %dx
	mov	%ecx, (%rdi)
	mov	%dx, 4(%rdi)
#ifdef USE_AS_STPCPY
	lea	6(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 6(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit7):
	mov	(%rsi), %ecx
	mov	3(%rsi), %edx
	mov	%ecx, (%rdi)
	mov	%edx, 3(%rdi)
#ifdef USE_AS_STPCPY
	lea	7(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 7(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit8):
	mov	(%rsi), %rdx
	mov	%rdx, (%rdi)
#ifdef USE_AS_STPCPY
	lea	8(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 8(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit9):
	mov	(%rsi), %rcx
	mov	8(%rsi), %dl
	mov	%rcx, (%rdi)
	mov	%dl, 8(%rdi)
#ifdef USE_AS_STPCPY
	lea	9(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 9(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit10):
	mov	(%rsi), %rcx
	mov	8(%rsi), %dx
	mov	%rcx, (%rdi)
	mov	%dx, 8(%rdi)
#ifdef USE_AS_STPCPY
	lea	10(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 10(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit11):
	mov	(%rsi), %rcx
	mov	7(%rsi), %edx
	mov	%rcx, (%rdi)
	mov	%edx, 7(%rdi)
#ifdef USE_AS_STPCPY
	lea	11(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 11(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit12):
	mov	(%rsi), %rcx
	mov	8(%rsi), %edx
	mov	%rcx, (%rdi)
	mov	%edx, 8(%rdi)
#ifdef USE_AS_STPCPY
	lea	12(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 12(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit13):
	mov	(%rsi), %rcx
	mov	5(%rsi), %rdx
	mov	%rcx, (%rdi)
	mov	%rdx, 5(%rdi)
#ifdef USE_AS_STPCPY
	lea	13(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 13(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit14):
	mov	(%rsi), %rcx
	mov	6(%rsi), %rdx
	mov	%rcx, (%rdi)
	mov	%rdx, 6(%rdi)
#ifdef USE_AS_STPCPY
	lea	14(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 14(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit15):
	mov	(%rsi), %rcx
	mov	7(%rsi), %rdx
	mov	%rcx, (%rdi)
	mov	%rdx, 7(%rdi)
#ifdef USE_AS_STPCPY
	lea	15(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 15(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit16):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	%xmm0, (%rdi)
#ifdef USE_AS_STPCPY
	lea	16(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 16(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit17):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %cl
	vmovdqu	%xmm0, (%rdi)
	mov	%cl, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	17(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 17(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit18):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %cx
	vmovdqu	%xmm0, (%rdi)
	mov	%cx, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	18(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 18(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit19):
	vmovdqu	(%rsi), %xmm0
	mov	15(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%ecx, 15(%rdi)
#ifdef USE_AS_STPCPY
	lea	19(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 19(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit20):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%ecx, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	20(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 20(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit21):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %ecx
	mov	20(%rsi), %dl
	vmovdqu	%xmm0, (%rdi)
	mov	%ecx, 16(%rdi)
	mov	%dl, 20(%rdi)
#ifdef USE_AS_STPCPY
	lea	21(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 21(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit22):
	vmovdqu	(%rsi), %xmm0
	mov	14(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 14(%rdi)
#ifdef USE_AS_STPCPY
	lea	22(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 22(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit23):
	vmovdqu	(%rsi), %xmm0
	mov	15(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 15(%rdi)
#ifdef USE_AS_STPCPY
	lea	23(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 23(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit24):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rcx
	vmovdqu	%xmm0, (%rdi)
	mov	%rcx, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	24(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 24(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit25):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	24(%rsi), %cl
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%cl, 24(%rdi)
#ifdef USE_AS_STPCPY
	lea	25(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 25(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit26):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	24(%rsi), %cx
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%cx, 24(%rdi)
#ifdef USE_AS_STPCPY
	lea	26(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 26(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit27):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	23(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%ecx, 23(%rdi)
#ifdef USE_AS_STPCPY
	lea	27(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 27(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit28):
	vmovdqu	(%rsi), %xmm0
	mov	16(%rsi), %rdx
	mov	24(%rsi), %ecx
	vmovdqu	%xmm0, (%rdi)
	mov	%rdx, 16(%rdi)
	mov	%ecx, 24(%rdi)
#ifdef USE_AS_STPCPY
	lea	28(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 28(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit29):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	13(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 13(%rdi)
#ifdef USE_AS_STPCPY
	lea	29(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 29(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit30):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	14(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 14(%rdi)
#ifdef USE_AS_STPCPY
	lea	30(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 30(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit31):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	15(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 15(%rdi)
#ifdef USE_AS_STPCPY
	lea	31(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 31(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit32):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm2
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 16(%rdi)
#ifdef USE_AS_STPCPY
	lea	32(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 32(%rdi)
#endif
	RETURN

	.p2align 4
L(StrncpyExit33):
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm2
	mov	32(%rsi), %cl
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm2, 16(%rdi)
	mov	%cl, 32(%rdi)
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 33(%rdi)
#endif
	RETURN

#ifndef USE_AS_STRCAT

	.p2align 4
L(Fill0):
	RETURN

	.p2align 4
L(Fill1):
	mov	%dl, (%rdi)
	RETURN

	.p2align 4
L(Fill2):
	mov	%dx, (%rdi)
	RETURN

	.p2align 4
L(Fill3):
	mov	%edx, -1(%rdi)
	RETURN

	.p2align 4
L(Fill4):
	mov	%edx, (%rdi)
	RETURN

	.p2align 4
L(Fill5):
	mov	%edx, (%rdi)
	mov	%dl, 4(%rdi)
	RETURN

	.p2align 4
L(Fill6):
	mov	%edx, (%rdi)
	mov	%dx, 4(%rdi)
	RETURN

	.p2align 4
L(Fill7):
	mov	%rdx, -1(%rdi)
	RETURN

	.p2align 4
L(Fill8):
	mov	%rdx, (%rdi)
	RETURN

	.p2align 4
L(Fill9):
	mov	%rdx, (%rdi)
	mov	%dl, 8(%rdi)
	RETURN

	.p2align 4
L(Fill10):
	mov	%rdx, (%rdi)
	mov	%dx, 8(%rdi)
	RETURN

	.p2align 4
L(Fill11):
	mov	%rdx, (%rdi)
	mov	%edx, 7(%rdi)
	RETURN

	.p2align 4
L(Fill12):
	mov	%rdx, (%rdi)
	mov	%edx, 8(%rdi)
	RETURN

	.p2align 4
L(Fill13):
	mov	%rdx, (%rdi)
	mov	%rdx, 5(%rdi)
	RETURN

	.p2align 4
L(Fill14):
	mov	%rdx, (%rdi)
	mov	%rdx, 6(%rdi)
	RETURN

	.p2align 4
L(Fill15):
	vmovdqu	%xmm0, -1(%rdi)
	RETURN

	.p2align 4
L(Fill16):
	vmovdqu	%xmm0, (%rdi)
	RETURN

	.p2align 4
L(CopyFrom1To16BytesUnalignedXmm2):
	vmovdqu	%xmm2, (%rdi, %rcx)

	.p2align 4
L(CopyFrom1To16BytesXmmExit):
	bsf	%rdx, %rdx
	add	$15, %r8
	add	%rcx, %rdi
#ifdef USE_AS_STPCPY
	lea	(%rdi, %rdx), %rax
#endif
	sub	%rdx, %r8
	lea	1(%rdi, %rdx), %rdi

	.p2align 4
L(StrncpyFillTailWithZero):
	vpxor %xmm0, %xmm0, %xmm0
	xor	%rdx, %rdx
	sub	$16, %r8
	jbe	L(StrncpyFillExit)

	vmovdqu	%xmm0, (%rdi)
	add	$16, %rdi

	mov	%rdi, %rsi
	and	$0xf, %rsi
	sub	%rsi, %rdi
	add	%rsi, %r8
	sub	$64, %r8
	jb	L(StrncpyFillLess64)

L(StrncpyFillLoopMovdqa):
	vmovdqa	%xmm0, (%rdi)
	vmovdqa	%xmm0, 16(%rdi)
	vmovdqa	%xmm0, 32(%rdi)
	vmovdqa	%xmm0, 48(%rdi)
	add	$64, %rdi
	sub	$64, %r8
	jae	L(StrncpyFillLoopMovdqa)

L(StrncpyFillLess64):
	add	$32, %r8
	jl	L(StrncpyFillLess32)
	vmovdqa	%xmm0, (%rdi)
	vmovdqa	%xmm0, 16(%rdi)
	add	$32, %rdi
	sub	$16, %r8
	jl	L(StrncpyFillExit)
	vmovdqa	%xmm0, (%rdi)
	add	$16, %rdi
	BRANCH_TO_JMPTBL_ENTRY (L(FillTable), %r8, 4)

L(StrncpyFillLess32):
	add	$16, %r8
	jl	L(StrncpyFillExit)
	vmovdqa	%xmm0, (%rdi)
	add	$16, %rdi
	BRANCH_TO_JMPTBL_ENTRY (L(FillTable), %r8, 4)

L(StrncpyFillExit):
	add	$16, %r8
	BRANCH_TO_JMPTBL_ENTRY (L(FillTable), %r8, 4)

/* end of ifndef USE_AS_STRCAT */
#endif

	.p2align 4
L(UnalignedLeaveCase2OrCase3):
	test	%rdx, %rdx
	jnz	L(Unaligned64LeaveCase2)
L(Unaligned64LeaveCase3):
	lea	64(%r8), %rcx
	and	$-16, %rcx
	add	$48, %r8
	jl	L(CopyFrom1To16BytesCase3)
	vmovdqu	%xmm4, (%rdi)
	sub	$16, %r8
	jb	L(CopyFrom1To16BytesCase3)
	vmovdqu	%xmm5, 16(%rdi)
	sub	$16, %r8
	jb	L(CopyFrom1To16BytesCase3)
	vmovdqu	%xmm6, 32(%rdi)
	sub	$16, %r8
	jb	L(CopyFrom1To16BytesCase3)
	vmovdqu	%xmm7, 48(%rdi)
#ifdef USE_AS_STPCPY
	lea	64(%rdi), %rax
#endif
#ifdef USE_AS_STRCAT
	xor	%ch, %ch
	movb	%ch, 64(%rdi)
#endif
	RETURN

	.p2align 4
L(Unaligned64LeaveCase2):
	xor	%rcx, %rcx
	vpcmpeqb %xmm4, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	add	$48, %r8
	jle	L(CopyFrom1To16BytesCase2OrCase3)
	test	%rdx, %rdx
#ifndef USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm4)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif
	vpcmpeqb %xmm5, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	vmovdqu	%xmm4, (%rdi)
	add	$16, %rcx
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
	test	%rdx, %rdx
#ifndef USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm5)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vpcmpeqb %xmm6, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	vmovdqu	%xmm5, 16(%rdi)
	add	$16, %rcx
	sub	$16, %r8
	jbe	L(CopyFrom1To16BytesCase2OrCase3)
	test	%rdx, %rdx
#ifndef USE_AS_STRCAT
	jnz	L(CopyFrom1To16BytesUnalignedXmm6)
#else
	jnz	L(CopyFrom1To16Bytes)
#endif

	vpcmpeqb %xmm7, %xmm0, %xmm0
	vpmovmskb %xmm0, %rdx
	vmovdqu	%xmm6, 32(%rdi)
	lea	16(%rdi, %rcx), %rdi
	lea	16(%rsi, %rcx), %rsi
	bsf	%rdx, %rdx
	cmp	%r8, %rdx
	jb	L(CopyFrom1To16BytesExit)
	BRANCH_TO_JMPTBL_ENTRY (L(ExitStrncpyTable), %r8, 4)

	.p2align 4
L(ExitZero):
#ifndef USE_AS_STRCAT
	mov	%rdi, %rax
#endif
	RETURN

#endif

#ifndef USE_AS_STRCAT
END (STRCPY)
#else
END (STRCAT)
#endif
	.p2align 4
	.section .rodata
L(ExitTable):
	.int	JMPTBL(L(Exit1), L(ExitTable))
	.int	JMPTBL(L(Exit2), L(ExitTable))
	.int	JMPTBL(L(Exit3), L(ExitTable))
	.int	JMPTBL(L(Exit4), L(ExitTable))
	.int	JMPTBL(L(Exit5), L(ExitTable))
	.int	JMPTBL(L(Exit6), L(ExitTable))
	.int	JMPTBL(L(Exit7), L(ExitTable))
	.int	JMPTBL(L(Exit8), L(ExitTable))
	.int	JMPTBL(L(Exit9), L(ExitTable))
	.int	JMPTBL(L(Exit10), L(ExitTable))
	.int	JMPTBL(L(Exit11), L(ExitTable))
	.int	JMPTBL(L(Exit12), L(ExitTable))
	.int	JMPTBL(L(Exit13), L(ExitTable))
	.int	JMPTBL(L(Exit14), L(ExitTable))
	.int	JMPTBL(L(Exit15), L(ExitTable))
	.int	JMPTBL(L(Exit16), L(ExitTable))
	.int	JMPTBL(L(Exit17), L(ExitTable))
	.int	JMPTBL(L(Exit18), L(ExitTable))
	.int	JMPTBL(L(Exit19), L(ExitTable))
	.int	JMPTBL(L(Exit20), L(ExitTable))
	.int	JMPTBL(L(Exit21), L(ExitTable))
	.int	JMPTBL(L(Exit22), L(ExitTable))
	.int	JMPTBL(L(Exit23), L(ExitTable))
	.int	JMPTBL(L(Exit24), L(ExitTable))
	.int	JMPTBL(L(Exit25), L(ExitTable))
	.int	JMPTBL(L(Exit26), L(ExitTable))
	.int	JMPTBL(L(Exit27), L(ExitTable))
	.int	JMPTBL(L(Exit28), L(ExitTable))
	.int	JMPTBL(L(Exit29), L(ExitTable))
	.int	JMPTBL(L(Exit30), L(ExitTable))
	.int	JMPTBL(L(Exit31), L(ExitTable))
	.int	JMPTBL(L(Exit32), L(ExitTable))
#ifdef USE_AS_STRNCPY
L(ExitStrncpyTable):
	.int	JMPTBL(L(StrncpyExit0), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit1), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit2), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit3), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit4), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit5), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit6), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit7), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit8), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit9), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit10), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit11), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit12), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit13), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit14), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit15), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit16), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit17), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit18), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit19), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit20), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit21), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit22), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit23), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit24), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit25), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit26), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit27), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit28), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit29), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit30), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit31), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit32), L(ExitStrncpyTable))
	.int	JMPTBL(L(StrncpyExit33), L(ExitStrncpyTable))
# ifndef USE_AS_STRCAT
	.p2align 4
L(FillTable):
	.int	JMPTBL(L(Fill0), L(FillTable))
	.int	JMPTBL(L(Fill1), L(FillTable))
	.int	JMPTBL(L(Fill2), L(FillTable))
	.int	JMPTBL(L(Fill3), L(FillTable))
	.int	JMPTBL(L(Fill4), L(FillTable))
	.int	JMPTBL(L(Fill5), L(FillTable))
	.int	JMPTBL(L(Fill6), L(FillTable))
	.int	JMPTBL(L(Fill7), L(FillTable))
	.int	JMPTBL(L(Fill8), L(FillTable))
	.int	JMPTBL(L(Fill9), L(FillTable))
	.int	JMPTBL(L(Fill10), L(FillTable))
	.int	JMPTBL(L(Fill11), L(FillTable))
	.int	JMPTBL(L(Fill12), L(FillTable))
	.int	JMPTBL(L(Fill13), L(FillTable))
	.int	JMPTBL(L(Fill14), L(FillTable))
	.int	JMPTBL(L(Fill15), L(FillTable))
	.int	JMPTBL(L(Fill16), L(FillTable))
# endif
#endif
