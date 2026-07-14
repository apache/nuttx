/*********************************************************************************
 * libs/libc/machine/x86_64/arch_memcmp_avx2.S
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
 * Included Files
 *********************************************************************************/

#include "cache.h"

/*********************************************************************************
 * Pre-processor Definitions
 *********************************************************************************/

#ifndef MEMCMP
# define MEMCMP		memcmp
#endif

#ifndef L
# define L(label)	.L##label
#endif

#ifndef ALIGN
# define ALIGN(n)	.p2align n
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
# define END(name)     \
	cfi_endproc;         \
	.size name, .-name
#endif

#ifndef ALIGN
# define ALIGN(n)	.p2align n
#endif

#define JMPTBL(I, B)	(I - B)

#define BRANCH_TO_JMPTBL_ENTRY(TABLE, INDEX, SCALE) \
  lea		TABLE(%rip), %r11;                          \
  movslq	(%r11, INDEX, SCALE), %rcx;               \
  add		%r11, %rcx;                                 \
  jmp		*%rcx;                                      \
  ud2

/*********************************************************************************
 * Public Functions
 *********************************************************************************/

	.section .text.avx2,"ax",@progbits
ENTRY (MEMCMP)
#ifdef USE_AS_WMEMCMP
	shl	$2, %rdx
#endif
	vpxor %xmm0, %xmm0, %xmm0
	cmp	$79, %rdx
	ja	L(79bytesormore)
#ifndef USE_AS_WMEMCMP
	cmp	$1, %rdx
	je	L(firstbyte)
#endif
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

#ifndef USE_AS_WMEMCMP
	ALIGN (4)
L(firstbyte):
	movzbl	(%rdi), %eax
	movzbl	(%rsi), %ecx
	sub	%ecx, %eax
	ret
#endif

	ALIGN (4)
L(79bytesormore):
	vmovdqu	(%rsi), %xmm1
	vmovdqu	(%rdi), %xmm2
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)
	mov	%rsi, %rcx
	and	$-16, %rsi
	add	$16, %rsi
	sub	%rsi, %rcx

	sub	%rcx, %rdi
	add	%rcx, %rdx
	test	$0xf, %rdi
	jz	L(2aligned)

	cmp	$128, %rdx
	ja	L(128bytesormore)
L(less128bytes):
	sub	$64, %rdx

	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqu	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)

	vmovdqu	32(%rdi), %xmm2
	vpxor 32(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(48bytesin256)

	vmovdqu	48(%rdi), %xmm2
	vpxor 48(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(64bytesin256)
	cmp	$32, %rdx
	jb	L(less32bytesin64)

	vmovdqu	64(%rdi), %xmm2
	vpxor 64(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(80bytesin256)

	vmovdqu	80(%rdi), %xmm2
	vpxor 80(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(96bytesin256)
	sub	$32, %rdx
	add	$32, %rdi
	add	$32, %rsi
L(less32bytesin64):
	add	$64, %rdi
	add	$64, %rsi
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

L(128bytesormore):
	cmp	$512, %rdx
	ja	L(512bytesormore)
	cmp	$256, %rdx
	ja	L(less512bytes)
L(less256bytes):
	sub	$128, %rdx

	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqu	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)

	vmovdqu	32(%rdi), %xmm2
	vpxor 32(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(48bytesin256)

	vmovdqu	48(%rdi), %xmm2
	vpxor 48(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(64bytesin256)

	vmovdqu	64(%rdi), %xmm2
	vpxor 64(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(80bytesin256)

	vmovdqu	80(%rdi), %xmm2
	vpxor 80(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(96bytesin256)

	vmovdqu	96(%rdi), %xmm2
	vpxor 96(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(112bytesin256)

	vmovdqu	112(%rdi), %xmm2
	vpxor 112(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(128bytesin256)

	add	$128, %rsi
	add	$128, %rdi

	cmp	$64, %rdx
	jae	L(less128bytes)

	cmp	$32, %rdx
	jb	L(less32bytesin128)

	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqu	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)
	sub	$32, %rdx
	add	$32, %rdi
	add	$32, %rsi
L(less32bytesin128):
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

L(less512bytes):
	sub	$256, %rdx
	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqu	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)

	vmovdqu	32(%rdi), %xmm2
	vpxor 32(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(48bytesin256)

	vmovdqu	48(%rdi), %xmm2
	vpxor 48(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(64bytesin256)

	vmovdqu	64(%rdi), %xmm2
	vpxor 64(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(80bytesin256)

	vmovdqu	80(%rdi), %xmm2
	vpxor 80(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(96bytesin256)

	vmovdqu	96(%rdi), %xmm2
	vpxor 96(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(112bytesin256)

	vmovdqu	112(%rdi), %xmm2
	vpxor 112(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(128bytesin256)

	vmovdqu	128(%rdi), %xmm2
	vpxor 128(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(144bytesin256)

	vmovdqu	144(%rdi), %xmm2
	vpxor 144(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(160bytesin256)

	vmovdqu	160(%rdi), %xmm2
	vpxor 160(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(176bytesin256)

	vmovdqu	176(%rdi), %xmm2
	vpxor 176(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(192bytesin256)

	vmovdqu	192(%rdi), %xmm2
	vpxor 192(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(208bytesin256)

	vmovdqu	208(%rdi), %xmm2
	vpxor 208(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(224bytesin256)

	vmovdqu	224(%rdi), %xmm2
	vpxor 224(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(240bytesin256)

	vmovdqu	240(%rdi), %xmm2
	vpxor 240(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(256bytesin256)

	add	$256, %rsi
	add	$256, %rdi

	cmp	$128, %rdx
	jae	L(less256bytes)

	cmp	$64, %rdx
	jae	L(less128bytes)

	cmp	$32, %rdx
	jb	L(less32bytesin256)

	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqu	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)
	sub	$32, %rdx
	add	$32, %rdi
	add	$32, %rsi
L(less32bytesin256):
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

	ALIGN (4)
L(512bytesormore):
#ifdef DATA_CACHE_SIZE_HALF
	mov	$DATA_CACHE_SIZE_HALF, %r8
#else
	mov	__x86_64_data_cache_size_half(%rip), %r8
#endif
	mov	%r8, %r9
	shr	$1, %r8
	add	%r9, %r8
	cmp	%r8, %rdx
	ja	L(L2_L3_cache_unaglined)
	sub	$64, %rdx
	ALIGN (4)
L(64bytesormore_loop):
	/* This loop stays 128-bit even in the AVX2 build: measurement on real
	 * hardware (TSC) showed that widening it to 256-bit ymm REGRESSES
	 * memcmp (the loop is latency-bound on the vptest->jnc dependency, and
	 * 256-bit vptest has higher latency than 128-bit; halving the
	 * instruction count does not help a latency-bound loop). Only the
	 * instruction ENCODING is VEX (v-prefixed) to avoid the AVX<->SSE
	 * transition penalty when memcmp is called between AVX code.
	 */
	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vmovdqa	%xmm2, %xmm1

	vmovdqu	16(%rdi), %xmm3
	vpxor 16(%rsi), %xmm3, %xmm3
	vpor %xmm3, %xmm1, %xmm1

	vmovdqu	32(%rdi), %xmm4
	vpxor 32(%rsi), %xmm4, %xmm4
	vpor %xmm4, %xmm1, %xmm1

	vmovdqu	48(%rdi), %xmm5
	vpxor 48(%rsi), %xmm5, %xmm5
	vpor %xmm5, %xmm1, %xmm1

	vptest	%xmm1, %xmm0
	jnc	L(64bytesormore_loop_end)
	add	$64, %rsi
	add	$64, %rdi
	sub	$64, %rdx
	jae	L(64bytesormore_loop)

	add	$64, %rdx
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

L(L2_L3_cache_unaglined):
	sub	$64, %rdx
	ALIGN (4)
L(L2_L3_unaligned_128bytes_loop):
	prefetchnta 0x1c0(%rdi)
	prefetchnta 0x1c0(%rsi)
	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vmovdqa	%xmm2, %xmm1

	vmovdqu	16(%rdi), %xmm3
	vpxor 16(%rsi), %xmm3, %xmm3
	vpor %xmm3, %xmm1, %xmm1

	vmovdqu	32(%rdi), %xmm4
	vpxor 32(%rsi), %xmm4, %xmm4
	vpor %xmm4, %xmm1, %xmm1

	vmovdqu	48(%rdi), %xmm5
	vpxor 48(%rsi), %xmm5, %xmm5
	vpor %xmm5, %xmm1, %xmm1

	vptest	%xmm1, %xmm0
	jnc	L(64bytesormore_loop_end)
	add	$64, %rsi
	add	$64, %rdi
	sub	$64, %rdx
	jae	L(L2_L3_unaligned_128bytes_loop)

	add	$64, %rdx
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

/*
 * This case is for machines which are sensitive for unaligned instructions.
 */
	ALIGN (4)
L(2aligned):
	cmp	$128, %rdx
	ja	L(128bytesormorein2aligned)
L(less128bytesin2aligned):
	sub	$64, %rdx

	vmovdqa	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqa	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)

	vmovdqa	32(%rdi), %xmm2
	vpxor 32(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(48bytesin256)

	vmovdqa	48(%rdi), %xmm2
	vpxor 48(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(64bytesin256)
	cmp	$32, %rdx
	jb	L(less32bytesin64in2alinged)

	vmovdqa	64(%rdi), %xmm2
	vpxor 64(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(80bytesin256)

	vmovdqa	80(%rdi), %xmm2
	vpxor 80(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(96bytesin256)
	sub	$32, %rdx
	add	$32, %rdi
	add	$32, %rsi
L(less32bytesin64in2alinged):
	add	$64, %rdi
	add	$64, %rsi
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

	ALIGN (4)
L(128bytesormorein2aligned):
	cmp	$512, %rdx
	ja	L(512bytesormorein2aligned)
	cmp	$256, %rdx
	ja	L(256bytesormorein2aligned)
L(less256bytesin2alinged):
	sub	$128, %rdx

	vmovdqa	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqa	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)

	vmovdqa	32(%rdi), %xmm2
	vpxor 32(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(48bytesin256)

	vmovdqa	48(%rdi), %xmm2
	vpxor 48(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(64bytesin256)

	vmovdqa	64(%rdi), %xmm2
	vpxor 64(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(80bytesin256)

	vmovdqa	80(%rdi), %xmm2
	vpxor 80(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(96bytesin256)

	vmovdqa	96(%rdi), %xmm2
	vpxor 96(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(112bytesin256)

	vmovdqa	112(%rdi), %xmm2
	vpxor 112(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(128bytesin256)

	add	$128, %rsi
	add	$128, %rdi

	cmp	$64, %rdx
	jae	L(less128bytesin2aligned)

	cmp	$32, %rdx
	jb	L(less32bytesin128in2aligned)

	vmovdqu	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqu	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)
	sub	$32, %rdx
	add	$32, %rdi
	add	$32, %rsi
L(less32bytesin128in2aligned):
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

	ALIGN (4)
L(256bytesormorein2aligned):

	sub	$256, %rdx
	vmovdqa	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqa	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)

	vmovdqa	32(%rdi), %xmm2
	vpxor 32(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(48bytesin256)

	vmovdqa	48(%rdi), %xmm2
	vpxor 48(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(64bytesin256)

	vmovdqa	64(%rdi), %xmm2
	vpxor 64(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(80bytesin256)

	vmovdqa	80(%rdi), %xmm2
	vpxor 80(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(96bytesin256)

	vmovdqa	96(%rdi), %xmm2
	vpxor 96(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(112bytesin256)

	vmovdqa	112(%rdi), %xmm2
	vpxor 112(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(128bytesin256)

	vmovdqa	128(%rdi), %xmm2
	vpxor 128(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(144bytesin256)

	vmovdqa	144(%rdi), %xmm2
	vpxor 144(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(160bytesin256)

	vmovdqa	160(%rdi), %xmm2
	vpxor 160(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(176bytesin256)

	vmovdqa	176(%rdi), %xmm2
	vpxor 176(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(192bytesin256)

	vmovdqa	192(%rdi), %xmm2
	vpxor 192(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(208bytesin256)

	vmovdqa	208(%rdi), %xmm2
	vpxor 208(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(224bytesin256)

	vmovdqa	224(%rdi), %xmm2
	vpxor 224(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(240bytesin256)

	vmovdqa	240(%rdi), %xmm2
	vpxor 240(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(256bytesin256)

	add	$256, %rsi
	add	$256, %rdi

	cmp	$128, %rdx
	jae	L(less256bytesin2alinged)

	cmp	$64, %rdx
	jae	L(less128bytesin2aligned)

	cmp	$32, %rdx
	jb	L(less32bytesin256in2alinged)

	vmovdqa	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(16bytesin256)

	vmovdqa	16(%rdi), %xmm2
	vpxor 16(%rsi), %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(32bytesin256)
	sub	$32, %rdx
	add	$32, %rdi
	add	$32, %rsi
L(less32bytesin256in2alinged):
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)

	ALIGN (4)
L(512bytesormorein2aligned):
#ifdef DATA_CACHE_SIZE_HALF
	mov	$DATA_CACHE_SIZE_HALF, %r8
#else
	mov	__x86_64_data_cache_size_half(%rip), %r8
#endif
	mov	%r8, %r9
	shr	$1, %r8
	add	%r9, %r8
	cmp	%r8, %rdx
	ja	L(L2_L3_cache_aglined)

	sub	$64, %rdx
	ALIGN (4)
L(64bytesormore_loopin2aligned):
	vmovdqa	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vmovdqa	%xmm2, %xmm1

	vmovdqa	16(%rdi), %xmm3
	vpxor 16(%rsi), %xmm3, %xmm3
	vpor %xmm3, %xmm1, %xmm1

	vmovdqa	32(%rdi), %xmm4
	vpxor 32(%rsi), %xmm4, %xmm4
	vpor %xmm4, %xmm1, %xmm1

	vmovdqa	48(%rdi), %xmm5
	vpxor 48(%rsi), %xmm5, %xmm5
	vpor %xmm5, %xmm1, %xmm1

	vptest	%xmm1, %xmm0
	jnc	L(64bytesormore_loop_end)
	add	$64, %rsi
	add	$64, %rdi
	sub	$64, %rdx
	jae	L(64bytesormore_loopin2aligned)

	add	$64, %rdx
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)
L(L2_L3_cache_aglined):
	sub	$64, %rdx
	ALIGN (4)
L(L2_L3_aligned_128bytes_loop):
	prefetchnta 0x1c0(%rdi)
	prefetchnta 0x1c0(%rsi)
	vmovdqa	(%rdi), %xmm2
	vpxor (%rsi), %xmm2, %xmm2
	vmovdqa	%xmm2, %xmm1

	vmovdqa	16(%rdi), %xmm3
	vpxor 16(%rsi), %xmm3, %xmm3
	vpor %xmm3, %xmm1, %xmm1

	vmovdqa	32(%rdi), %xmm4
	vpxor 32(%rsi), %xmm4, %xmm4
	vpor %xmm4, %xmm1, %xmm1

	vmovdqa	48(%rdi), %xmm5
	vpxor 48(%rsi), %xmm5, %xmm5
	vpor %xmm5, %xmm1, %xmm1

	vptest	%xmm1, %xmm0
	jnc	L(64bytesormore_loop_end)
	add	$64, %rsi
	add	$64, %rdi
	sub	$64, %rdx
	jae	L(L2_L3_aligned_128bytes_loop)

	add	$64, %rdx
	add	%rdx, %rsi
	add	%rdx, %rdi
	BRANCH_TO_JMPTBL_ENTRY(L(table_64bytes), %rdx, 4)


	ALIGN (4)
L(64bytesormore_loop_end):
	add	$16, %rdi
	add	$16, %rsi
	vptest	%xmm2, %xmm0
	jnc	L(16bytes)

	add	$16, %rdi
	add	$16, %rsi
	vptest	%xmm3, %xmm0
	jnc	L(16bytes)

	add	$16, %rdi
	add	$16, %rsi
	vptest	%xmm4, %xmm0
	jnc	L(16bytes)

	add	$16, %rdi
	add	$16, %rsi
	jmp	L(16bytes)

L(256bytesin256):
	add	$256, %rdi
	add	$256, %rsi
	jmp	L(16bytes)
L(240bytesin256):
	add	$240, %rdi
	add	$240, %rsi
	jmp	L(16bytes)
L(224bytesin256):
	add	$224, %rdi
	add	$224, %rsi
	jmp	L(16bytes)
L(208bytesin256):
	add	$208, %rdi
	add	$208, %rsi
	jmp	L(16bytes)
L(192bytesin256):
	add	$192, %rdi
	add	$192, %rsi
	jmp	L(16bytes)
L(176bytesin256):
	add	$176, %rdi
	add	$176, %rsi
	jmp	L(16bytes)
L(160bytesin256):
	add	$160, %rdi
	add	$160, %rsi
	jmp	L(16bytes)
L(144bytesin256):
	add	$144, %rdi
	add	$144, %rsi
	jmp	L(16bytes)
L(128bytesin256):
	add	$128, %rdi
	add	$128, %rsi
	jmp	L(16bytes)
L(112bytesin256):
	add	$112, %rdi
	add	$112, %rsi
	jmp	L(16bytes)
L(96bytesin256):
	add	$96, %rdi
	add	$96, %rsi
	jmp	L(16bytes)
L(80bytesin256):
	add	$80, %rdi
	add	$80, %rsi
	jmp	L(16bytes)
L(64bytesin256):
	add	$64, %rdi
	add	$64, %rsi
	jmp	L(16bytes)
L(48bytesin256):
	add	$16, %rdi
	add	$16, %rsi
L(32bytesin256):
	add	$16, %rdi
	add	$16, %rsi
L(16bytesin256):
	add	$16, %rdi
	add	$16, %rsi
L(16bytes):
	mov	-16(%rdi), %rax
	mov	-16(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
L(8bytes):
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(12bytes):
	mov	-12(%rdi), %rax
	mov	-12(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
L(4bytes):
	mov	-4(%rsi), %ecx
	mov	-4(%rdi), %eax
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
L(0bytes):
	xor	%eax, %eax
	ret

#ifndef USE_AS_WMEMCMP
/* unreal case for wmemcmp */
	ALIGN (4)
L(65bytes):
	vmovdqu	-65(%rdi), %xmm1
	vmovdqu	-65(%rsi), %xmm2
	mov	$-65, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(49bytes):
	vmovdqu	-49(%rdi), %xmm1
	vmovdqu	-49(%rsi), %xmm2
	mov	$-49, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(33bytes):
	vmovdqu	-33(%rdi), %xmm1
	vmovdqu	-33(%rsi), %xmm2
	mov	$-33, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(17bytes):
	mov	-17(%rdi), %rax
	mov	-17(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
L(9bytes):
	mov	-9(%rdi), %rax
	mov	-9(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	movzbl	-1(%rdi), %eax
	movzbl	-1(%rsi), %edx
	sub	%edx, %eax
	ret

	ALIGN (4)
L(13bytes):
	mov	-13(%rdi), %rax
	mov	-13(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(5bytes):
	mov	-5(%rdi), %eax
	mov	-5(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	movzbl	-1(%rdi), %eax
	movzbl	-1(%rsi), %edx
	sub	%edx, %eax
	ret

	ALIGN (4)
L(66bytes):
	vmovdqu	-66(%rdi), %xmm1
	vmovdqu	-66(%rsi), %xmm2
	mov	$-66, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(50bytes):
	vmovdqu	-50(%rdi), %xmm1
	vmovdqu	-50(%rsi), %xmm2
	mov	$-50, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(34bytes):
	vmovdqu	-34(%rdi), %xmm1
	vmovdqu	-34(%rsi), %xmm2
	mov	$-34, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(18bytes):
	mov	-18(%rdi), %rax
	mov	-18(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
L(10bytes):
	mov	-10(%rdi), %rax
	mov	-10(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	movzwl	-2(%rdi), %eax
	movzwl	-2(%rsi), %ecx
	cmp	%cl, %al
	jne	L(end)
	and	$0xffff, %eax
	and	$0xffff, %ecx
	sub	%ecx, %eax
	ret

	ALIGN (4)
L(14bytes):
	mov	-14(%rdi), %rax
	mov	-14(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(6bytes):
	mov	-6(%rdi), %eax
	mov	-6(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
L(2bytes):
	movzwl	-2(%rsi), %ecx
	movzwl	-2(%rdi), %eax
	cmp	%cl, %al
	jne	L(end)
	and	$0xffff, %eax
	and	$0xffff, %ecx
	sub	%ecx, %eax
	ret

	ALIGN (4)
L(67bytes):
	vmovdqu	-67(%rdi), %xmm2
	vmovdqu	-67(%rsi), %xmm1
	mov	$-67, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(51bytes):
	vmovdqu	-51(%rdi), %xmm2
	vmovdqu	-51(%rsi), %xmm1
	mov	$-51, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(35bytes):
	vmovdqu	-35(%rsi), %xmm1
	vmovdqu	-35(%rdi), %xmm2
	mov	$-35, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(19bytes):
	mov	-19(%rdi), %rax
	mov	-19(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
L(11bytes):
	mov	-11(%rdi), %rax
	mov	-11(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-4(%rdi), %eax
	mov	-4(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(15bytes):
	mov	-15(%rdi), %rax
	mov	-15(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(7bytes):
	mov	-7(%rdi), %eax
	mov	-7(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	mov	-4(%rdi), %eax
	mov	-4(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(3bytes):
	movzwl	-3(%rdi), %eax
	movzwl	-3(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin2bytes)
L(1bytes):
	movzbl	-1(%rdi), %eax
	movzbl	-1(%rsi), %ecx
	sub	%ecx, %eax
	ret
#endif

	ALIGN (4)
L(68bytes):
	vmovdqu	-68(%rdi), %xmm2
	vmovdqu	-68(%rsi), %xmm1
	mov	$-68, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(52bytes):
	vmovdqu	-52(%rdi), %xmm2
	vmovdqu	-52(%rsi), %xmm1
	mov	$-52, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(36bytes):
	vmovdqu	-36(%rdi), %xmm2
	vmovdqu	-36(%rsi), %xmm1
	mov	$-36, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(20bytes):
	vmovdqu	-20(%rdi), %xmm2
	vmovdqu	-20(%rsi), %xmm1
	mov	$-20, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-4(%rdi), %eax
	mov	-4(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	xor	%eax, %eax
	ret

#ifndef USE_AS_WMEMCMP
/* unreal cases for wmemcmp */
	ALIGN (4)
L(69bytes):
	vmovdqu	-69(%rsi), %xmm1
	vmovdqu	-69(%rdi), %xmm2
	mov	$-69, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(53bytes):
	vmovdqu	-53(%rsi), %xmm1
	vmovdqu	-53(%rdi), %xmm2
	mov	$-53, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(37bytes):
	vmovdqu	-37(%rsi), %xmm1
	vmovdqu	-37(%rdi), %xmm2
	mov	$-37, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(21bytes):
	vmovdqu	-21(%rsi), %xmm1
	vmovdqu	-21(%rdi), %xmm2
	mov	$-21, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(70bytes):
	vmovdqu	-70(%rsi), %xmm1
	vmovdqu	-70(%rdi), %xmm2
	mov	$-70, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(54bytes):
	vmovdqu	-54(%rsi), %xmm1
	vmovdqu	-54(%rdi), %xmm2
	mov	$-54, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(38bytes):
	vmovdqu	-38(%rsi), %xmm1
	vmovdqu	-38(%rdi), %xmm2
	mov	$-38, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(22bytes):
	vmovdqu	-22(%rsi), %xmm1
	vmovdqu	-22(%rdi), %xmm2
	mov	$-22, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(71bytes):
	vmovdqu	-71(%rsi), %xmm1
	vmovdqu	-71(%rdi), %xmm2
	mov	$-71, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(55bytes):
	vmovdqu	-55(%rdi), %xmm2
	vmovdqu	-55(%rsi), %xmm1
	mov	$-55, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(39bytes):
	vmovdqu	-39(%rdi), %xmm2
	vmovdqu	-39(%rsi), %xmm1
	mov	$-39, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(23bytes):
	vmovdqu	-23(%rdi), %xmm2
	vmovdqu	-23(%rsi), %xmm1
	mov	$-23, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret
#endif

	ALIGN (4)
L(72bytes):
	vmovdqu	-72(%rsi), %xmm1
	vmovdqu	-72(%rdi), %xmm2
	mov	$-72, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(56bytes):
	vmovdqu	-56(%rdi), %xmm2
	vmovdqu	-56(%rsi), %xmm1
	mov	$-56, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(40bytes):
	vmovdqu	-40(%rdi), %xmm2
	vmovdqu	-40(%rsi), %xmm1
	mov	$-40, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(24bytes):
	vmovdqu	-24(%rdi), %xmm2
	vmovdqu	-24(%rsi), %xmm1
	mov	$-24, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

#ifndef USE_AS_WMEMCMP
/* unreal cases for wmemcmp */
	ALIGN (4)
L(73bytes):
	vmovdqu	-73(%rsi), %xmm1
	vmovdqu	-73(%rdi), %xmm2
	mov	$-73, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(57bytes):
	vmovdqu	-57(%rdi), %xmm2
	vmovdqu	-57(%rsi), %xmm1
	mov	$-57, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(41bytes):
	vmovdqu	-41(%rdi), %xmm2
	vmovdqu	-41(%rsi), %xmm1
	mov	$-41, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(25bytes):
	vmovdqu	-25(%rdi), %xmm2
	vmovdqu	-25(%rsi), %xmm1
	mov	$-25, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-9(%rdi), %rax
	mov	-9(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	movzbl	-1(%rdi), %eax
	movzbl	-1(%rsi), %ecx
	sub	%ecx, %eax
	ret

	ALIGN (4)
L(74bytes):
	vmovdqu	-74(%rsi), %xmm1
	vmovdqu	-74(%rdi), %xmm2
	mov	$-74, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(58bytes):
	vmovdqu	-58(%rdi), %xmm2
	vmovdqu	-58(%rsi), %xmm1
	mov	$-58, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(42bytes):
	vmovdqu	-42(%rdi), %xmm2
	vmovdqu	-42(%rsi), %xmm1
	mov	$-42, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(26bytes):
	vmovdqu	-26(%rdi), %xmm2
	vmovdqu	-26(%rsi), %xmm1
	mov	$-26, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-10(%rdi), %rax
	mov	-10(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	movzwl	-2(%rdi), %eax
	movzwl	-2(%rsi), %ecx
	jmp	L(diffin2bytes)

	ALIGN (4)
L(75bytes):
	vmovdqu	-75(%rsi), %xmm1
	vmovdqu	-75(%rdi), %xmm2
	mov	$-75, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(59bytes):
	vmovdqu	-59(%rdi), %xmm2
	vmovdqu	-59(%rsi), %xmm1
	mov	$-59, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(43bytes):
	vmovdqu	-43(%rdi), %xmm2
	vmovdqu	-43(%rsi), %xmm1
	mov	$-43, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(27bytes):
	vmovdqu	-27(%rdi), %xmm2
	vmovdqu	-27(%rsi), %xmm1
	mov	$-27, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-11(%rdi), %rax
	mov	-11(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-4(%rdi), %eax
	mov	-4(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	xor	%eax, %eax
	ret
#endif
	ALIGN (4)
L(76bytes):
	vmovdqu	-76(%rsi), %xmm1
	vmovdqu	-76(%rdi), %xmm2
	mov	$-76, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(60bytes):
	vmovdqu	-60(%rdi), %xmm2
	vmovdqu	-60(%rsi), %xmm1
	mov	$-60, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(44bytes):
	vmovdqu	-44(%rdi), %xmm2
	vmovdqu	-44(%rsi), %xmm1
	mov	$-44, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(28bytes):
	vmovdqu	-28(%rdi), %xmm2
	vmovdqu	-28(%rsi), %xmm1
	mov	$-28, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-12(%rdi), %rax
	mov	-12(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-4(%rdi), %eax
	mov	-4(%rsi), %ecx
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	xor	%eax, %eax
	ret

#ifndef USE_AS_WMEMCMP
/* unreal cases for wmemcmp */
	ALIGN (4)
L(77bytes):
	vmovdqu	-77(%rsi), %xmm1
	vmovdqu	-77(%rdi), %xmm2
	mov	$-77, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(61bytes):
	vmovdqu	-61(%rdi), %xmm2
	vmovdqu	-61(%rsi), %xmm1
	mov	$-61, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(45bytes):
	vmovdqu	-45(%rdi), %xmm2
	vmovdqu	-45(%rsi), %xmm1
	mov	$-45, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(29bytes):
	vmovdqu	-29(%rdi), %xmm2
	vmovdqu	-29(%rsi), %xmm1
	mov	$-29, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)

	mov	-13(%rdi), %rax
	mov	-13(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)

	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(78bytes):
	vmovdqu	-78(%rsi), %xmm1
	vmovdqu	-78(%rdi), %xmm2
	mov	$-78, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(62bytes):
	vmovdqu	-62(%rdi), %xmm2
	vmovdqu	-62(%rsi), %xmm1
	mov	$-62, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(46bytes):
	vmovdqu	-46(%rdi), %xmm2
	vmovdqu	-46(%rsi), %xmm1
	mov	$-46, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(30bytes):
	vmovdqu	-30(%rdi), %xmm2
	vmovdqu	-30(%rsi), %xmm1
	mov	$-30, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-14(%rdi), %rax
	mov	-14(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

	ALIGN (4)
L(79bytes):
	vmovdqu	-79(%rsi), %xmm1
	vmovdqu	-79(%rdi), %xmm2
	mov	$-79, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(63bytes):
	vmovdqu	-63(%rdi), %xmm2
	vmovdqu	-63(%rsi), %xmm1
	mov	$-63, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(47bytes):
	vmovdqu	-47(%rdi), %xmm2
	vmovdqu	-47(%rsi), %xmm1
	mov	$-47, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(31bytes):
	vmovdqu	-31(%rdi), %xmm2
	vmovdqu	-31(%rsi), %xmm1
	mov	$-31, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
	mov	-15(%rdi), %rax
	mov	-15(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret
#endif
	ALIGN (4)
L(64bytes):
	vmovdqu	-64(%rdi), %xmm2
	vmovdqu	-64(%rsi), %xmm1
	mov	$-64, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(48bytes):
	vmovdqu	-48(%rdi), %xmm2
	vmovdqu	-48(%rsi), %xmm1
	mov	$-48, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)
L(32bytes):
	vmovdqu	-32(%rdi), %xmm2
	vmovdqu	-32(%rsi), %xmm1
	mov	$-32, %dl
	vpxor %xmm1, %xmm2, %xmm2
	vptest	%xmm2, %xmm0
	jnc	L(less16bytes)

	mov	-16(%rdi), %rax
	mov	-16(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)

	mov	-8(%rdi), %rax
	mov	-8(%rsi), %rcx
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	xor	%eax, %eax
	ret

/*
 * Aligned 8 bytes to avoid 2 branch "taken" in one 16 aligned code block.
 */
	ALIGN (3)
L(less16bytes):
	movsbq	%dl, %rdx
	mov	(%rsi, %rdx), %rcx
	mov	(%rdi, %rdx), %rax
	cmp	%rax, %rcx
	jne	L(diffin8bytes)
	mov	8(%rsi, %rdx), %rcx
	mov	8(%rdi, %rdx), %rax
L(diffin8bytes):
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	shr	$32, %rcx
	shr	$32, %rax

#ifdef USE_AS_WMEMCMP
/* for wmemcmp */
	cmp	%eax, %ecx
	jne	L(diffin4bytes)
	xor	%eax, %eax
	ret
#endif

L(diffin4bytes):
#ifndef USE_AS_WMEMCMP
	cmp	%cx, %ax
	jne	L(diffin2bytes)
	shr	$16, %ecx
	shr	$16, %eax
L(diffin2bytes):
	cmp	%cl, %al
	jne	L(end)
	and	$0xffff, %eax
	and	$0xffff, %ecx
	sub	%ecx, %eax
	ret
#else

/* for wmemcmp */
	mov	$1, %eax
	jl	L(nequal_bigger)
	neg	%eax
	ret

	ALIGN (4)
L(nequal_bigger):
	ret

L(unreal_case):
	xor	%eax, %eax
	ret
#endif

	ALIGN (4)
L(end):
	and	$0xff, %eax
	and	$0xff, %ecx
	sub	%ecx, %eax
	ret

END (MEMCMP)

	.section .rodata.avx2,"a",@progbits
	ALIGN (3)
#ifndef USE_AS_WMEMCMP
L(table_64bytes):
	.int	JMPTBL (L(0bytes), L(table_64bytes))
	.int	JMPTBL (L(1bytes), L(table_64bytes))
	.int	JMPTBL (L(2bytes), L(table_64bytes))
	.int	JMPTBL (L(3bytes), L(table_64bytes))
	.int	JMPTBL (L(4bytes), L(table_64bytes))
	.int	JMPTBL (L(5bytes), L(table_64bytes))
	.int	JMPTBL (L(6bytes), L(table_64bytes))
	.int	JMPTBL (L(7bytes), L(table_64bytes))
	.int	JMPTBL (L(8bytes), L(table_64bytes))
	.int	JMPTBL (L(9bytes), L(table_64bytes))
	.int	JMPTBL (L(10bytes), L(table_64bytes))
	.int	JMPTBL (L(11bytes), L(table_64bytes))
	.int	JMPTBL (L(12bytes), L(table_64bytes))
	.int	JMPTBL (L(13bytes), L(table_64bytes))
	.int	JMPTBL (L(14bytes), L(table_64bytes))
	.int	JMPTBL (L(15bytes), L(table_64bytes))
	.int	JMPTBL (L(16bytes), L(table_64bytes))
	.int	JMPTBL (L(17bytes), L(table_64bytes))
	.int	JMPTBL (L(18bytes), L(table_64bytes))
	.int	JMPTBL (L(19bytes), L(table_64bytes))
	.int	JMPTBL (L(20bytes), L(table_64bytes))
	.int	JMPTBL (L(21bytes), L(table_64bytes))
	.int	JMPTBL (L(22bytes), L(table_64bytes))
	.int	JMPTBL (L(23bytes), L(table_64bytes))
	.int	JMPTBL (L(24bytes), L(table_64bytes))
	.int	JMPTBL (L(25bytes), L(table_64bytes))
	.int	JMPTBL (L(26bytes), L(table_64bytes))
	.int	JMPTBL (L(27bytes), L(table_64bytes))
	.int	JMPTBL (L(28bytes), L(table_64bytes))
	.int	JMPTBL (L(29bytes), L(table_64bytes))
	.int	JMPTBL (L(30bytes), L(table_64bytes))
	.int	JMPTBL (L(31bytes), L(table_64bytes))
	.int	JMPTBL (L(32bytes), L(table_64bytes))
	.int	JMPTBL (L(33bytes), L(table_64bytes))
	.int	JMPTBL (L(34bytes), L(table_64bytes))
	.int	JMPTBL (L(35bytes), L(table_64bytes))
	.int	JMPTBL (L(36bytes), L(table_64bytes))
	.int	JMPTBL (L(37bytes), L(table_64bytes))
	.int	JMPTBL (L(38bytes), L(table_64bytes))
	.int	JMPTBL (L(39bytes), L(table_64bytes))
	.int	JMPTBL (L(40bytes), L(table_64bytes))
	.int	JMPTBL (L(41bytes), L(table_64bytes))
	.int	JMPTBL (L(42bytes), L(table_64bytes))
	.int	JMPTBL (L(43bytes), L(table_64bytes))
	.int	JMPTBL (L(44bytes), L(table_64bytes))
	.int	JMPTBL (L(45bytes), L(table_64bytes))
	.int	JMPTBL (L(46bytes), L(table_64bytes))
	.int	JMPTBL (L(47bytes), L(table_64bytes))
	.int	JMPTBL (L(48bytes), L(table_64bytes))
	.int	JMPTBL (L(49bytes), L(table_64bytes))
	.int	JMPTBL (L(50bytes), L(table_64bytes))
	.int	JMPTBL (L(51bytes), L(table_64bytes))
	.int	JMPTBL (L(52bytes), L(table_64bytes))
	.int	JMPTBL (L(53bytes), L(table_64bytes))
	.int	JMPTBL (L(54bytes), L(table_64bytes))
	.int	JMPTBL (L(55bytes), L(table_64bytes))
	.int	JMPTBL (L(56bytes), L(table_64bytes))
	.int	JMPTBL (L(57bytes), L(table_64bytes))
	.int	JMPTBL (L(58bytes), L(table_64bytes))
	.int	JMPTBL (L(59bytes), L(table_64bytes))
	.int	JMPTBL (L(60bytes), L(table_64bytes))
	.int	JMPTBL (L(61bytes), L(table_64bytes))
	.int	JMPTBL (L(62bytes), L(table_64bytes))
	.int	JMPTBL (L(63bytes), L(table_64bytes))
	.int	JMPTBL (L(64bytes), L(table_64bytes))
	.int	JMPTBL (L(65bytes), L(table_64bytes))
	.int	JMPTBL (L(66bytes), L(table_64bytes))
	.int	JMPTBL (L(67bytes), L(table_64bytes))
	.int	JMPTBL (L(68bytes), L(table_64bytes))
	.int	JMPTBL (L(69bytes), L(table_64bytes))
	.int	JMPTBL (L(70bytes), L(table_64bytes))
	.int	JMPTBL (L(71bytes), L(table_64bytes))
	.int	JMPTBL (L(72bytes), L(table_64bytes))
	.int	JMPTBL (L(73bytes), L(table_64bytes))
	.int	JMPTBL (L(74bytes), L(table_64bytes))
	.int	JMPTBL (L(75bytes), L(table_64bytes))
	.int	JMPTBL (L(76bytes), L(table_64bytes))
	.int	JMPTBL (L(77bytes), L(table_64bytes))
	.int	JMPTBL (L(78bytes), L(table_64bytes))
	.int	JMPTBL (L(79bytes), L(table_64bytes))
#else
L(table_64bytes):
	.int	JMPTBL (L(0bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(4bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(8bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(12bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(16bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(20bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(24bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(28bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(32bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(36bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(40bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(44bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(48bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(52bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(56bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(60bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(64bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(68bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(72bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(76bytes), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
	.int	JMPTBL (L(unreal_case), L(table_64bytes))
#endif
