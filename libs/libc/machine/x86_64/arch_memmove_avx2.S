/*********************************************************************************
 * libs/libc/machine/x86_64/arch_memmove_avx2.S
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
 * AVX2 variant of memmove()/memcpy().
 *
 * This is a fork of arch_memmove.S with two differences:
 *   1. Every SIMD instruction uses the VEX encoding (v-prefixed). Mixing
 *      legacy-SSE (movdqu %xmm) with VEX-256 (vmov* %ymm) triggers the
 *      AVX<->SSE state transition penalty (tens of cycles per boundary), so
 *      the ENTIRE function is VEX-encoded even for the 128-bit head/tail
 *      operations. A single vzeroupper at the exit clears the upper YMM
 *      state before returning to legacy-SSE callers.
 *   2. The four fixed-length 64B/iteration copy loops (forward/backward main
 *      loops and forward/backward non-temporal large-page loops) use 256-bit
 *      ymm registers: 2 stores of 32 bytes replace the previous 4 stores of
 *      16 bytes, halving the store-instruction count on the streaming path.
 *      The loop destination is always aligned to 64 bytes, which satisfies
 *      the 32-byte alignment required by vmovdqa/vmovntdq.
 *
 * The <=128B head/tail blocks keep their original 128-bit (xmm) overlap
 * layout to avoid re-deriving the overlap offsets (a known source of bugs);
 * they only switch to the VEX encoding.
 *
 * This file is selected by Make.defs / CMakeLists.txt when
 * CONFIG_ARCH_X86_64_AVX=y; otherwise arch_memmove.S (SSE2) is built.
 *********************************************************************************/

/*********************************************************************************
 * Included Files
 *********************************************************************************/

#include "cache.h"

/*********************************************************************************
 * Pre-processor Definitions
 *********************************************************************************/

#ifndef MEMMOVE
# define MEMMOVE		memmove
#endif

#ifndef L
# define L(label)	.L##label
#endif

#ifndef cfi_startproc
# define cfi_startproc	.cfi_startproc
#endif

#ifndef cfi_endproc
# define cfi_endproc	.cfi_endproc
#endif

#ifndef cfi_rel_offset
# define cfi_rel_offset(reg, off)	.cfi_rel_offset reg, off
#endif

#ifndef cfi_restore
# define cfi_restore(reg)	.cfi_restore reg
#endif

#ifndef cfi_adjust_cfa_offset
# define cfi_adjust_cfa_offset(off)	.cfi_adjust_cfa_offset off
#endif

#ifndef ENTRY
# define ENTRY(name)      \
	.type name,  @function; \
	.globl name;            \
	.p2align 4;             \
name:                     \
	cfi_startproc
#endif

#ifndef ALIAS_SYMBOL
# define ALIAS_SYMBOL(alias, original) \
	.globl alias;                        \
	.equ alias, original
#endif

#ifndef END
# define END(name)  \
	cfi_endproc;      \
	.size name, .-name
#endif

#define CFI_PUSH(REG)           \
	cfi_adjust_cfa_offset (4);    \
	cfi_rel_offset (REG, 0)

#define CFI_POP(REG)            \
	cfi_adjust_cfa_offset (-4);   \
	cfi_restore (REG)

#define PUSH(REG)	push REG;
#define POP(REG)	pop REG;

#define ENTRANCE	PUSH (%rbx);
#define RETURN_END	POP (%rbx); ret
#define RETURN		RETURN_END;

/*********************************************************************************
 * Public Functions
 *********************************************************************************/

	.section .text.avx2,"ax",@progbits
ENTRY (MEMMOVE)
	ENTRANCE
	mov	%rdi, %rax

/* Check whether we should copy backward or forward.  */
	cmp	%rsi, %rdi
	je	L(mm_return)
	jg	L(mm_len_0_or_more_backward)

/* Now do checks for lengths. We do [0..16], [0..32], [0..64], [0..128]
	separately.  */
	cmp	$16, %rdx
	jbe	L(mm_len_0_16_bytes_forward)

	cmp	$32, %rdx
	ja	L(mm_len_32_or_more_forward)

/* Copy [0..32] and return.  */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	-16(%rsi, %rdx), %xmm1
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, -16(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_len_32_or_more_forward):
	cmp	$64, %rdx
	ja	L(mm_len_64_or_more_forward)

/* Copy [0..64] and return.  */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm1
	vmovdqu	-16(%rsi, %rdx), %xmm2
	vmovdqu	-32(%rsi, %rdx), %xmm3
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, 16(%rdi)
	vmovdqu	%xmm2, -16(%rdi, %rdx)
	vmovdqu	%xmm3, -32(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_len_64_or_more_forward):
	cmp	$128, %rdx
	ja	L(mm_len_128_or_more_forward)

/* Copy [0..128] and return.  */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm1
	vmovdqu	32(%rsi), %xmm2
	vmovdqu	48(%rsi), %xmm3
	vmovdqu	-64(%rsi, %rdx), %xmm4
	vmovdqu	-48(%rsi, %rdx), %xmm5
	vmovdqu	-32(%rsi, %rdx), %xmm6
	vmovdqu	-16(%rsi, %rdx), %xmm7
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, 16(%rdi)
	vmovdqu	%xmm2, 32(%rdi)
	vmovdqu	%xmm3, 48(%rdi)
	vmovdqu	%xmm4, -64(%rdi, %rdx)
	vmovdqu	%xmm5, -48(%rdi, %rdx)
	vmovdqu	%xmm6, -32(%rdi, %rdx)
	vmovdqu	%xmm7, -16(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_len_128_or_more_forward):
/* Aligning the address of destination.  */
/*  save first unaligned 64 bytes */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm1
	vmovdqu	32(%rsi), %xmm2
	vmovdqu	48(%rsi), %xmm3

	lea	64(%rdi), %r8
	and	$-64, %r8  /* r8 now aligned to next 64 byte boundary */
	sub	%rdi, %rsi /* rsi = src - dst = diff */

	vmovdqu	(%r8, %rsi), %xmm4
	vmovdqu	16(%r8, %rsi), %xmm5
	vmovdqu	32(%r8, %rsi), %xmm6
	vmovdqu	48(%r8, %rsi), %xmm7

	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, 16(%rdi)
	vmovdqu	%xmm2, 32(%rdi)
	vmovdqu	%xmm3, 48(%rdi)
	vmovdqa	%xmm4, (%r8)
	vmovdqa	%xmm5, 16(%r8)
	vmovdqa	%xmm6, 32(%r8)
	vmovdqa	%xmm7, 48(%r8)
	add	$64, %r8

	lea	(%rdi, %rdx), %rbx
	and	$-64, %rbx
	cmp	%r8, %rbx
	jbe	L(mm_copy_remaining_forward)

	cmp	$SHARED_CACHE_SIZE_HALF, %rdx
	jae	L(mm_large_page_loop_forward)

	.p2align 4
L(mm_main_loop_forward):

	prefetcht0 128(%r8, %rsi)

	/* 64 bytes/iteration via two 256-bit ymm loads/stores. dst (%r8) is
	 * 64-byte aligned, so vmovdqa is safe.
	 */
	vmovdqu	(%r8, %rsi), %ymm0
	vmovdqu	32(%r8, %rsi), %ymm1
	vmovdqa	%ymm0, (%r8)
	vmovdqa	%ymm1, 32(%r8)
	lea	64(%r8), %r8
	cmp	%r8, %rbx
	ja	L(mm_main_loop_forward)

L(mm_copy_remaining_forward):
	add	%rdi, %rdx
	sub	%r8, %rdx
/* We copied all up till %rdi position in the dst.
	In %rdx now is how many bytes are left to copy.
	Now we need to advance %r8. */
	lea	(%r8, %rsi), %r9

L(mm_remaining_0_64_bytes_forward):
	cmp	$32, %rdx
	ja	L(mm_remaining_33_64_bytes_forward)
	cmp	$16, %rdx
	ja	L(mm_remaining_17_32_bytes_forward)
	test	%rdx, %rdx
	.p2align 4,,2
	je	L(mm_return)

	cmpb	$8, %dl
	ja	L(mm_remaining_9_16_bytes_forward)
	cmpb	$4, %dl
	.p2align 4,,5
	ja	L(mm_remaining_5_8_bytes_forward)
	cmpb	$2, %dl
	.p2align 4,,1
	ja	L(mm_remaining_3_4_bytes_forward)
	movzbl	-1(%r9,%rdx), %esi
	movzbl	(%r9), %ebx
	movb	%sil, -1(%r8,%rdx)
	movb	%bl, (%r8)
	jmp	L(mm_return)

L(mm_remaining_33_64_bytes_forward):
	vmovdqu	(%r9), %xmm0
	vmovdqu	16(%r9), %xmm1
	vmovdqu	-32(%r9, %rdx), %xmm2
	vmovdqu	-16(%r9, %rdx), %xmm3
	vmovdqu	%xmm0, (%r8)
	vmovdqu	%xmm1, 16(%r8)
	vmovdqu	%xmm2, -32(%r8, %rdx)
	vmovdqu	%xmm3, -16(%r8, %rdx)
	jmp	L(mm_return)

L(mm_remaining_17_32_bytes_forward):
	vmovdqu	(%r9), %xmm0
	vmovdqu	-16(%r9, %rdx), %xmm1
	vmovdqu	%xmm0, (%r8)
	vmovdqu	%xmm1, -16(%r8, %rdx)
	jmp	L(mm_return)

L(mm_remaining_5_8_bytes_forward):
	movl	(%r9), %esi
	movl	-4(%r9,%rdx), %ebx
	movl	%esi, (%r8)
	movl	%ebx, -4(%r8,%rdx)
	jmp	L(mm_return)

L(mm_remaining_9_16_bytes_forward):
	mov	(%r9), %rsi
	mov	-8(%r9, %rdx), %rbx
	mov	%rsi, (%r8)
	mov	%rbx, -8(%r8, %rdx)
	jmp	L(mm_return)

L(mm_remaining_3_4_bytes_forward):
	movzwl	-2(%r9,%rdx), %esi
	movzwl	(%r9), %ebx
	movw	%si, -2(%r8,%rdx)
	movw	%bx, (%r8)
	jmp	L(mm_return)

L(mm_len_0_16_bytes_forward):
	testb	$24, %dl
	jne	L(mm_len_9_16_bytes_forward)
	testb	$4, %dl
	.p2align 4,,5
	jne	L(mm_len_5_8_bytes_forward)
	test	%rdx, %rdx
	.p2align 4,,2
	je	L(mm_return)
	testb	$2, %dl
	.p2align 4,,1
	jne	L(mm_len_2_4_bytes_forward)
	movzbl	-1(%rsi,%rdx), %ebx
	movzbl	(%rsi), %esi
	movb	%bl, -1(%rdi,%rdx)
	movb	%sil, (%rdi)
	jmp	L(mm_return)

L(mm_len_2_4_bytes_forward):
	movzwl	-2(%rsi,%rdx), %ebx
	movzwl	(%rsi), %esi
	movw	%bx, -2(%rdi,%rdx)
	movw	%si, (%rdi)
	jmp	L(mm_return)

L(mm_len_5_8_bytes_forward):
	movl	(%rsi), %ebx
	movl	-4(%rsi,%rdx), %esi
	movl	%ebx, (%rdi)
	movl	%esi, -4(%rdi,%rdx)
	jmp	L(mm_return)

L(mm_len_9_16_bytes_forward):
	mov	(%rsi), %rbx
	mov	-8(%rsi, %rdx), %rsi
	mov	%rbx, (%rdi)
	mov	%rsi, -8(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_recalc_len):
/* Compute in %rdx how many bytes are left to copy after
	the main loop stops.  */
	mov 	%rbx, %rdx
	sub 	%rdi, %rdx
/* The code for copying backwards.  */
L(mm_len_0_or_more_backward):

/* Now do checks for lengths. We do [0..16], [16..32], [32..64], [64..128]
	separately.  */
	cmp	$16, %rdx
	jbe	L(mm_len_0_16_bytes_backward)

	cmp	$32, %rdx
	ja	L(mm_len_32_or_more_backward)

/* Copy [0..32] and return.  */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	-16(%rsi, %rdx), %xmm1
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, -16(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_len_32_or_more_backward):
	cmp	$64, %rdx
	ja	L(mm_len_64_or_more_backward)

/* Copy [0..64] and return.  */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm1
	vmovdqu	-16(%rsi, %rdx), %xmm2
	vmovdqu	-32(%rsi, %rdx), %xmm3
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, 16(%rdi)
	vmovdqu	%xmm2, -16(%rdi, %rdx)
	vmovdqu	%xmm3, -32(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_len_64_or_more_backward):
	cmp	$128, %rdx
	ja	L(mm_len_128_or_more_backward)

/* Copy [0..128] and return.  */
	vmovdqu	(%rsi), %xmm0
	vmovdqu	16(%rsi), %xmm1
	vmovdqu	32(%rsi), %xmm2
	vmovdqu	48(%rsi), %xmm3
	vmovdqu	-64(%rsi, %rdx), %xmm4
	vmovdqu	-48(%rsi, %rdx), %xmm5
	vmovdqu	-32(%rsi, %rdx), %xmm6
	vmovdqu	-16(%rsi, %rdx), %xmm7
	vmovdqu	%xmm0, (%rdi)
	vmovdqu	%xmm1, 16(%rdi)
	vmovdqu	%xmm2, 32(%rdi)
	vmovdqu	%xmm3, 48(%rdi)
	vmovdqu	%xmm4, -64(%rdi, %rdx)
	vmovdqu	%xmm5, -48(%rdi, %rdx)
	vmovdqu	%xmm6, -32(%rdi, %rdx)
	vmovdqu	%xmm7, -16(%rdi, %rdx)
	jmp	L(mm_return)

L(mm_len_128_or_more_backward):
/* Aligning the address of destination. We need to save
	16 bits from the source in order not to overwrite them.  */
	vmovdqu	-16(%rsi, %rdx), %xmm0
	vmovdqu	-32(%rsi, %rdx), %xmm1
	vmovdqu	-48(%rsi, %rdx), %xmm2
	vmovdqu	-64(%rsi, %rdx), %xmm3

	lea	(%rdi, %rdx), %r9
	and	$-64, %r9 /* r9 = aligned dst */

	mov	%rsi, %r8
	sub	%rdi, %r8 /* r8 = src - dst, diff */

	vmovdqu	-16(%r9, %r8), %xmm4
	vmovdqu	-32(%r9, %r8), %xmm5
	vmovdqu	-48(%r9, %r8), %xmm6
	vmovdqu	-64(%r9, %r8), %xmm7

	vmovdqu	%xmm0, -16(%rdi, %rdx)
	vmovdqu	%xmm1, -32(%rdi, %rdx)
	vmovdqu	%xmm2, -48(%rdi, %rdx)
	vmovdqu	%xmm3, -64(%rdi, %rdx)
	vmovdqa	%xmm4, -16(%r9)
	vmovdqa	%xmm5, -32(%r9)
	vmovdqa	%xmm6, -48(%r9)
	vmovdqa	%xmm7, -64(%r9)
	lea	-64(%r9), %r9

	lea	64(%rdi), %rbx
	and	$-64, %rbx

	cmp	%r9, %rbx
	jae	L(mm_recalc_len)

	cmp	$SHARED_CACHE_SIZE_HALF, %rdx
	jae	L(mm_large_page_loop_backward)

	.p2align 4
L(mm_main_loop_backward):

	prefetcht0 -128(%r9, %r8)

	/* 64 bytes/iteration via two 256-bit ymm loads/stores. dst (%r9) is
	 * 64-byte aligned, so vmovdqa is safe.
	 */
	vmovdqu	-64(%r9, %r8), %ymm0
	vmovdqu	-32(%r9, %r8), %ymm1
	vmovdqa	%ymm0, -64(%r9)
	vmovdqa	%ymm1, -32(%r9)
	lea	-64(%r9), %r9
	cmp	%r9, %rbx
	jb	L(mm_main_loop_backward)
	jmp	L(mm_recalc_len)

/* Copy [0..16] and return.  */
L(mm_len_0_16_bytes_backward):
	testb	$24, %dl
	jnz	L(mm_len_9_16_bytes_backward)
	testb	$4, %dl
	.p2align 4,,5
	jnz	L(mm_len_5_8_bytes_backward)
	test	%rdx, %rdx
	.p2align 4,,2
	je	L(mm_return)
	testb	$2, %dl
	.p2align 4,,1
	jne	L(mm_len_3_4_bytes_backward)
	movzbl	-1(%rsi,%rdx), %ebx
	movzbl	(%rsi), %ecx
	movb	%bl, -1(%rdi,%rdx)
	movb	%cl, (%rdi)
	jmp	L(mm_return)

L(mm_len_3_4_bytes_backward):
	movzwl	-2(%rsi,%rdx), %ebx
	movzwl	(%rsi), %ecx
	movw	%bx, -2(%rdi,%rdx)
	movw	%cx, (%rdi)
	jmp	L(mm_return)

L(mm_len_9_16_bytes_backward):
	movl	-4(%rsi,%rdx), %ebx
	movl	-8(%rsi,%rdx), %ecx
	movl	%ebx, -4(%rdi,%rdx)
	movl	%ecx, -8(%rdi,%rdx)
	sub	$8, %rdx
	jmp	L(mm_len_0_16_bytes_backward)

L(mm_len_5_8_bytes_backward):
	movl	(%rsi), %ebx
	movl	-4(%rsi,%rdx), %ecx
	movl	%ebx, (%rdi)
	movl	%ecx, -4(%rdi,%rdx)

L(mm_return):
	/* Clear the upper YMM state before returning to a possibly legacy-SSE
	 * caller. Harmless on the GPR-only small-copy paths.
	 */
	vzeroupper
	RETURN

/* Big length copy forward part.  */

	.p2align 4
L(mm_large_page_loop_forward):
	vmovdqu	(%r8, %rsi), %ymm0
	vmovdqu	32(%r8, %rsi), %ymm1
	vmovntdq %ymm0, (%r8)
	vmovntdq %ymm1, 32(%r8)
	lea 	64(%r8), %r8
	cmp	%r8, %rbx
	ja	L(mm_large_page_loop_forward)
	sfence
	jmp	L(mm_copy_remaining_forward)

/* Big length copy backward part.  */
	.p2align 4
L(mm_large_page_loop_backward):
	vmovdqu	-64(%r9, %r8), %ymm0
	vmovdqu	-32(%r9, %r8), %ymm1
	vmovntdq %ymm0, -64(%r9)
	vmovntdq %ymm1, -32(%r9)
	lea 	-64(%r9), %r9
	cmp	%r9, %rbx
	jb	L(mm_large_page_loop_backward)
	sfence
	jmp	L(mm_recalc_len)

END (MEMMOVE)

ALIAS_SYMBOL(memcpy, MEMMOVE)
