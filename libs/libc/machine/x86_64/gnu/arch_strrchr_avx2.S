/* strrchr/wcsrchr optimized with AVX2.
   Copyright (C) 2017-2022 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <https://www.gnu.org/licenses/>.  */

# include "sysdep.h"

# ifndef STRRCHR
#  define STRRCHR	strrchr
# endif

# ifdef USE_AS_WCSRCHR
#  define VPBROADCAST	vpbroadcastd
#  define VPCMPEQ	vpcmpeqd
#  define VPMIN	vpminud
#  define CHAR_SIZE	4
# else
#  define VPBROADCAST	vpbroadcastb
#  define VPCMPEQ	vpcmpeqb
#  define VPMIN	vpminub
#  define CHAR_SIZE	1
# endif

# ifndef VZEROUPPER
#  define VZEROUPPER	vzeroupper
# endif

# ifndef SECTION
#  define SECTION(p)	p##.avx
# endif

# define VEC_SIZE	32
# define PAGE_SIZE	4096

	.section SECTION(.text), "ax", @progbits
ENTRY(STRRCHR)
	vmovd	%esi, %xmm7
	movl	%edi, %eax
	/* Broadcast CHAR to YMM4.  */
	VPBROADCAST %xmm7, %ymm7
	vpxor	%xmm0, %xmm0, %xmm0

	/* Shift here instead of `andl` to save code size (saves a fetch
	   block).  */
	sall	$20, %eax
	cmpl	$((PAGE_SIZE - VEC_SIZE) << 20), %eax
	ja	L(cross_page)

L(page_cross_continue):
	vmovdqu	(%rdi), %ymm1
	/* Check end of string match.  */
	VPCMPEQ	%ymm1, %ymm0, %ymm6
	vpmovmskb %ymm6, %ecx
	testl	%ecx, %ecx
	jz	L(aligned_more)

	/* Only check match with search CHAR if needed.  */
	VPCMPEQ	%ymm1, %ymm7, %ymm1
	vpmovmskb %ymm1, %eax
	/* Check if match before first zero.  */
	blsmskl	%ecx, %ecx
	andl	%ecx, %eax
	jz	L(ret0)
	bsrl	%eax, %eax
	addq	%rdi, %rax
	/* We are off by 3 for wcsrchr if search CHAR is non-zero. If
	   search CHAR is zero we are correct. Either way `andq
	   -CHAR_SIZE, %rax` gets the correct result.  */
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
L(ret0):
L(return_vzeroupper):
	ZERO_UPPER_VEC_REGISTERS_RETURN

	/* Returns for first vec x1/x2 have hard coded backward search
	   path for earlier matches.  */
	.p2align 4,, 10
L(first_vec_x1):
	VPCMPEQ	%ymm2, %ymm7, %ymm6
	vpmovmskb %ymm6, %eax
	blsmskl	%ecx, %ecx
	andl	%ecx, %eax
	jnz	L(first_vec_x1_return)

	.p2align 4,, 4
L(first_vec_x0_test):
	VPCMPEQ	%ymm1, %ymm7, %ymm6
	vpmovmskb %ymm6, %eax
	testl	%eax, %eax
	jz	L(ret1)
	bsrl	%eax, %eax
	addq	%r8, %rax
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
L(ret1):
	VZEROUPPER_RETURN

	.p2align 4,, 10
L(first_vec_x0_x1_test):
	VPCMPEQ	%ymm2, %ymm7, %ymm6
	vpmovmskb %ymm6, %eax
	/* Check ymm2 for search CHAR match. If no match then check ymm1
	   before returning.  */
	testl	%eax, %eax
	jz	L(first_vec_x0_test)
	.p2align 4,, 4
L(first_vec_x1_return):
	bsrl	%eax, %eax
	leaq	1(%rdi, %rax), %rax
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
	VZEROUPPER_RETURN


	.p2align 4,, 10
L(first_vec_x2):
	VPCMPEQ	%ymm3, %ymm7, %ymm6
	vpmovmskb %ymm6, %eax
	blsmskl	%ecx, %ecx
	/* If no in-range search CHAR match in ymm3 then need to check
	   ymm1/ymm2 for an earlier match (we delay checking search
	   CHAR matches until needed).  */
	andl	%ecx, %eax
	jz	L(first_vec_x0_x1_test)
	bsrl	%eax, %eax
	leaq	(VEC_SIZE + 1)(%rdi, %rax), %rax
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
	VZEROUPPER_RETURN


	.p2align 4
L(aligned_more):
	/* Save original pointer if match was in VEC 0.  */
	movq	%rdi, %r8

	/* Align src.  */
	orq	$(VEC_SIZE - 1), %rdi
	vmovdqu	1(%rdi), %ymm2
	VPCMPEQ	%ymm2, %ymm0, %ymm6
	vpmovmskb %ymm6, %ecx
	testl	%ecx, %ecx
	jnz	L(first_vec_x1)

	vmovdqu	(VEC_SIZE + 1)(%rdi), %ymm3
	VPCMPEQ	%ymm3, %ymm0, %ymm6
	vpmovmskb %ymm6, %ecx
	testl	%ecx, %ecx
	jnz	L(first_vec_x2)

	/* Save pointer again before realigning.  */
	movq	%rdi, %rsi
	addq	$(VEC_SIZE + 1), %rdi
	andq	$-(VEC_SIZE * 2), %rdi
	.p2align 4
L(first_aligned_loop):
	/* Do 2x VEC at a time. Any more and the cost of finding the
	   match outweights loop benefit.  */
	vmovdqa	(VEC_SIZE * 0)(%rdi), %ymm4
	vmovdqa	(VEC_SIZE * 1)(%rdi), %ymm5

	VPCMPEQ	%ymm4, %ymm7, %ymm6
	VPMIN	%ymm4, %ymm5, %ymm8
	VPCMPEQ	%ymm5, %ymm7, %ymm10
	vpor	%ymm6, %ymm10, %ymm5
	VPCMPEQ	%ymm8, %ymm0, %ymm8
	vpor	%ymm5, %ymm8, %ymm9

	vpmovmskb %ymm9, %eax
	addq	$(VEC_SIZE * 2), %rdi
	/* No zero or search CHAR.  */
	testl	%eax, %eax
	jz	L(first_aligned_loop)

	/* If no zero CHAR then go to second loop (this allows us to
	   throw away all prior work).  */
	vpmovmskb %ymm8, %ecx
	testl	%ecx, %ecx
	jz	L(second_aligned_loop_prep)

	/* Search char could be zero so we need to get the true match.
	 */
	vpmovmskb %ymm5, %eax
	testl	%eax, %eax
	jnz	L(first_aligned_loop_return)

	.p2align 4,, 4
L(first_vec_x1_or_x2):
	VPCMPEQ	%ymm3, %ymm7, %ymm3
	VPCMPEQ	%ymm2, %ymm7, %ymm2
	vpmovmskb %ymm3, %eax
	vpmovmskb %ymm2, %edx
	/* Use add for macro-fusion.  */
	addq	%rax, %rdx
	jz	L(first_vec_x0_test)
	/* NB: We could move this shift to before the branch and save a
	   bit of code size / performance on the fall through. The
	   branch leads to the null case which generally seems hotter
	   than char in first 3x VEC.  */
	salq	$32, %rax
	addq	%rdx, %rax
	bsrq	%rax, %rax
	leaq	1(%rsi, %rax), %rax
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
	VZEROUPPER_RETURN

	.p2align 4,, 8
L(first_aligned_loop_return):
	VPCMPEQ	%ymm4, %ymm0, %ymm4
	vpmovmskb %ymm4, %edx
	salq	$32, %rcx
	orq	%rdx, %rcx

	vpmovmskb %ymm10, %eax
	vpmovmskb %ymm6, %edx
	salq	$32, %rax
	orq	%rdx, %rax
	blsmskq	%rcx, %rcx
	andq	%rcx, %rax
	jz	L(first_vec_x1_or_x2)

	bsrq	%rax, %rax
	leaq	-(VEC_SIZE * 2)(%rdi, %rax), %rax
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
	VZEROUPPER_RETURN

	/* Search char cannot be zero.  */
	.p2align 4
L(second_aligned_loop_set_furthest_match):
	/* Save VEC and pointer from most recent match.  */
L(second_aligned_loop_prep):
	movq	%rdi, %rsi
	vmovdqu	%ymm6, %ymm2
	vmovdqu	%ymm10, %ymm3

	.p2align 4
L(second_aligned_loop):
	/* Search 2x at at time.  */
	vmovdqa	(VEC_SIZE * 0)(%rdi), %ymm4
	vmovdqa	(VEC_SIZE * 1)(%rdi), %ymm5

	VPCMPEQ	%ymm4, %ymm7, %ymm6
	VPMIN	%ymm4, %ymm5, %ymm1
	VPCMPEQ	%ymm5, %ymm7, %ymm10
	vpor	%ymm6, %ymm10, %ymm5
	VPCMPEQ	%ymm1, %ymm0, %ymm1
	vpor	%ymm5, %ymm1, %ymm9

	vpmovmskb %ymm9, %eax
	addq	$(VEC_SIZE * 2), %rdi
	testl	%eax, %eax
	jz	L(second_aligned_loop)
	vpmovmskb %ymm1, %ecx
	testl	%ecx, %ecx
	jz	L(second_aligned_loop_set_furthest_match)
	vpmovmskb %ymm5, %eax
	testl	%eax, %eax
	jnz	L(return_new_match)

	/* This is the hot patch. We know CHAR is inbounds and that
	   ymm3/ymm2 have latest match.  */
	.p2align 4,, 4
L(return_old_match):
	vpmovmskb %ymm3, %eax
	vpmovmskb %ymm2, %edx
	salq	$32, %rax
	orq	%rdx, %rax
	bsrq	%rax, %rax
	/* Search char cannot be zero so safe to just use lea for
	   wcsrchr.  */
	leaq	(VEC_SIZE * -2 -(CHAR_SIZE - 1))(%rsi, %rax), %rax
	VZEROUPPER_RETURN

	/* Last iteration also potentially has a match.  */
	.p2align 4,, 8
L(return_new_match):
	VPCMPEQ	%ymm4, %ymm0, %ymm4
	vpmovmskb %ymm4, %edx
	salq	$32, %rcx
	orq	%rdx, %rcx

	vpmovmskb %ymm10, %eax
	vpmovmskb %ymm6, %edx
	salq	$32, %rax
	orq	%rdx, %rax
	blsmskq	%rcx, %rcx
	andq	%rcx, %rax
	jz	L(return_old_match)
	bsrq	%rax, %rax
	/* Search char cannot be zero so safe to just use lea for
	   wcsrchr.  */
	leaq	(VEC_SIZE * -2 -(CHAR_SIZE - 1))(%rdi, %rax), %rax
	VZEROUPPER_RETURN

	.p2align 4,, 4
L(cross_page):
	movq	%rdi, %rsi
	andq	$-VEC_SIZE, %rsi
	vmovdqu	(%rsi), %ymm1
	VPCMPEQ	%ymm1, %ymm0, %ymm6
	vpmovmskb %ymm6, %ecx
	/* Shift out zero CHAR matches that are before the begining of
	   src (rdi).  */
	shrxl	%edi, %ecx, %ecx
	testl	%ecx, %ecx
	jz	L(page_cross_continue)
	VPCMPEQ	%ymm1, %ymm7, %ymm1
	vpmovmskb %ymm1, %eax

	/* Shift out search CHAR matches that are before the begining of
	   src (rdi).  */
	shrxl	%edi, %eax, %eax
	blsmskl	%ecx, %ecx
	/* Check if any search CHAR match in range.  */
	andl	%ecx, %eax
	jz	L(ret2)
	bsrl	%eax, %eax
	addq	%rdi, %rax
# ifdef USE_AS_WCSRCHR
	andq	$-CHAR_SIZE, %rax
# endif
L(ret2):
	VZEROUPPER_RETURN
END(STRRCHR)
