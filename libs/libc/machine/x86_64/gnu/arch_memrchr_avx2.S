/* memrchr optimized with AVX2.
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

# ifndef MEMRCHR
#  define MEMRCHR				memrchr
# endif

# ifndef VZEROUPPER
#  define VZEROUPPER			vzeroupper
# endif

# ifndef SECTION
#  define SECTION(p)	p##.avx
# endif

# define VEC_SIZE			32
# define PAGE_SIZE			4096
	.section SECTION(.text), "ax", @progbits
ENTRY_P2ALIGN(MEMRCHR, 6)
# ifdef __ILP32__
	/* Clear upper bits.  */
	and	%RDX_LP, %RDX_LP
# else
	test	%RDX_LP, %RDX_LP
# endif
	jz	L(zero_0)

	vmovd	%esi, %xmm0
	/* Get end pointer. Minus one for two reasons. 1) It is necessary for a
	   correct page cross check and 2) it correctly sets up end ptr to be
	   subtract by lzcnt aligned.  */
	leaq	-1(%rdx, %rdi), %rax

	vpbroadcastb %xmm0, %ymm0

	/* Check if we can load 1x VEC without cross a page.  */
	testl	$(PAGE_SIZE - VEC_SIZE), %eax
	jz	L(page_cross)

	vpcmpeqb -(VEC_SIZE - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx
	cmpq	$VEC_SIZE, %rdx
	ja	L(more_1x_vec)

L(ret_vec_x0_test):
	/* If ecx is zero (no matches) lzcnt will set it 32 (VEC_SIZE) which
	   will gurantee edx (len) is less than it.  */
	lzcntl	%ecx, %ecx

	/* Hoist vzeroupper (not great for RTM) to save code size. This allows
	   all logic for edx (len) <= VEC_SIZE to fit in first cache line.  */
	COND_VZEROUPPER
	cmpl	%ecx, %edx
	jle	L(zero_0)
	subq	%rcx, %rax
	ret

	/* Fits in aligning bytes of first cache line.  */
L(zero_0):
	xorl	%eax, %eax
	ret

	.p2align 4,, 9
L(ret_vec_x0):
	lzcntl	%ecx, %ecx
	subq	%rcx, %rax
L(return_vzeroupper):
	ZERO_UPPER_VEC_REGISTERS_RETURN

	.p2align 4,, 10
L(more_1x_vec):
	testl	%ecx, %ecx
	jnz	L(ret_vec_x0)

	/* Align rax (string pointer).  */
	andq	$-VEC_SIZE, %rax

	/* Recompute remaining length after aligning.  */
	movq	%rax, %rdx
	/* Need this comparison next no matter what.  */
	vpcmpeqb -(VEC_SIZE)(%rax), %ymm0, %ymm1
	subq	%rdi, %rdx
	decq	%rax
	vpmovmskb %ymm1, %ecx
	/* Fall through for short (hotter than length).  */
	cmpq	$(VEC_SIZE * 2), %rdx
	ja	L(more_2x_vec)
L(last_2x_vec):
	cmpl	$VEC_SIZE, %edx
	jbe	L(ret_vec_x0_test)

	testl	%ecx, %ecx
	jnz	L(ret_vec_x0)

	vpcmpeqb -(VEC_SIZE * 2 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx
	/* 64-bit lzcnt. This will naturally add 32 to position.  */
	lzcntq	%rcx, %rcx
	COND_VZEROUPPER
	cmpl	%ecx, %edx
	jle	L(zero_0)
	subq	%rcx, %rax
	ret


	/* Inexpensive place to put this regarding code size / target alignments
	   / ICache NLP. Necessary for 2-byte encoding of jump to page cross
	   case which in turn is necessary for hot path (len <= VEC_SIZE) to fit
	   in first cache line.  */
L(page_cross):
	movq	%rax, %rsi
	andq	$-VEC_SIZE, %rsi
	vpcmpeqb (%rsi), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx
	/* Shift out negative alignment (because we are starting from endptr and
	   working backwards).  */
	movl	%eax, %r8d
	/* notl because eax already has endptr - 1.  (-x = ~(x - 1)).  */
	notl	%r8d
	shlxl	%r8d, %ecx, %ecx
	cmpq	%rdi, %rsi
	ja	L(more_1x_vec)
	lzcntl	%ecx, %ecx
	COND_VZEROUPPER
	cmpl	%ecx, %edx
	jle	L(zero_0)
	subq	%rcx, %rax
	ret
	.p2align 4,, 11
L(ret_vec_x1):
	/* This will naturally add 32 to position.  */
	lzcntq	%rcx, %rcx
	subq	%rcx, %rax
	VZEROUPPER_RETURN
	.p2align 4,, 10
L(more_2x_vec):
	testl	%ecx, %ecx
	jnz	L(ret_vec_x0)

	vpcmpeqb -(VEC_SIZE * 2 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx
	testl	%ecx, %ecx
	jnz	L(ret_vec_x1)


	/* Needed no matter what.  */
	vpcmpeqb -(VEC_SIZE * 3 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx

	subq	$(VEC_SIZE * 4), %rdx
	ja	L(more_4x_vec)

	cmpl	$(VEC_SIZE * -1), %edx
	jle	L(ret_vec_x2_test)

L(last_vec):
	testl	%ecx, %ecx
	jnz	L(ret_vec_x2)

	/* Needed no matter what.  */
	vpcmpeqb -(VEC_SIZE * 4 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx
	lzcntl	%ecx, %ecx
	subq	$(VEC_SIZE * 3), %rax
	COND_VZEROUPPER
	subq	%rcx, %rax
	cmpq	%rax, %rdi
	ja	L(zero_2)
	ret

	/* First in aligning bytes.  */
L(zero_2):
	xorl	%eax, %eax
	ret

	.p2align 4,, 4
L(ret_vec_x2_test):
	lzcntl	%ecx, %ecx
	subq	$(VEC_SIZE * 2), %rax
	COND_VZEROUPPER
	subq	%rcx, %rax
	cmpq	%rax, %rdi
	ja	L(zero_2)
	ret


	.p2align 4,, 11
L(ret_vec_x2):
	/* ecx must be non-zero.  */
	bsrl	%ecx, %ecx
	leaq	(VEC_SIZE * -3 + 1)(%rcx, %rax), %rax
	VZEROUPPER_RETURN

	.p2align 4,, 14
L(ret_vec_x3):
	/* ecx must be non-zero.  */
	bsrl	%ecx, %ecx
	leaq	(VEC_SIZE * -4 + 1)(%rcx, %rax), %rax
	VZEROUPPER_RETURN



	.p2align 4
L(more_4x_vec):
	testl	%ecx, %ecx
	jnz	L(ret_vec_x2)

	vpcmpeqb -(VEC_SIZE * 4 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx

	testl	%ecx, %ecx
	jnz	L(ret_vec_x3)

	/* Check if near end before re-aligning (otherwise might do an
	   unnecissary loop iteration).  */
	addq	$-(VEC_SIZE * 4), %rax
	cmpq	$(VEC_SIZE * 4), %rdx
	jbe	L(last_4x_vec)

	/* Align rax to (VEC_SIZE - 1).  */
	orq	$(VEC_SIZE * 4 - 1), %rax
	movq	%rdi, %rdx
	/* Get endptr for loop in rdx. NB: Can't just do while rax > rdi because
	   lengths that overflow can be valid and break the comparison.  */
	orq	$(VEC_SIZE * 4 - 1), %rdx

	.p2align 4
L(loop_4x_vec):
	/* Need this comparison next no matter what.  */
	vpcmpeqb -(VEC_SIZE * 1 - 1)(%rax), %ymm0, %ymm1
	vpcmpeqb -(VEC_SIZE * 2 - 1)(%rax), %ymm0, %ymm2
	vpcmpeqb -(VEC_SIZE * 3 - 1)(%rax), %ymm0, %ymm3
	vpcmpeqb -(VEC_SIZE * 4 - 1)(%rax), %ymm0, %ymm4

	vpor	%ymm1, %ymm2, %ymm2
	vpor	%ymm3, %ymm4, %ymm4
	vpor	%ymm2, %ymm4, %ymm4
	vpmovmskb %ymm4, %esi

	testl	%esi, %esi
	jnz	L(loop_end)

	addq	$(VEC_SIZE * -4), %rax
	cmpq	%rdx, %rax
	jne	L(loop_4x_vec)

	subl	%edi, %edx
	incl	%edx

L(last_4x_vec):
	/* Used no matter what.  */
	vpcmpeqb -(VEC_SIZE * 1 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx

	cmpl	$(VEC_SIZE * 2), %edx
	jbe	L(last_2x_vec)

	testl	%ecx, %ecx
	jnz	L(ret_vec_x0_end)

	vpcmpeqb -(VEC_SIZE * 2 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx
	testl	%ecx, %ecx
	jnz	L(ret_vec_x1_end)

	/* Used no matter what.  */
	vpcmpeqb -(VEC_SIZE * 3 - 1)(%rax), %ymm0, %ymm1
	vpmovmskb %ymm1, %ecx

	cmpl	$(VEC_SIZE * 3), %edx
	ja	L(last_vec)

	lzcntl	%ecx, %ecx
	subq	$(VEC_SIZE * 2), %rax
	COND_VZEROUPPER
	subq	%rcx, %rax
	cmpq	%rax, %rdi
	jbe	L(ret0)
	xorl	%eax, %eax
L(ret0):
	ret


	.p2align 4
L(loop_end):
	vpmovmskb %ymm1, %ecx
	testl	%ecx, %ecx
	jnz	L(ret_vec_x0_end)

	vpmovmskb %ymm2, %ecx
	testl	%ecx, %ecx
	jnz	L(ret_vec_x1_end)

	vpmovmskb %ymm3, %ecx
	/* Combine last 2 VEC matches. If ecx (VEC3) is zero (no CHAR in VEC3)
	   then it won't affect the result in esi (VEC4). If ecx is non-zero
	   then CHAR in VEC3 and bsrq will use that position.  */
	salq	$32, %rcx
	orq	%rsi, %rcx
	bsrq	%rcx, %rcx
	leaq	(VEC_SIZE * -4 + 1)(%rcx, %rax), %rax
	VZEROUPPER_RETURN

	.p2align 4,, 4
L(ret_vec_x1_end):
	/* 64-bit version will automatically add 32 (VEC_SIZE).  */
	lzcntq	%rcx, %rcx
	subq	%rcx, %rax
	VZEROUPPER_RETURN

	.p2align 4,, 4
L(ret_vec_x0_end):
	lzcntl	%ecx, %ecx
	subq	%rcx, %rax
	VZEROUPPER_RETURN

	/* 2 bytes until next cache line.  */
END(MEMRCHR)
